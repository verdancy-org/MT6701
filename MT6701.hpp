#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: MT6701 SSI angle sensor driver
constructor_args: []
template_args: []
required_hardware: [mt6701_spi, mt6701_spi_cs]
depends: []
=== END MANIFEST === */
// clang-format on

#include <atomic>
#include <cstdint>

#include "app_framework.hpp"
#include "crc.hpp"
#include "gpio.hpp"
#include "libxr_def.hpp"
#include "spi.hpp"

class MT6701 : public LibXR::Application
{
 public:
  MT6701(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app)
      : mt6701_spi_(hw.template FindOrExit<LibXR::SPI>({"mt6701_spi"})),
        mt6701_spi_cs_(hw.template FindOrExit<LibXR::GPIO>({"mt6701_spi_cs"})),
        spi_done_cb_(LibXR::Callback<LibXR::ErrorCode>::Create(
            [](bool in_isr, MT6701* self, LibXR::ErrorCode err)
            { self->OnTransferDone(in_isr, err); }, this)),
        spi_op_(spi_done_cb_)
  {
    UNUSED(app);

    mt6701_spi_cs_->Write(true);
    ASSERT(ConfigureSPI() == LibXR::ErrorCode::OK);
    Start();
  }

  void OnMonitor() override {}

  void Start()
  {
    const bool WAS_RUNNING = running_.exchange(true, std::memory_order_acq_rel);
    if (!WAS_RUNNING)
    {
      TryStartTransfer(false);
    }
  }

  void Stop() { running_.store(false, std::memory_order_release); }

  [[nodiscard]] float ReadAngleRad()
  {
    CacheData cache = {};
    if (LoadCache(cache) != LibXR::ErrorCode::OK)
    {
      return DEFAULT_ANGLE_RAD;
    }
    return cache.angle_rad;
  }

  [[nodiscard]] bool IsOverspeed() { return (ReadRawMg() & 0x08u) != 0u; }

  [[nodiscard]] bool IsPushDetected() { return (ReadRawMg() & 0x04u) != 0u; }

  [[nodiscard]] bool IsFieldTooStrong()
  {
    return static_cast<uint8_t>(ReadRawMg() & 0x03u) == 1u;
  }

  [[nodiscard]] bool IsFieldTooWeak()
  {
    return static_cast<uint8_t>(ReadRawMg() & 0x03u) == 2u;
  }

  [[nodiscard]] uint8_t GetMagnetStrength()
  {
    return static_cast<uint8_t>(ReadRawMg() & 0x03u);
  }

  [[nodiscard]] uint8_t GetRawMg() { return ReadRawMg(); }

 private:
  static constexpr uint32_t FRAME_MASK = 0x00FFFFFFu;
  static constexpr uint32_t PAYLOAD_MASK = 0x0003FFFFu;
  static constexpr uint32_t ANGLE_RESOLUTION = 16384u;
  static constexpr float DEFAULT_ANGLE_RAD = 0.0f;
  static constexpr float RAD_PER_LSB =
      6.28318530717958647692f / static_cast<float>(ANGLE_RESOLUTION);

  LibXR::ErrorCode ConfigureSPI() const
  {
    auto cfg = mt6701_spi_->GetConfig();
    cfg.clock_polarity = LibXR::SPI::ClockPolarity::HIGH;
    cfg.clock_phase = LibXR::SPI::ClockPhase::EDGE_1;
    return mt6701_spi_->SetConfig(cfg);
  }

  struct CacheData
  {
    float angle_rad = DEFAULT_ANGLE_RAD;
    uint8_t raw_mg = 0u;
  };

  [[nodiscard]] LibXR::ErrorCode LoadCache(CacheData& cache)
  {
    if (!cache_valid_.load(std::memory_order_acquire))
    {
      return LibXR::ErrorCode::EMPTY;
    }

    while (true)
    {
      const uint32_t SEQ_BEGIN = cache_seq_.load(std::memory_order_acquire);
      if ((SEQ_BEGIN & 1u) != 0u)
      {
        continue;
      }
      cache = cache_data_;
      const uint32_t SEQ_END = cache_seq_.load(std::memory_order_acquire);
      if (SEQ_BEGIN == SEQ_END)
      {
        return LibXR::ErrorCode::OK;
      }
    }
  }

  [[nodiscard]] uint8_t ReadRawMg()
  {
    CacheData cache = {};
    if (LoadCache(cache) != LibXR::ErrorCode::OK)
    {
      return 0u;
    }
    return cache.raw_mg;
  }

  static bool DecodeFrame(uint32_t frame24, float& angle_rad)
  {
    const uint8_t CRC_RX = static_cast<uint8_t>(frame24 & 0x3Fu);
    const uint8_t CRC_CALC =
        LibXR::CRC6::CalculateBits((frame24 >> 6) & PAYLOAD_MASK, 18);
    if (CRC_RX != CRC_CALC)
    {
      return false;
    }

    const uint16_t RAW14 = static_cast<uint16_t>((frame24 >> 10) & 0x3FFFu);
    angle_rad = static_cast<float>(RAW14) * RAD_PER_LSB;
    return true;
  }

  void StoreSample(float angle_rad, uint8_t raw_mg)
  {
    cache_seq_.fetch_add(1u, std::memory_order_acq_rel);
    cache_data_.angle_rad = angle_rad;
    cache_data_.raw_mg = static_cast<uint8_t>(raw_mg & 0x0Fu);
    cache_seq_.fetch_add(1u, std::memory_order_release);
    cache_valid_.store(true, std::memory_order_release);
  }

  void OnTransferDone(bool in_isr, LibXR::ErrorCode err)
  {
    mt6701_spi_cs_->Write(true);

    if (err != LibXR::ErrorCode::OK)
    {
      transfer_pending_.store(false, std::memory_order_release);
      running_.store(false, std::memory_order_release);
      ASSERT_FROM_CALLBACK(false, in_isr);
      return;
    }

    uint32_t frame24 = (static_cast<uint32_t>(rx_buf_[0]) << 16) |
                       (static_cast<uint32_t>(rx_buf_[1]) << 8) |
                       static_cast<uint32_t>(rx_buf_[2]);
    frame24 &= FRAME_MASK;

    float angle_rad = DEFAULT_ANGLE_RAD;
    const uint8_t MG_RAW = static_cast<uint8_t>((frame24 >> 6) & 0x0Fu);
    if (DecodeFrame(frame24, angle_rad))
    {
      StoreSample(angle_rad, MG_RAW);
    }

    transfer_pending_.store(false, std::memory_order_release);

    if (running_.load(std::memory_order_acquire))
    {
      TryStartTransfer(in_isr);
    }
  }

  void TryStartTransfer(bool in_isr)
  {
    if (!running_.load(std::memory_order_acquire))
    {
      return;
    }

    bool expected = false;
    if (!transfer_pending_.compare_exchange_strong(
            expected, true, std::memory_order_acq_rel, std::memory_order_acquire))
    {
      return;
    }

    mt6701_spi_cs_->Write(false);
    LibXR::ErrorCode err = mt6701_spi_->ReadAndWrite(
        LibXR::RawData(rx_buf_, sizeof(rx_buf_)),
        LibXR::ConstRawData(tx_buf_, sizeof(tx_buf_)), spi_op_, in_isr);
    if (err != LibXR::ErrorCode::OK)
    {
      mt6701_spi_cs_->Write(true);
      transfer_pending_.store(false, std::memory_order_release);
      running_.store(false, std::memory_order_release);
      ASSERT_FROM_CALLBACK(false, in_isr);
    }
    if (in_isr && !transfer_pending_.load(std::memory_order_acquire))
    {
      running_.store(false, std::memory_order_release);
      ASSERT_FROM_CALLBACK(false, in_isr);
    }
  }

  LibXR::SPI* mt6701_spi_;
  LibXR::GPIO* mt6701_spi_cs_;
  LibXR::Callback<LibXR::ErrorCode> spi_done_cb_;
  LibXR::SPI::OperationRW spi_op_;
  std::atomic<uint32_t> cache_seq_{0u};
  std::atomic<bool> cache_valid_{false};
  std::atomic<bool> running_{false};
  std::atomic<bool> transfer_pending_{false};
  CacheData cache_data_ = {};
  uint8_t tx_buf_[3] = {0u, 0u, 0u};
  uint8_t rx_buf_[3] = {0u, 0u, 0u};
};
