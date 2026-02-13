# MT6701

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://github.com/verdancy-org/MT6701)
[![Language](https://img.shields.io/badge/language-C++17-orange.svg)](https://en.cppreference.com/)
[![Framework](https://img.shields.io/badge/Framework-LibXR-green)](https://github.com/Jiu-xiao/libxr)
[![GitHub stars](https://img.shields.io/github/stars/verdancy-org/MT6701?style=social)](https://github.com/verdancy-org/MT6701/stargazers)
[![GitHub forks](https://img.shields.io/github/forks/verdancy-org/MT6701?style=social)](https://github.com/verdancy-org/MT6701/network/members)
[![GitHub issues](https://img.shields.io/github/issues/verdancy-org/MT6701)](https://github.com/verdancy-org/MT6701/issues)

**MT6701** 是一个面向嵌入式实时控制场景的 SSI 磁编码器驱动模块，基于 LibXR `Application` 架构实现。

## 模块介绍

该模块专注 MT6701 的 SPI/SSI 数据链路，采用“SPI 完成回调自驱 + seqlock 读缓存”模式，目标是让控制环路以无阻塞方式稳定获取角度。  
驱动在写端完成 CRC 过滤，仅把有效帧写入缓存；读端只消费最近一次有效结果，不承担通信错误分支。

## 特性

- 仅依赖 `LibXR::SPI` 与 `LibXR::GPIO`，硬件抽象层清晰，跨平台迁移成本低。
- 遵循 `Application` 生命周期，构造后自动启动采样链，`OnMonitor()` 保持空实现。
- SSI 帧解析内置：`D[13:0] + Mg[3:0] + CRC[5:0]`，CRC6 多项式 `x^6 + x + 1`。
- 读接口无阻塞、无错误码分支，适配高频控制循环（FOC/PID）。
- 状态查询 API 直接基于 `raw_mg` 按位解析，接口简单且执行开销小。

## 硬件要求

- 芯片工作在 SSI 模式（MODE 选择到 I2C/SSI 功能组）。
- 默认硬件别名：
  - `mt6701_spi`（SPI 设备）
  - `mt6701_spi_cs`（CS GPIO）
- `mt6701_spi` 连接 MT6701 的 `DO/CLK`，`mt6701_spi_cs` 连接 `CSN`（低有效）。
- 本模块不包含 I2C 路径。

## 构造参数

- MANIFEST 构造参数：
  - `spi_alias`（默认 `"mt6701_spi"`）
  - `cs_alias`（默认 `"mt6701_spi_cs"`）
- 构造函数签名：
  - `MT6701(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app, const char* spi_alias = "mt6701_spi", const char* cs_alias = "mt6701_spi_cs")`
- 构造阶段行为：
  - 按构造参数查找 SPI 与 CS 设备
  - 设置 SPI 必需参数：`CPOL=HIGH`、`CPHA=EDGE_1`
  - 调用 `Start()` 启动连续采样

## 接口参考

```cpp
void Start();
void Stop();

float Read();

bool IsOverspeed();
bool IsPushDetected();
bool IsFieldTooStrong();
bool IsFieldTooWeak();
uint8_t GetMagnetStrength();  // 0=normal, 1=too strong, 2=too weak, 3=reserved
uint8_t GetRawMg();
```

## 依赖

- 无额外第三方依赖（除 LibXR 基础框架外）。
