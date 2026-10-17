//! 飞特（FeeTech SCS / HD-1910）半双工总线后端。
//!
//! 本 crate 最终为官方控制层 `duck-control` 提供 `RobotIo` 的飞特实现，让官方 50Hz 控制环
//! 驱动 15 颗 HD-1910 磁编码舵机 + 200 号 IMU 板，而官方的策略、观测、控制逻辑一行不改。
//!
//! 模块划分：
//! - [`wire`]：帧编解码、寄存器地址、单位换算。只处理字节，不碰串口、不依赖硬件，因此全部
//!   编解码都能在开发机离线单测，且与我们实测跑通的 servo-web Python 实现逐字节对齐。
//! - [`bus`]：[`bus::Ft1910Io`]，实现 `RobotIo`，一次 SYNC_READ 读回 15 舵机 + IMU，并内含
//!   200 号 IMU 板的 ft6/native 就绪判据（#19）。
//! - 标定：[`bus::Calibration`] 从 TOML 读取（启动默认 `/etc/robot/ft1910.toml`，可用环境变量
//!   `FT1910_CONFIG` 覆盖；随仓库模板见 `config/ft1910.toml`），含关节方向/零位、PD、IMU
//!   人格与安装朝向。robotd 开 `ft1910` feature 即选用本后端。

pub mod bus;
pub mod wire;

pub use bus::{Calibration, Ft1910Io, JointCalibration, Transport};
