//! 飞特（FeeTech SCS / HD-1910）半双工总线后端。
//!
//! 本 crate 最终为官方控制层 `duck-control` 提供 `RobotIo` 的飞特实现，让官方 50Hz 控制环
//! 驱动 15 颗 HD-1910 磁编码舵机 + 200 号 IMU 板，而官方的策略、观测、控制逻辑一行不改。
//!
//! 模块划分：
//! - [`wire`]：帧编解码、寄存器地址、单位换算。只处理字节，不碰串口、不依赖硬件，因此全部
//!   编解码都能在开发机离线单测，且与我们实测跑通的 servo-web Python 实现逐字节对齐。
//! - [`bus`]：[`bus::Ft1910Io`]，实现 `RobotIo`，负责一次 SYNC_READ 读回 15 舵机 + IMU。
//! - （后续）`imu`：200 号 IMU 板的 ft6 状态判据与挂载复核（#19，当前先复用官方解码器）。
//! - （后续）`config`：关节映射、零位标定、PD 与 IMU 配置文件（#21）。

pub mod bus;
pub mod wire;

pub use bus::{Calibration, Ft1910Io, JointCalibration, Transport};
