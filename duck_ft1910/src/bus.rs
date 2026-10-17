//! 飞特 HD-1910 总线后端 [`Ft1910Io`]：对官方控制层实现 [`RobotIo`]。
//!
//! 这是第 ① 层「总线后端」的主体。它只做一件事：把官方 `RobotIo` 的每 20ms 一次
//! `read()` / `write()`，翻译成飞特半双工总线上的一帧 SYNC_READ / SYNC_WRITE，并把
//! 舵机和 IMU 的原始字节换回官方的 [`Sensors`] / [`JointTargets`]。
//!
//! 设计上刻意对齐官方的 DYNAMIXEL 后端：
//! - 一次 `SYNC_READ`（地址 56、15 字节）把 16 个设备（200 号 IMU 在前，15 颗舵机随后）
//!   一起读回来，应答按设备 id 归档而不是按到达顺序——半双工总线上顺序并不可靠。
//! - IMU 前 12 字节与官方 `SflpDecoder` 完全同格式，**直接复用**官方解码器，不重写。
//! - 位置/速度/电流的单位换算、符号-幅值解码都在 [`crate::wire`]，这里只做零位与方向。
//!
//! 串口被 [`Transport`] 抽象隔离：真机用 [`SerialTransport`]，离线单测用一个回放字节的
//! 假传输层，所以 `read()` / `write()` 的整条链路在开发机上就能验证，不必上板。
//!
//! 写路径（#18）：目标位置用一帧 SYNC_WRITE 广播，P/D 增益只写 RAM 50/51（重启后从
//! EEPROM 重载，从不写 EEPROM）。扭矩开关（寄存器 40）**必须逐颗 WRITE 并等应答**——
//! 实测 SYNC_WRITE 写 40 不生效。上使能严格按「读当前姿态 → 写增益并校验 → 把目标贴成
//! 当前位置并校验 → 逐颗上扭矩 → 读回」的顺序，让吸合瞬间每颗舵机的误差为 0，避免
//! 15 颗同时冲向旧目标、电流涌入触发板子欠压复位。
//!
//! 50Hz 时序预算（1M 波特，10µs/字节，20ms 一 tick）：每 tick 一次 SYNC_READ（16 设备 ×
//! 15 字节，约 3.6ms）+ 一次广播 SYNC_WRITE（约 0.53ms）≈ 4.1ms，占周期约 21%。增益不变时
//! set_gain 一个字节都不发；增益切换那一 tick 额外写+读回约 3.2ms，仍 < 20ms。缺设备时
//! burst 收齐其余应答后按 BURST_IDLE(4ms) 收摊、整总线全断才吃满 BURST_LIMIT(12ms)，都不会
//! 顶破一个 tick。慢传感器（电压/温度）的飞特地址 62/63 已落在每 tick 的状态块内，
//! slow_sensors() 直接复用 read 缓存、零额外总线（不像 Dynamixel 要每秒补一次读）。

use std::array;
use std::time::{Duration, Instant};

use duck_control::imu::{SflpDecoder, IMU_BLOCK_LEN};
use duck_control::io::{ImuStale, IoError, JointTargets, Result, RobotIo, Sensors, SlowSensors};
use duck_control::model::{JOINT_IDS, JOINT_NAMES, MOUTH_INDEX, NUM_JOINTS};
use serde::{Deserialize, Serialize};

use crate::wire::{
    self, block, ft6_status, gains_block, imu_block, imu_identity, native_status, reg,
    ExpectedReply, FrameReader,
};

// ── 总线时序常量 ─────────────────────────────────────────────────────────────

/// 飞特总线固定 1M 波特。
pub const BAUD_RATE: u32 = 1_000_000;

/// 半双工突发窗口：收到第一个字节后，等待同一批应答里下一个设备的最长间隔。
///
/// 太短会把一次 burst 的后续设备误判成超时；太长会让单个缺失设备拖满 20ms 周期。
/// 4ms 是飞特单帧应答间隔的实测余量。
const BURST_IDLE: Duration = Duration::from_millis(4);

/// 一次突发的总预算上限：发出请求后，超过这个时间就收摊、按缺设备报错。
const BURST_LIMIT: Duration = Duration::from_millis(12);

// ── 标定 ─────────────────────────────────────────────────────────────────────

/// 单颗关节的机械标定。
///
/// 我们的零位做法与社区版默认略有不同：调试阶段已经用飞特舵机自身的 `0x0B CAL` 指令
/// 把每颗舵机的机械零位**烧进舵机内部 EEPROM**（校完后舵机在机械零位自报 2048），所以
/// 主机端 `zero_ticks` 恒为 2048，不需要再为每颗舵机维护一张偏移表；主机只剩一个
/// 装配方向 `direction`（个别关节反装时为 −1）。字段仍保留，重装或某颗舵机偏移丢失时
/// 可在配置里覆盖。
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct JointCalibration {
    /// 机械零位对应的原始计数（我们内部 CAL 后恒为 2048）。
    pub zero_ticks: i32,
    /// 装配方向：正装 +1，反装 −1。
    pub direction: f64,
    /// 软件行程限位（原始计数），写目标时裁剪，防止标错方向时把腿怼到极限。
    pub limits: (i32, i32),
}

impl Default for JointCalibration {
    fn default() -> Self {
        Self {
            zero_ticks: 2048,
            direction: 1.0,
            limits: (0, 4095),
        }
    }
}

/// 飞特 P/D 增益档位（写 RAM 寄存器 50/51，掉电不保留）。
///
/// 控制环传下来的 `gain` 是 XL330 的单位（200 标称、50 软），不能原样发给飞特：直接
/// 按该值写 P 会比出厂增益大约 20 倍。这里保留一组飞特自己的基准 P，再把策略增益按
/// 比例缩放到它附近，保留「增益越大越硬、软倒时确实软」的含义，但不假装两族舵机共用单位。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct PdProfile {
    /// 普通关节在策略标称增益（200）下的 P（飞特单位）。
    pub kp: u8,
    /// 所有关节固定的 D。
    pub kd: u8,
    /// 嘴机构比腿轻得多，单独给一个更大的 P。
    pub mouth_kp: u8,
}

impl Default for PdProfile {
    fn default() -> Self {
        Self {
            kp: 6,
            kd: 20,
            mouth_kp: 10,
        }
    }
}

impl PdProfile {
    /// 某关节在给定策略增益下应写入的 `[P, D]` 两个字节。
    ///
    /// 四舍五入到最近，下限钳到 1：P=0 不是「软」，而是开环失控；上限 255。
    pub fn registers(&self, joint: usize, policy_gain: u16) -> [u8; 2] {
        let base = if joint == MOUTH_INDEX {
            u32::from(self.mouth_kp.max(1))
        } else {
            u32::from(self.kp.max(1))
        };
        let scaled = (base * u32::from(policy_gain) + 100) / 200;
        [scaled.clamp(1, 255) as u8, self.kd]
    }
}

/// IMU 固件人格：状态字节位定义、就绪判据和安装朝向随人格而不同。
///
/// 我们的社区 IMU 板刷的是 ft6 v1.6.0 固件，默认 [`ImuPersonality::Ft6`]；
/// 人格也可由 [`Ft1910Io::detect_personality`] 读 ID 200 身份窗自动识别。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum ImuPersonality {
    /// 飞特 ft6 人格（v1.6.0 起默认）：就绪 = (byte13 & 0x07)==0；12 个控制字节已在板端
    /// 做安装预旋转，主机 mount 用单位四元数。配置文件里写 `imu_personality = "Ft6"`。
    #[default]
    Ft6,
    /// community-kit native 人格（v1.4.x 契约）：byte13 的 0x01/0x80 是好事，就绪靠
    /// 官方解码器累计有效四元数；状态位极性与 ft6 相反。
    Native,
}

/// 整机标定：IMU 地址/人格、15 个关节的零位/方向、增益档位、IMU 安装朝向、波特率。
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Calibration {
    /// IMU 在总线上的舵机 id（伪装成舵机）。
    pub imu_id: u8,
    pub joints: [JointCalibration; NUM_JOINTS],
    /// 飞特 RAM P/D 增益档位（可在配置文件里覆盖）。
    pub pd: PdProfile,
    /// IMU 固件人格（我们的板子默认 ft6 v1.6.0）。
    pub imu_personality: ImuPersonality,
    /// IMU 安装四元数 `[w, x, y, z]`，把板载坐标系转到躯干坐标系。
    ///
    /// ft6 固件（v1.6.0）已在**板端**做了安装预旋转（X、Y 反向，输出即躯干坐标），
    /// 主机端不再施加旋转，所以用单位四元数。
    pub imu_mount: [f64; 4],
    pub baud_rate: u32,
}

impl Default for Calibration {
    fn default() -> Self {
        Self {
            imu_id: 200,
            joints: array::from_fn(|_| JointCalibration::default()),
            pd: PdProfile::default(),
            imu_personality: ImuPersonality::default(),
            imu_mount: [1.0, 0.0, 0.0, 0.0],
            baud_rate: BAUD_RATE,
        }
    }
}

impl Calibration {
    /// 打开总线前校验一遍，避免把一张方向/零位越界的标定带到机器人上。
    pub fn validate(&self) -> std::result::Result<(), String> {
        if self.imu_id == wire::BROADCAST_ID {
            return Err("IMU id 不能是广播地址 0xFE".into());
        }
        for (i, joint) in self.joints.iter().enumerate() {
            if joint.direction != 1.0 && joint.direction != -1.0 {
                return Err(format!("关节 {} 的 direction 只能是 +1 或 −1", JOINT_NAMES[i]));
            }
            if !(0..=4095).contains(&joint.zero_ticks) {
                return Err(format!("关节 {} 的 zero_ticks 超出单圈范围", JOINT_NAMES[i]));
            }
            if joint.limits.0 < 0
                || joint.limits.1 > 4095
                || joint.limits.0 > joint.limits.1
            {
                return Err(format!("关节 {} 的行程限位非法", JOINT_NAMES[i]));
            }
        }
        Ok(())
    }

    /// 板子上飞特标定文件的默认位置，与 robotd.toml 同目录（/etc/robot）。
    pub const CONFIG_PATH: &str = "/etc/robot/ft1910.toml";

    /// 从 TOML 文件读取并校验一张标定。文件缺失、解析失败或数值越界都返回错误说明。
    pub fn from_file(path: &str) -> std::result::Result<Self, String> {
        let text = std::fs::read_to_string(path)
            .map_err(|e| format!("读取飞特标定文件 {path} 失败: {e}"))?;
        let cal: Calibration = toml::from_str(&text)
            .map_err(|e| format!("解析飞特标定文件 {path} 失败: {e}"))?;
        cal.validate()?;
        Ok(cal)
    }

    /// 启动加载：优先环境变量 `FT1910_CONFIG`，否则用 [`Self::CONFIG_PATH`]。
    ///
    /// 读不到或不合法都返回错误，**不静默回退到默认方向**——默认标定方向全是 +1，而本机
    /// 实测 13 个关节为 −1，静默回退会让腿往反方向动。调用方（robotd 的 open_bus）把它当作
    /// 总线未就绪、继续重试等待，和舵机没上电走同一条路径。
    pub fn load_for_boot() -> std::result::Result<Self, String> {
        let path =
            std::env::var("FT1910_CONFIG").unwrap_or_else(|_| Self::CONFIG_PATH.to_owned());
        Self::from_file(&path)
    }
}

// ── 传输层抽象（串口可替换，便于离线测试）─────────────────────────────────────

/// 半双工总线的最小收发能力。真机是串口，测试是回放字节的假实现。
pub trait Transport: Send {
    fn write_all(&mut self, buf: &[u8]) -> std::io::Result<()>;
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize>;
    fn set_timeout(&mut self, timeout: Duration) -> std::io::Result<()>;
}

/// 真实串口传输。`serialport` 的 `SerialPort` 本身继承了 `Send`，可以整个放进
/// `Box<dyn Transport>` 跨线程移动。
pub struct SerialTransport {
    port: Box<dyn serialport::SerialPort>,
}

impl Transport for SerialTransport {
    fn write_all(&mut self, buf: &[u8]) -> std::io::Result<()> {
        // 故意不 flush：板子上 flush 会固定耗掉 ~12ms。
        std::io::Write::write_all(&mut self.port, buf)
    }

    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        std::io::Read::read(&mut self.port, buf)
    }

    fn set_timeout(&mut self, timeout: Duration) -> std::io::Result<()> {
        // serialport::Error 通过 `?` 转成 std::io::Error（serialport 提供了 From）。
        Ok(self.port.set_timeout(timeout)?)
    }
}

// ── IMU 陈旧计数 ─────────────────────────────────────────────────────────────

/// 前 12 字节连续多少帧完全相同就判姿态冻结（50Hz 下半秒），沿用官方 STALE_RUN_WARN。
const FROZEN_RUN: u64 = 25;
/// ft6 四元数序号连续 3 个相同（= 2 次重复）即判节点不再出姿态（协议 §16.3）。
const FT6_SEQ_STALE_RUN: u64 = 2;

/// 跟踪 IMU 是否在刷新。上报口径严格对齐官方 `StaleImuTracker` / [`ImuStale`]：只比
/// **前 12 字节窗口**（陀螺 + 四元数，不含 byte12 序号）。
///
/// * `stale_total`：陈旧读数累计——只有当前块与上一块前 12 字节逐字节相同才 +1，第一块
///   不计数、永不复位；对应 [`ImuStale::total`]。
/// * `window_run`：当前连续重复长度，任一新鲜块归零；对应 [`ImuStale::run`]，上层用
///   `FROZEN_RUN` 判姿态冻结。
///
/// 另有一路 ft6 独有的 `seq_run`：四元数序号（byte12）连续相同长度，比窗口更早发现板端
/// 停发（连续 3 个即过期，§16.3）。它只驱动 ft6 分支的 `tracing::warn`，**不进 [`ImuStale`]**：
/// 上层 `FROZEN_RUN` 阈值是按窗口重复标定的，把序号塞进去会让健康指标含义错乱。
#[derive(Debug, Default)]
struct StaleTracker {
    /// 前 12 字节窗口重复累计次数（= ImuStale.total），仅重复时累加，永不复位。
    stale_total: u64,
    last_seq: Option<u8>,
    /// 四元数序号连续重复长度（仅用于 ft6 §16.3 的 warn）。
    seq_run: u64,
    last_window: Option<[u8; IMU_BLOCK_LEN]>,
    /// 前 12 字节连续重复长度（= ImuStale.run）。
    window_run: u64,
}

impl StaleTracker {
    fn new() -> Self {
        Self::default()
    }

    /// 观测一帧：返回 (序号重复 run, 窗口重复 run)，新鲜均为 0。
    fn observe(&mut self, window: &[u8; IMU_BLOCK_LEN], seq: u8) -> (u64, u64) {
        if self.last_seq == Some(seq) {
            self.seq_run = self.seq_run.saturating_add(1);
        } else {
            self.last_seq = Some(seq);
            self.seq_run = 0;
        }
        if self.last_window == Some(*window) {
            self.stale_total = self.stale_total.saturating_add(1);
            self.window_run = self.window_run.saturating_add(1);
        } else {
            self.last_window = Some(*window);
            self.window_run = 0;
        }
        (self.seq_run, self.window_run)
    }

    /// 上报给控制环：严格对齐官方 [`ImuStale`] 的前 12 字节窗口语义。
    fn snapshot(&self) -> ImuStale {
        ImuStale {
            total: self.stale_total,
            run: self.window_run,
        }
    }
}

// ── 后端主体 ─────────────────────────────────────────────────────────────────

/// 飞特 HD-1910 的 [`RobotIo`] 实现。
pub struct Ft1910Io {
    transport: Box<dyn Transport>,
    /// 仅用于错误信息，让人知道是哪条串口在失败。
    path: String,
    calibration: Calibration,
    /// 一次 SYNC_READ 点名的设备，顺序固定为 IMU 在前、15 颗舵机随后。
    ids: Vec<u8>,
    decoder: SflpDecoder,
    stale: StaleTracker,
    /// 当前按哪种固件人格解析状态字节。
    personality: ImuPersonality,
    imu_status: u8,
    /// 最近一帧 IMU 是否就绪（ft6 由状态位决定，native 由解码器有效四元数计数决定）。
    imu_ready_flag: bool,
    /// 最近一次写入的策略增益（XL330 单位）。初始为 0，确保第一次 `set_gain` 一定真的下发。
    gain: u16,
    /// 每颗舵机 RAM 里的 P/D 是否确实等于当前 profile；舵机重启、检测到漂移、写失败都会
    /// 清掉它，绝不假设。
    gains_verified: bool,
    /// 15 颗舵机是否已全部确认使能扭矩。松脱时 [`RobotIo::write`] 不发目标。
    enabled: bool,
    /// 最近一次成功 read() 顺带解析出的电压/温度。飞特寄存器 62(电压)/63(温度) 落在每 tick
    /// 已读的 15 字节状态块（地址 56..70）里，所以慢传感器不额外发总线帧。首次成功 read 前为 None。
    last_slow: Option<SlowSensors>,
    io_timeout: Duration,
}

impl Ft1910Io {
    /// 打开一条串口并构造后端。
    pub fn open(path: &str, calibration: Calibration) -> Result<Self> {
        let port = serialport::new(path, calibration.baud_rate)
            .data_bits(serialport::DataBits::Eight)
            .stop_bits(serialport::StopBits::One)
            .parity(serialport::Parity::None)
            .timeout(BURST_LIMIT)
            .open()
            .map_err(|source| IoError::Port {
                path: path.to_owned(),
                source: source.into(),
            })?;
        Self::with_transport(Box::new(SerialTransport { port }), path.to_owned(), calibration)
    }

    /// 用任意传输层构造（主要给离线测试注入假传输层）。
    pub fn with_transport(
        transport: Box<dyn Transport>,
        path: String,
        calibration: Calibration,
    ) -> Result<Self> {
        calibration.validate().map_err(IoError::Bus)?;
        let mut ids = Vec::with_capacity(NUM_JOINTS + 1);
        ids.push(calibration.imu_id);
        ids.extend(JOINT_IDS.iter().copied());
        Ok(Self {
            transport,
            path,
            calibration,
            ids,
            decoder: SflpDecoder::new(calibration.imu_mount),
            stale: StaleTracker::new(),
            personality: calibration.imu_personality,
            imu_status: 0,
            imu_ready_flag: false,
            gain: 0,
            gains_verified: false,
            enabled: false,
            last_slow: None,
            io_timeout: BURST_LIMIT,
        })
    }

    /// 官方 `DynamixelIo::interpolate_to` 的飞特等价实现：从当前实际位置线性插值到
    /// `target`，分 `step` 步、每步阻塞写入。只被 `robotd init` 调用（开扭矩后、回 home
    /// 姿势前）；控制环从不调用它。
    pub fn interpolate_to(
        &mut self,
        target: &[f64; NUM_JOINTS],
        duration: Duration,
        step: Duration,
    ) -> Result<()> {
        let start = self.read()?.positions;
        let steps = (duration.as_secs_f64() / step.as_secs_f64())
            .ceil()
            .max(1.0) as u32;
        for i in 1..=steps {
            let t = i as f64 / steps as f64;
            let mut next = [0.0; NUM_JOINTS];
            for j in 0..NUM_JOINTS {
                next[j] = start[j] + (target[j] - start[j]) * t;
            }
            self.write(&JointTargets::new(next))?;
            std::thread::sleep(step);
        }
        Ok(())
    }


    /// 把串口错误包成带端口路径的 [`IoError::Port`]。
    fn io_err(&self, source: std::io::Error) -> IoError {
        IoError::Port {
            path: self.path.clone(),
            source,
        }
    }

    /// 发出一帧请求，按 id 收集所有应答数据块。
    ///
    /// 半双工总线上应答可能晚到、被截断、夹杂我们自己写帧的回声：靠 [`FrameReader`] 按
    /// 「id + 确切应答长度」重组，收齐后再统一检查缺失设备。
    fn request(&mut self, frame: &[u8], expected: &ExpectedReply) -> Result<Vec<Vec<u8>>> {
        self.transport.write_all(frame).map_err(|e| self.io_err(e))?;

        let mut reader = FrameReader::new();
        let mut blocks: Vec<Option<Vec<u8>>> = vec![None; expected.ids().len()];
        let mut buf = [0u8; 256];

        // 第一次读给满窗口；收到任意字节后缩短到 burst 间隔，尽快收摊。
        self.transport.set_timeout(BURST_LIMIT).map_err(|e| self.io_err(e))?;
        let started = Instant::now();

        loop {
            match self.transport.read(&mut buf) {
                Ok(n) => {
                    if n > 0 {
                        reader.push(&buf[..n]);
                        self.transport.set_timeout(BURST_IDLE).ok();
                    }
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {
                    if reader.buffered() == 0 {
                        break;
                    }
                    self.transport.set_timeout(BURST_IDLE).ok();
                }
                Err(e) => return Err(self.io_err(e)),
            }

            while let Some(raw_frame) = reader.next_frame(expected) {
                let ack =
                    wire::parse_ack(&raw_frame).map_err(|e| IoError::Bus(format!("{e}（{}）", self.path)))?;
                if ack.status != 0 {
                    // 舵机在报警（过载/过温），但帧本身是合法的：先记录，不直接判失败。
                    tracing::warn!(id = ack.id, status = ack.status, "舵机上报非零状态");
                }
                if ack.data.len() != expected.reply_data_len() {
                    return Err(IoError::ShortRead {
                        what: "总线状态块",
                        expected: expected.reply_data_len(),
                        got: ack.data.len(),
                    });
                }
                let slot = expected
                    .ids()
                    .iter()
                    .position(|id| *id == ack.id)
                    .ok_or_else(|| IoError::Bus(format!("收到未点名设备 id {} 的应答", ack.id)))?;
                // 同一 id 重复应答只留第一份。
                if blocks[slot].is_none() {
                    blocks[slot] = Some(ack.data);
                }
            }

            if started.elapsed() >= self.io_timeout {
                break;
            }
        }

        let missing: Vec<String> = blocks
            .iter()
            .enumerate()
            .filter(|(_, data)| data.is_none())
            .map(|(slot, _)| {
                let id = expected.ids()[slot];
                format!("id {id} {}", joint_of(id))
            })
            .collect();
        if !missing.is_empty() {
            return Err(IoError::Bus(format!(
                "总线上没有响应的设备：{}（{}）",
                missing.join("，"),
                self.path
            )));
        }

        Ok(blocks.into_iter().map(|data| data.unwrap()).collect())
    }

    /// 原始位置计数 → 关节弧度（减零位、乘装配方向）。
    fn radians(&self, joint: usize, raw: u16) -> f64 {
        let cal = &self.calibration.joints[joint];
        (wire::sign15(raw) as f64 - cal.zero_ticks as f64) * wire::RAD_PER_COUNT * cal.direction
    }

    /// 关节弧度 → 目标原始字（`radians` 的逆变换），并钳进该关节的软件行程窗口。
    ///
    /// 钳制放在主机而不是交给舵机限位寄存器：越界目标舵机会拒绝或折叠，而一个已绕到单圈
    /// 外的关节会被读成好几圈误差。
    fn ticks_for(&self, joint: usize, radians: f64) -> u16 {
        let cal = &self.calibration.joints[joint];
        let counts =
            cal.zero_ticks + (radians * cal.direction / wire::RAD_PER_COUNT).round() as i32;
        let clamped = counts.clamp(cal.limits.0, cal.limits.1);
        wire::position_counts_to_raw(clamped)
    }

    /// 只对 15 颗关节（不含 IMU 节点）做一次 SYNC_READ，返回各关节数据块。
    fn read_joint_blocks(&mut self, addr: u8, len: usize) -> Result<Vec<Vec<u8>>> {
        let frame = wire::sync_read(&JOINT_IDS, addr, len as u8);
        let expected = ExpectedReply::new(&JOINT_IDS, len);
        self.request(&frame, &expected)
    }

    /// 读 IMU 节点寄存器 0..4 的身份窗，识别并记住固件人格。
    ///
    /// ft6 应答 `06 00 00 00 F2`、native 应答 `01 00 00 49 4D`。读不到或无法识别时返回
    /// 错误且不改变当前人格（构造时按标定文件的 `imu_personality`，默认 ft6）。这是一次
    /// 额外的总线读，默认不自动调用，交给上层在需要时显式识别。
    pub fn detect_personality(&mut self) -> Result<ImuPersonality> {
        let id = self.calibration.imu_id;
        let frame = wire::read(id, imu_identity::ADDR, imu_identity::LEN);
        let expected = ExpectedReply::new(&[id], imu_identity::LEN as usize);
        let blocks = self.request(&frame, &expected)?;
        let ident = blocks[0].as_slice();
        let personality = if ident == imu_identity::FT6.as_slice() {
            ImuPersonality::Ft6
        } else if ident == imu_identity::NATIVE.as_slice() {
            ImuPersonality::Native
        } else {
            return Err(IoError::Bus(format!(
                "无法识别 IMU 身份窗：{ident:02x?}（ft6 应为 {:02x?}，native 应为 {:02x?}）",
                imu_identity::FT6, imu_identity::NATIVE
            )));
        };
        self.personality = personality;
        tracing::info!(?personality, "识别到 IMU 固件人格");
        Ok(personality)
    }

    /// 当前 profile + 最新策略增益下，每个关节应写入的 `[P, D]`。
    fn want_gains(&self) -> Vec<[u8; 2]> {
        (0..NUM_JOINTS)
            .map(|joint| self.calibration.pd.registers(joint, self.gain))
            .collect()
    }

    /// 读回 RAM P/D，逐关节比对是否等于 want。
    fn verify_gains(&mut self, want: &[[u8; 2]]) -> Result<bool> {
        let blocks = self.read_joint_blocks(reg::KP, gains_block::LEN)?;
        Ok(blocks.iter().zip(want).all(|(block, want)| {
            block[gains_block::KP] == want[0] && block[gains_block::KD] == want[1]
        }))
    }

    /// 一帧 SYNC_WRITE 把 15 个关节的 RAM P/D 写下去，再读回校验；不符就报错。
    fn write_gains(&mut self) -> Result<()> {
        let rows: Vec<[u8; 2]> = self.want_gains();
        let refs: Vec<&[u8]> = rows.iter().map(|r| r.as_slice()).collect();
        let frame = wire::sync_write(&JOINT_IDS, reg::KP, &refs)
            .map_err(|e| IoError::Bus(format!("飞特增益帧构造失败: {e}")))?;
        self.transport.write_all(&frame).map_err(|e| self.io_err(e))?;
        self.gains_verified = self.verify_gains(&rows)?;
        if !self.gains_verified {
            return Err(IoError::Bus(
                "P/D 写下去后读回不一致，拒绝使能扭矩".to_owned(),
            ));
        }
        Ok(())
    }

    /// 幂等：只在尚未确认时写一次 RAM P/D。
    fn ensure_gains(&mut self) -> Result<()> {
        if self.gains_verified {
            return Ok(());
        }
        self.write_gains()
    }

    /// 一次 SYNC_READ 读回 15 颗的扭矩使能位。
    fn torque_enabled(&mut self) -> Result<Vec<bool>> {
        let blocks = self.read_joint_blocks(reg::TORQUE_ENABLE, 1)?;
        Ok(blocks.iter().map(|b| b[0] != 0).collect())
    }

    /// 扭矩开关，逐颗 WRITE 并各等一次应答——刻意**不用** SYNC_WRITE。
    ///
    /// 本机实测：SYNC_WRITE 到寄存器 40 不粘（立刻读回是 0，几秒后又自己较劲），只有
    /// 直接 WRITE 到 40 才可靠。代价是 15 次往返（约 5ms），只在使能/松脱/重启时发生，
    /// 不在 20ms 控制 tick 上。
    fn write_torque(&mut self, on: bool) -> Result<()> {
        let value = [u8::from(on)];
        for &id in JOINT_IDS.iter() {
            let frame = wire::write(id, reg::TORQUE_ENABLE, &value);
            let expected = ExpectedReply::new(&[id], 0);
            self.request(&frame, &expected)
                .map_err(|e| IoError::Bus(format!("id {id} 扭矩开关写下去没有被应答: {e}")))?;
        }
        Ok(())
    }

    /// 一帧 SYNC_WRITE 广播 15 个目标位置（广播写，设备不答）。
    fn send_goals(&mut self, targets: &JointTargets) -> Result<()> {
        let rows: Vec<[u8; 2]> = (0..NUM_JOINTS)
            .map(|joint| self.ticks_for(joint, targets.positions[joint]).to_le_bytes())
            .collect();
        let refs: Vec<&[u8]> = rows.iter().map(|r| r.as_slice()).collect();
        let frame = wire::sync_write(&JOINT_IDS, reg::GOAL_POSITION_L, &refs)
            .map_err(|e| IoError::Bus(format!("飞特目标帧构造失败: {e}")))?;
        self.transport.write_all(&frame).map_err(|e| self.io_err(e))
    }

    /// 把目标贴成给定姿态并读回逐颗校验。**在扭矩吸合之前**调用，所以不能走
    /// [`RobotIo::write`]（那个在未使能时会被守卫挡掉）。
    fn write_and_verify_goals(&mut self, targets: &JointTargets) -> Result<()> {
        self.send_goals(targets)?;
        let want: Vec<u16> = (0..NUM_JOINTS)
            .map(|joint| self.ticks_for(joint, targets.positions[joint]))
            .collect();
        let blocks = self.read_joint_blocks(reg::GOAL_POSITION_L, 2)?;
        for (joint, block) in blocks.iter().enumerate() {
            let got = wire::le_u16(block[0], block[1]);
            if got != want[joint] {
                return Err(IoError::Bus(format!(
                    "{}: 目标位置读回 {got}，写下去的是 {}，拒绝给一颗没到位的舵机上扭矩",
                    JOINT_NAMES[joint], want[joint]
                )));
            }
        }
        Ok(())
    }
}

impl RobotIo for Ft1910Io {
    fn read(&mut self) -> Result<Sensors> {
        // IMU 在前、15 颗舵机随后，地址 56、读 15 字节。
        let frame = wire::sync_read(&self.ids, reg::PRESENT_POSITION_L, block::LEN as u8);
        let expected = ExpectedReply::new(&self.ids, block::LEN);
        let blocks = self.request(&frame, &expected)?;

        let mut sensors = Sensors::default();

        // 第一个块是 IMU（地址 56、15 字节）：前 12 字节（陀螺 + 半精度四元数）格式与官方
        // v2 板相同，直接交给官方解码器；byte12/13/14 是飞特人格特有的序号/状态/保留。
        let imu = &blocks[0];
        if imu.len() < imu_block::LEN {
            return Err(IoError::ShortRead {
                what: "IMU 控制块",
                expected: imu_block::LEN,
                got: imu.len(),
            });
        }
        let mut raw = [0u8; IMU_BLOCK_LEN];
        raw.copy_from_slice(&imu[..IMU_BLOCK_LEN]);
        let status = imu[imu_block::STATUS];
        let count = imu[imu_block::COUNT];
        let reserved = imu[imu_block::RESERVED];
        self.imu_status = status;

        // 陈旧统计先于形状校验：即便块畸形也记一次窗口/序号（与 Python 参考实现一致）。
        let (seq_run, window_run) = self.stale.observe(&raw, count);
        // 板端还没写出 SFLP 表时，四元数 6 字节（raw[6..12]）全 0。
        let quat_all_zero = imu[6..imu_block::CTRL_LEN].iter().all(|b| *b == 0);

        // 形状校验在前：畸形块直接 Err，且不交给解码器，避免垃圾四元数污染中值滤波。
        let mut native_sensor_err = false;
        match self.personality {
            ImuPersonality::Ft6 => {
                // 形状：reserved 必须 0、状态高 4 位必须 0，否则按 §16.3 拒绝整块。
                if reserved != 0 || (status & ft6_status::SHAPE_MASK) != 0 {
                    return Err(IoError::Bus(format!(
                        "IMU ft6 块形状错误：reserved=0x{reserved:02x}（要求 0）、状态高4位=0x{:01x}（要求 0），拒绝整块",
                        status & ft6_status::SHAPE_MASK
                    )));
                }
                if status & ft6_status::SENSOR_ERR != 0 {
                    tracing::error!(
                        status,
                        "IMU 报告传感器错误（ft6 bit2 SENSOR_ERR）：LSM6DSV16X 通信/数据异常"
                    );
                }
                // ready = bit0/1/2 全 0 且四元数非全 0；bit3 READER_SLOW 是信息位，不参与。
                self.imu_ready_flag = (status & ft6_status::READY_MASK) == 0 && !quat_all_zero;
                if seq_run == FT6_SEQ_STALE_RUN
                    || (seq_run > FT6_SEQ_STALE_RUN && seq_run.is_multiple_of(500))
                {
                    tracing::warn!(
                        consecutive = seq_run,
                        "IMU 四元数序号连续相同，节点不再产生新姿态（ft6 §16.3）"
                    );
                }
            }
            ImuPersonality::Native => {
                if reserved != 0 {
                    return Err(IoError::Bus(format!(
                        "IMU 保留字节 = 0x{reserved:02x}，规格要求 0"
                    )));
                }
                native_sensor_err = status & native_status::SENSOR_ERR != 0;
                if native_sensor_err {
                    tracing::error!(status, "IMU 报告传感器错误（native SENSOR_ERR）");
                }
            }
        }

        // 只有通过形状校验的合法块才交给解码器：全 0 四元数时它沿用上一帧姿态，不谎报直立。
        sensors.imu = self.decoder.decode(&raw);
        if self.personality == ImuPersonality::Native {
            // native 没有可靠的 ready 位（0x01/0x80 是好事），沿用官方解码器：累计满 25 个
            // 有效四元数，全 0 未出表时解码器本身不计数；SENSOR_ERR 与 ft6 一样 gate 就绪。
            self.imu_ready_flag = self.decoder.ready() && !quat_all_zero && !native_sensor_err;
        }

        // 前 12 字节冻结检测（两种人格共用）：到阈值报一次，之后每 500 帧报一次。
        if window_run == FROZEN_RUN || (window_run > FROZEN_RUN && window_run.is_multiple_of(500))
        {
            tracing::warn!(
                consecutive = window_run,
                "IMU 前 12 字节连续相同，姿态冻结（orientation is frozen）"
            );
        }

        // 其余 15 块按关节表顺序填充。read() 是 all-or-nothing：走到这里 15 块都齐，电压/
        // 温度也一并缓存，供每秒一次的 slow_sensors() 复用，不再单独发总线帧。
        let mut temps_c = [0.0f64; NUM_JOINTS];
        let mut volts_sum = 0.0f64;
        let mut volts_n = 0u32;
        for joint in 0..NUM_JOINTS {
            let blk = &blocks[joint + 1];
            if blk.len() < block::LEN {
                return Err(IoError::ShortRead {
                    what: "关节状态块",
                    expected: block::LEN,
                    got: blk.len(),
                });
            }
            let position = wire::le_u16(blk[block::POSITION], blk[block::POSITION + 1]);
            let velocity = wire::le_u16(blk[block::SPEED], blk[block::SPEED + 1]);
            let current = wire::le_u16(blk[block::CURRENT], blk[block::CURRENT + 1]);
            let direction = self.calibration.joints[joint].direction;

            sensors.positions[joint] = self.radians(joint, position);
            sensors.velocities[joint] =
                wire::sign15(velocity) as f64 * wire::RAD_PER_SEC_PER_SPEED_COUNT * direction;
            // 电流取无符号绝对值，负载是正还是负对热和限流没有意义。
            sensors.currents_ma[joint] =
                wire::sign15(current).unsigned_abs() as f64 * wire::MA_PER_CURRENT_COUNT;

            // 电压(寄存器 62、块内偏移 6) 单位 0.1V；温度(寄存器 63、偏移 7) 直接是 ℃、无偏移。
            // 15 颗在同一电池上，电压平均降噪；读数 0 是上电瞬间的无意义值，丢弃不平均。
            let volt_raw = blk[block::VOLTAGE];
            if volt_raw != 0 {
                volts_sum += f64::from(volt_raw) * wire::VOLTS_PER_COUNT;
                volts_n += 1;
            }
            temps_c[joint] = f64::from(blk[block::TEMPERATURE]);
        }
        // 至少一颗读到非零电压才刷新慢传感器样本；全 0（上电瞬间）时保留上一帧。
        if volts_n > 0 {
            self.last_slow = Some(SlowSensors {
                volts: volts_sum / f64::from(volts_n),
                temps_c,
            });
        }

        Ok(sensors)
    }

    fn write(&mut self, targets: &JointTargets) -> Result<()> {
        // 刚松脱的舵机不能收到目标：实测扭矩刚关时若又来目标，舵机会悄悄重新上劲，等于把
        // 一次 relax 抵消掉。增益写不会这样，只有目标写需要这个守卫。
        if !self.enabled {
            return Ok(());
        }
        self.send_goals(targets)
    }

    fn set_gain(&mut self, kp: u16) -> Result<()> {
        // 每个 tick 都会调；增益没变且已校验，就一个字节都不发，省掉 20ms 周期里的总线占用。
        if kp == self.gain && self.gains_verified {
            return Ok(());
        }
        // 变化的增益双向都照写（站→走→软倒本来就要调刚度）；写失败则回滚，避免对外报告
        // 一个其实没落地的新增益，让慢变量巡检每轮都去重写。
        let previous = self.gain;
        self.gain = kp;
        self.gains_verified = false;
        if let Err(e) = self.ensure_gains() {
            self.gain = previous;
            self.gains_verified = false;
            return Err(e);
        }
        Ok(())
    }

    fn set_torque(&mut self, on: bool) -> Result<()> {
        if !on {
            self.write_torque(false)?;
            let states = self.torque_enabled()?;
            self.enabled = false;
            // 有舵机松不开是这里唯一值得报错的失败：人没法扶鸭子，relax 等于没发生。
            let stuck: Vec<String> = states
                .iter()
                .enumerate()
                .filter(|(_, energised)| **energised)
                .map(|(joint, _)| format!("id {} {}", JOINT_IDS[joint], JOINT_NAMES[joint]))
                .collect();
            if !stuck.is_empty() {
                return Err(IoError::Bus(format!(
                    "扭矩关后这些舵机仍使能，人无法扶动: {}",
                    stuck.join(", ")
                )));
            }
            return Ok(());
        }

        // 上电顺序不可调换：若目标寄存器还留着旧目标，一上扭矩 15 颗会同时冲过去，
        // 最大 26°×15 的误差涌入电流曾在 8 分钟内 4 次触发板子欠压锁存、复位单板机。
        // 先读出机器人现在的姿态，把增益、目标都贴到当前，让误差为 0 再吸合扭矩。
        let present = self.read()?;
        self.ensure_gains()?;
        self.write_and_verify_goals(&JointTargets::new(present.positions))?;

        self.write_torque(true)?;
        let states = self.torque_enabled()?;
        let off: Vec<String> = states
            .iter()
            .enumerate()
            .filter(|(_, energised)| !**energised)
            .map(|(joint, _)| format!("id {} {}", JOINT_IDS[joint], JOINT_NAMES[joint]))
            .collect();
        if !off.is_empty() {
            // 全或无：部分使能意味着部分关节硬撑、其余瘫软，而控制环并不知道是哪些。
            self.enabled = false;
            return Err(IoError::Bus(format!(
                "部分舵机未使能、已保持松弛，松开舵机后重试: {}",
                off.join(", ")
            )));
        }
        self.enabled = true;
        Ok(())
    }

    fn reboot(&mut self, id: u8) -> Result<()> {
        if !JOINT_IDS.contains(&id) {
            return Err(IoError::Bus(format!("reboot: {id} 不是关节舵机")));
        }
        // 先关扭矩：舵机重启后带着 EEPROM 增益、却可能仍朝旧目标使劲，带电重启等于一颗
        // 没有控制环保持的关节。
        self.transport
            .write_all(&wire::write(id, reg::TORQUE_ENABLE, &[0]))
            .map_err(|e| self.io_err(e))?;
        self.transport
            .write_all(&wire::reboot(id))
            .map_err(|e| self.io_err(e))?;
        // 重启不回应答（实测约 823ms 后回来），等待与重新使能由调用方负责。RAM 增益随之
        // 丢失（重启用 EEPROM 21/22 覆盖 50/51），标记失配，下次上使能时重新写。
        self.gains_verified = false;
        self.enabled = false;
        Ok(())
    }

    /// 电压/温度复用每 tick 状态块（地址 62/63 在 56..70 内），**不额外占用总线**。
    /// 控制环每秒调一次；尚未成功 read 过时返回 Err，调用方保留上一个样本（与官方后端
    /// 一次偶发读失败时保留上次采样的处理一致）。
    fn slow_sensors(&mut self) -> Result<SlowSensors> {
        self.last_slow.ok_or_else(|| {
            IoError::Bus("尚无成功的总线 read，慢传感器（电压/温度）暂无数据".into())
        })
    }

    // 就绪在每帧 read() 里按人格更新：ft6 看状态字节 (byte13 & 0x07)==0，native 看解码器
    // 累计的有效四元数。这里只返回最近一帧的结论。
    fn imu_ready(&self) -> bool {
        self.imu_ready_flag
    }

    fn imu_stale(&self) -> ImuStale {
        self.stale.snapshot()
    }
}

/// 舵机 id → 关节名（仅用于错误信息）；IMU 单独叫 "IMU"。
fn joint_of(id: u8) -> String {
    JOINT_IDS
        .iter()
        .position(|joint| *joint == id)
        .map(|i| JOINT_NAMES[i].to_owned())
        .unwrap_or_else(|| "IMU".to_owned())
}

// ── 离线单测：用回放字节的假传输层把整条 read 解析链跑通 ───────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wire;
    use std::sync::{Arc, Mutex};

    /// 构造一个设备的应答帧（status=0，带 data），校验和算好。
    fn ack_frame(id: u8, data: &[u8]) -> Vec<u8> {
        let len = data.len() as u8 + 2;
        let mut sum = id.wrapping_add(len); // status 字节为 0
        for b in data {
            sum = sum.wrapping_add(*b);
        }
        let mut frame = vec![0xFF, 0xFF, id, len, 0x00];
        frame.extend_from_slice(data);
        frame.push(!sum);
        frame
    }

    /// 拼一个舵机的 15 字节状态块（只填位置/速度/电流，其余为 0）。
    /// 位置/速度/电流都是 15 位符号-幅值。
    fn servo_block(position_raw: u16, speed_raw: u16, current_raw: u16) -> Vec<u8> {
        let mut b = vec![0u8; block::LEN];
        b[block::POSITION] = position_raw.to_le_bytes()[0];
        b[block::POSITION + 1] = position_raw.to_le_bytes()[1];
        b[block::SPEED] = speed_raw.to_le_bytes()[0];
        b[block::SPEED + 1] = speed_raw.to_le_bytes()[1];
        b[block::CURRENT] = current_raw.to_le_bytes()[0];
        b[block::CURRENT + 1] = current_raw.to_le_bytes()[1];
        b
    }

    /// 拼 IMU 的 15 字节块：前 6 字节陀螺（i16），6..12 半精度四元数 xyz，
    /// 12 帧序号、13 状态、14 保留。
    fn imu_block(gyro: [i16; 3], quat_half: [u16; 3], count: u8, status: u8) -> Vec<u8> {
        let mut b = vec![0u8; block::LEN];
        for (i, g) in gyro.iter().enumerate() {
            let lo = (i * 2) as usize;
            b[lo..lo + 2].copy_from_slice(&g.to_le_bytes());
        }
        for (i, q) in quat_half.iter().enumerate() {
            let lo = 6 + i * 2;
            b[lo..lo + 2].copy_from_slice(&q.to_le_bytes());
        }
        b[imu_block::COUNT] = count;
        b[imu_block::STATUS] = status;
        b
    }

    /// 回放预设字节的假传输层，同时记录主机发出的每一帧。
    ///
    /// 供数按「一次期待应答的请求 → 一个 burst」的边界：只有 READ/WRITE/SYNC_READ/PING
    /// 这类舵机会回答的指令才激活下一个 burst；SYNC_WRITE/REBOOT 等广播或无应答指令不
    /// 激活（写路径里夹杂的广播写若也激活，会把后续应答的序号错位）。该 burst 放完后
    /// `read` 超时——和真实半双工总线「设备只应答一次、随后空闲」一致。
    struct ScriptedTransport {
        bursts: Vec<Vec<u8>>,
        next_burst: usize,
        active: Option<(usize, usize)>,
        written: Arc<Mutex<Vec<Vec<u8>>>>,
    }
    impl ScriptedTransport {
        fn new(bursts: Vec<Vec<u8>>) -> Self {
            Self {
                bursts,
                next_burst: 0,
                active: None,
                written: Arc::new(Mutex::new(Vec::new())),
            }
        }
        /// 同 new，但额外返回一个写帧日志的共享句柄，测试断言主机到底发了什么。
        fn with_log(bursts: Vec<Vec<u8>>) -> (Self, Arc<Mutex<Vec<Vec<u8>>>>) {
            let transport = Self::new(bursts);
            let log = transport.written.clone();
            (transport, log)
        }
    }
    /// 该指令物理上是否会有应答：广播写/重启/动作不答，读/单写/同步读/ping 会答。
    fn frame_expects_reply(frame: &[u8]) -> bool {
        // 飞特帧 [FF FF ID LEN INST ...]，指令在第 5 字节。
        frame.len() >= 5
            && !matches!(
                frame[4],
                wire::inst::SYNC_WRITE
                    | wire::inst::REG_WRITE
                    | wire::inst::ACTION
                    | wire::inst::REBOOT
            )
    }
    impl Transport for ScriptedTransport {
        fn write_all(&mut self, buf: &[u8]) -> std::io::Result<()> {
            self.written.lock().unwrap().push(buf.to_vec());
            if frame_expects_reply(buf) {
                // 每发一帧期待应答的请求，激活它对应的下一个 burst。
                if self.next_burst < self.bursts.len() {
                    self.active = Some((self.next_burst, 0));
                    self.next_burst += 1;
                } else {
                    self.active = None;
                }
            }
            Ok(())
        }
        fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
            fn timed_out() -> std::io::Error {
                std::io::Error::new(std::io::ErrorKind::TimedOut, "当前 burst 已放完")
            }
            let Some((burst_index, pos)) = self.active else {
                return Err(timed_out());
            };
            let data = &self.bursts[burst_index];
            if pos >= data.len() {
                return Err(timed_out());
            }
            let n = buf.len().min(data.len() - pos);
            buf[..n].copy_from_slice(&data[pos..pos + n]);
            self.active = Some((burst_index, pos + n));
            Ok(n)
        }
        fn set_timeout(&mut self, _timeout: Duration) -> std::io::Result<()> {
            Ok(())
        }
    }

    /// 构造后端：每个 burst = 1 个 IMU 块 + 15 个相同舵机块（IMU 在前）；`imu_blocks`
    /// 有几项，就准备几个 burst，供连续多次 read（喂饱官方解码器的中值滤波）。
    fn backend_with(imu_blocks: Vec<Vec<u8>>, servo: &[u8], calibration: Calibration) -> Ft1910Io {
        let mut bursts = Vec::new();
        for imu in &imu_blocks {
            let mut burst = ack_frame(calibration.imu_id, imu);
            for id in JOINT_IDS.iter() {
                burst.extend(ack_frame(*id, servo));
            }
            bursts.push(burst);
        }
        Ft1910Io::with_transport(
            Box::new(ScriptedTransport::new(bursts)),
            "synthetic".to_owned(),
            calibration,
        )
        .expect("标定合法")
    }

    #[test]
    fn read_解析15舵机加imu() {
        // 零位 2048、方向 +1：位置 +100，速度 +10，电流 −20（验证取绝对值）。
        let servo = servo_block(2048 + 100, 10, 0x8000 | 20);
        // 陀螺全 0；四元数 x≈0.125（半精度 0x3000），y=z=0（绕 x 前倾）。
        // 序号递增、quat 低位也逐帧微抖（0x3000/1/2）：真芯片静置时低位照样在跳，这样前
        // 12 字节窗口每帧不同，不会被窗口冻结计数误判；3 点中值 settle 到 0x3001（仍≈0.125）。
        let imu_blocks: Vec<Vec<u8>> = (1u8..=3)
            .map(|c| imu_block([0, 0, 0], [0x3000 + u16::from(c - 1), 0, 0], c, 0x00))
            .collect();

        let mut io = backend_with(imu_blocks, &servo, Calibration::default());
        let mut sensors = Sensors::default();
        for _ in 0..3 {
            sensors = io.read().expect("一次 read 应成功");
        }

        for joint in 0..NUM_JOINTS {
            assert!(
                (sensors.positions[joint] - 100.0 * wire::RAD_PER_COUNT).abs() < 1e-12,
                "关节 {joint} 位置不对"
            );
            assert!(
                (sensors.velocities[joint] - 10.0 * wire::RAD_PER_SEC_PER_SPEED_COUNT).abs()
                    < 1e-12,
                "关节 {joint} 速度不对"
            );
            assert!(
                (sensors.currents_ma[joint] - 20.0 * wire::MA_PER_CURRENT_COUNT).abs() < 1e-9,
                "关节 {joint} 电流应对称取绝对值"
            );
        }

        // 陀螺为 0；四元数从第一帧起就是新姿态（无中值），x≈0.125。
        for axis in 0..3 {
            assert!(sensors.imu.gyro[axis].abs() < 1e-9);
        }
        assert!(
            (sensors.imu.quat[1] - 0.125).abs() < 1e-3,
            "四元数 x 应被解码为 0.125"
        );
        // 重力经中值后 settle：绕 x 前倾使重力在 yz 平面倾斜，x 分量保持 0，y 分量非零。
        let g = sensors.imu.gravity;
        let norm = (g[0] * g[0] + g[1] * g[1] + g[2] * g[2]).sqrt();
        assert!((norm - 1.0).abs() < 1e-6, "重力必须是单位向量");
        assert!(g[0].abs() < 1e-6, "绕 x 前倾时重力 x 分量应为 0");
        assert!(g[1].abs() > 0.1, "中值 settle 后重力 y 应被转出倾斜");

        // 3 帧窗口逐帧不同、序号递增：没有任何陈旧读数，total/run 都为 0。
        let stale = io.imu_stale();
        assert_eq!(stale.total, 0);
        assert_eq!(stale.run, 0);
    }

    #[test]
    fn read_应用零位与反向方向() {
        // 自定义：零位 2000、全部反装 direction = −1。
        let mut calibration = Calibration::default();
        for joint in calibration.joints.iter_mut() {
            joint.zero_ticks = 2000;
            joint.direction = -1.0;
        }
        // 位置 2100：相对零位 +100，再乘 −1，应得 −100 计数。
        let servo = servo_block(2100, 0, 0);
        let imu_blocks = vec![imu_block([0, 0, 0], [0x3000, 0, 0], 1, 0x00)];

        let mut io = backend_with(imu_blocks, &servo, calibration);
        let sensors = io.read().expect("read 应成功");
        for joint in 0..NUM_JOINTS {
            assert!(
                (sensors.positions[joint] + 100.0 * wire::RAD_PER_COUNT).abs() < 1e-12,
                "关节 {joint} 应套用零位与反向方向"
            );
        }
    }

    #[test]
    fn read_缺设备时报总线错误() {
        // 只放一个含 IMU 的 burst，15 颗舵机全部缺席。
        let imu = imu_block([0, 0, 0], [0x3000, 0, 0], 1, 0x00);
        let burst = ack_frame(200, &imu);

        let mut io = Ft1910Io::with_transport(
            Box::new(ScriptedTransport::new(vec![burst])),
            "synthetic".to_owned(),
            Calibration::default(),
        )
        .expect("标定合法");

        let err = io.read().expect_err("缺设备应报错");
        assert!(matches!(err, IoError::Bus(msg) if msg.contains("没有响应的设备")));
    }

    #[test]
    fn slow_sensors_首次read前无数据_成功后复用状态块() {
        // 15 颗电压都读 120（=12.0V）、温度 35℃。
        let mut servo = servo_block(2048, 0, 0);
        servo[block::VOLTAGE] = 120;
        servo[block::TEMPERATURE] = 35;
        let imu_blocks = vec![imu_block([0, 0, 0], [0x3000, 0, 0], 1, 0x00)];

        let mut io = backend_with(imu_blocks, &servo, Calibration::default());
        assert!(io.slow_sensors().is_err(), "尚未 read 时应无慢传感器样本");

        io.read().expect("read 应成功");
        let slow = io.slow_sensors().expect("read 后应能取到慢传感器");
        assert!((slow.volts - 12.0).abs() < 1e-9, "电压 120×0.1 应 = 12.0V");
        for joint in 0..NUM_JOINTS {
            assert!(
                (slow.temps_c[joint] - 35.0).abs() < 1e-9,
                "关节 {joint} 温度应为 35℃"
            );
        }
    }

    #[test]
    fn slow_sensors_全零电压帧不产生也不覆盖样本() {
        // 第一帧电压 120（正常），第二帧 15 颗电压全 0（上电瞬间的无意义读数）。
        let mut good = servo_block(2048, 0, 0);
        good[block::VOLTAGE] = 120;
        good[block::TEMPERATURE] = 35;
        let dead = servo_block(2048, 0, 0); // 电压/温度字节默认 0
        let mk_burst = |servo: &[u8], count: u8| {
            // 四元数随帧微抖，保证前 12 字节窗口逐帧不同。
            let mut burst = ack_frame(
                200,
                &imu_block([0, 0, 0], [0x3000 + u16::from(count), 0, 0], count, 0x00),
            );
            for id in JOINT_IDS.iter() {
                burst.extend(ack_frame(*id, servo));
            }
            burst
        };
        let bursts = vec![mk_burst(&good, 1), mk_burst(&dead, 2)];

        let mut io = Ft1910Io::with_transport(
            Box::new(ScriptedTransport::new(bursts)),
            "synthetic".to_owned(),
            Calibration::default(),
        )
        .expect("标定合法");

        io.read().expect("第一帧正常");
        io.read().expect("第二帧全 0 电压");
        let slow = io.slow_sensors().expect("全 0 帧应保留上一帧样本，而非清空");
        assert!((slow.volts - 12.0).abs() < 1e-9, "全 0 帧不应覆盖已有电压");
        assert!(
            (slow.temps_c[0] - 35.0).abs() < 1e-9,
            "全 0 帧不应覆盖已有温度"
        );
    }

    #[test]
    fn 标定_非法方向被拒绝() {
        let mut calibration = Calibration::default();
        calibration.joints[0].direction = 0.5;
        assert!(Ft1910Io::with_transport(
            Box::new(ScriptedTransport::new(vec![])),
            "synthetic".to_owned(),
            calibration,
        )
        .is_err());
    }

    /// 15 颗关节各一块的 SYNC_READ 应答 burst（块顺序与 JOINT_IDS 一致）。
    fn joint_blocks_burst(blocks: &[Vec<u8>]) -> Vec<u8> {
        let mut burst = Vec::new();
        for (id, block) in JOINT_IDS.iter().zip(blocks) {
            burst.extend(ack_frame(*id, block));
        }
        burst
    }

    /// 15 个单颗空应答（每颗扭矩 WRITE 各一个）的 burst 列表，顺序按 JOINT_IDS。
    fn torque_ack_bursts() -> Vec<Vec<u8>> {
        JOINT_IDS.iter().map(|id| ack_frame(*id, &[])).collect()
    }

    /// 一次完整 read()（IMU + 15 舵机，舵机块相同）的应答 burst。
    fn full_read_burst(servo: &[u8]) -> Vec<u8> {
        let mut burst = ack_frame(200, &imu_block([0, 0, 0], [0, 0, 0], 1, 0x00));
        for id in JOINT_IDS.iter() {
            burst.extend(ack_frame(*id, servo));
        }
        burst
    }

    /// 15 个关节的 gains 块，内容等于当前 profile 在给定策略增益下的期望值。
    fn gain_blocks(policy_gain: u16, cal: &Calibration) -> Vec<Vec<u8>> {
        (0..NUM_JOINTS)
            .map(|j| {
                let mut b = vec![0u8; gains_block::LEN];
                let pd = cal.pd.registers(j, policy_gain);
                b[gains_block::KP] = pd[0];
                b[gains_block::KD] = pd[1];
                b
            })
            .collect()
    }

    /// 解析一帧 SYNC_WRITE，返回 (地址, 每颗的 (id, data))。
    fn parse_sync_write(frame: &[u8]) -> (u8, Vec<(u8, Vec<u8>)>) {
        let addr = frame[5];
        let width = frame[6] as usize;
        let mut entries = Vec::new();
        let mut i = 7;
        while i < frame.len() - 1 {
            let id = frame[i];
            let data = frame[i + 1..i + 1 + width].to_vec();
            entries.push((id, data));
            i += 1 + width;
        }
        (addr, entries)
    }

    /// 日志中所有 SYNC_WRITE 广播帧。
    fn sync_write_frames(log: &Arc<Mutex<Vec<Vec<u8>>>>) -> Vec<Vec<u8>> {
        log.lock()
            .unwrap()
            .iter()
            .filter(|f| f[4] == wire::inst::SYNC_WRITE)
            .cloned()
            .collect()
    }

    /// 用带写帧日志的假传输层构造后端。
    fn io_with_log(
        bursts: Vec<Vec<u8>>,
        cal: Calibration,
    ) -> (Ft1910Io, Arc<Mutex<Vec<Vec<u8>>>>) {
        let (transport, log) = ScriptedTransport::with_log(bursts);
        let io = Ft1910Io::with_transport(Box::new(transport), "synthetic".into(), cal).unwrap();
        (io, log)
    }

    #[test]
    fn ticks_for_零位方向与钳制() {
        let (io, _log) = io_with_log(vec![], Calibration::default());
        let rad = wire::RAD_PER_COUNT;
        // 默认标定 direction=+1、零位 2048。
        assert_eq!(io.ticks_for(0, 0.0), 2048);
        assert_eq!(io.ticks_for(0, 100.0 * rad), 2148);
        assert_eq!(io.ticks_for(0, -100.0 * rad), 1948);
        // 超出行程钳到 0..4095。
        assert_eq!(io.ticks_for(0, 100_000.0 * rad), 4095);
        assert_eq!(io.ticks_for(0, -100_000.0 * rad), 0);

        // direction=-1：正角对应减计数。
        let mut cal = Calibration::default();
        cal.joints[0].direction = -1.0;
        let (io2, _log) = io_with_log(vec![], cal);
        assert_eq!(io2.ticks_for(0, 100.0 * rad), 1948);
        assert_eq!(io2.ticks_for(0, -100.0 * rad), 2148);
    }

    #[test]
    fn write_未使能时不发目标帧() {
        let (mut io, log) = io_with_log(vec![], Calibration::default());
        io.write(&JointTargets::new([0.0; NUM_JOINTS])).unwrap();
        assert!(log.lock().unwrap().is_empty(), "未使能时一个字节都不该发");
    }

    #[test]
    fn set_gain_写入并读回校验() {
        let cal = Calibration::default();
        let bursts = vec![joint_blocks_burst(&gain_blocks(200, &cal))];
        let (mut io, log) = io_with_log(bursts, cal);
        io.set_gain(200).unwrap();
        assert_eq!(io.gain, 200);
        assert!(io.gains_verified);
        // 一帧 SYNC_WRITE 到 KP(50)，15 个关节。
        let sw = sync_write_frames(&log);
        assert_eq!(sw.len(), 1);
        let (addr, entries) = parse_sync_write(&sw[0]);
        assert_eq!(addr, reg::KP);
        assert_eq!(entries.len(), NUM_JOINTS);
        // 普通关节 P=6，嘴（更轻）P=10，D 恒为 20。
        assert_eq!(entries[0].1, vec![6, 20]);
        assert_eq!(entries[MOUTH_INDEX].1, vec![10, 20]);
        // 幂等：同增益再来一次不发帧。
        io.set_gain(200).unwrap();
        assert_eq!(sync_write_frames(&log).len(), 1);
    }

    #[test]
    fn set_gain_读回不符则报错并回滚() {
        let cal = Calibration::default();
        let bad: Vec<Vec<u8>> = (0..NUM_JOINTS)
            .map(|_| {
                let mut b = vec![0u8; gains_block::LEN];
                b[gains_block::KP] = 99;
                b[gains_block::KD] = 20;
                b
            })
            .collect();
        let (mut io, _log) = io_with_log(vec![joint_blocks_burst(&bad)], cal);
        assert!(io.set_gain(200).is_err());
        // 失败回滚到初始增益 0、且未确认。
        assert_eq!(io.gain, 0);
        assert!(!io.gains_verified);
    }

    #[test]
    fn set_torque_off_逐颗关并读回() {
        let cal = Calibration::default();
        let mut bursts = torque_ack_bursts();
        let zeros: Vec<Vec<u8>> = (0..NUM_JOINTS).map(|_| vec![0u8]).collect();
        bursts.push(joint_blocks_burst(&zeros));
        let (mut io, log) = io_with_log(bursts, cal);
        io.set_torque(false).unwrap();
        assert!(!io.enabled);
        // 15 帧逐颗 WRITE 到 40、data=[0]，绝不能是 SYNC_WRITE。
        let (torque_count, order_ok) = {
            let frames = log.lock().unwrap();
            let writes: Vec<&Vec<u8>> = frames
                .iter()
                .filter(|f| f[4] == wire::inst::WRITE && f[5] == reg::TORQUE_ENABLE)
                .collect();
            let order_ok = writes
                .iter()
                .enumerate()
                .all(|(joint, f)| f[2] == JOINT_IDS[joint] && &f[6..f.len() - 1] == &[0u8]);
            (writes.len(), order_ok)
        };
        assert_eq!(torque_count, NUM_JOINTS);
        assert!(order_ok, "逐颗 id 顺序正确且 data=[0]");
        assert_eq!(sync_write_frames(&log).len(), 0, "关扭矩不能用 SYNC_WRITE");
    }

    #[test]
    fn set_torque_on_贴当前位再逐颗上扭矩() {
        let cal = Calibration::default();
        let mut bursts = Vec::new();
        // 1. read()：位置全 2048（零位）。
        bursts.push(full_read_burst(&servo_block(2048, 0, 0)));
        // 2. ensure_gains（初始 gain=0 → P 钳为 1）读回。
        bursts.push(joint_blocks_burst(&gain_blocks(0, &cal)));
        // 3. verify_goals：目标应被贴成当前位 2048。
        let goals: Vec<Vec<u8>> =
            (0..NUM_JOINTS).map(|_| 2048u16.to_le_bytes().to_vec()).collect();
        bursts.push(joint_blocks_burst(&goals));
        // 4. 逐颗上扭矩的 15 个空 ack。
        bursts.extend(torque_ack_bursts());
        // 5. torque_enabled 读回全 1。
        let ones: Vec<Vec<u8>> = (0..NUM_JOINTS).map(|_| vec![1u8]).collect();
        bursts.push(joint_blocks_burst(&ones));
        let (mut io, log) = io_with_log(bursts, cal);
        io.set_torque(true).unwrap();
        assert!(io.enabled);

        // 广播帧恰好两帧：增益(50)、目标(42)，顺序固定。
        let sw = sync_write_frames(&log);
        assert_eq!(sw.len(), 2);
        assert_eq!(sw[0][5], reg::KP);
        let (goal_addr, goal_entries) = parse_sync_write(&sw[1]);
        assert_eq!(goal_addr, reg::GOAL_POSITION_L);
        assert_eq!(goal_entries.len(), NUM_JOINTS);
        for (_id, data) in &goal_entries {
            assert_eq!(data, &2048u16.to_le_bytes().to_vec(), "目标必须贴当前位");
        }
        // 15 帧逐颗 WRITE 到 40、data=[1]。
        let torque_on = log
            .lock()
            .unwrap()
            .iter()
            .filter(|f| f[4] == wire::inst::WRITE && f[5] == reg::TORQUE_ENABLE)
            .count();
        assert_eq!(torque_on, NUM_JOINTS);
    }

    #[test]
    fn set_torque_on_部分使能失败则保持松弛() {
        let cal = Calibration::default();
        let mut bursts = Vec::new();
        bursts.push(full_read_burst(&servo_block(2048, 0, 0)));
        bursts.push(joint_blocks_burst(&gain_blocks(0, &cal)));
        let goals: Vec<Vec<u8>> =
            (0..NUM_JOINTS).map(|_| 2048u16.to_le_bytes().to_vec()).collect();
        bursts.push(joint_blocks_burst(&goals));
        bursts.extend(torque_ack_bursts());
        // 第一颗读回 0（未使能），其余 1。
        let mut tblocks: Vec<Vec<u8>> = (0..NUM_JOINTS).map(|_| vec![1u8]).collect();
        tblocks[0] = vec![0u8];
        bursts.push(joint_blocks_burst(&tblocks));
        let (mut io, _log) = io_with_log(bursts, cal);
        assert!(io.set_torque(true).is_err());
        assert!(!io.enabled, "部分使能失败时必须保持松弛");
    }

    #[test]
    fn reboot_先关扭矩再重启并复位标志() {
        let cal = Calibration::default();
        let (mut io, log) = io_with_log(vec![], cal);
        io.enabled = true;
        io.gains_verified = true;
        let id = JOINT_IDS[0];
        io.reboot(id).unwrap();
        {
            let frames = log.lock().unwrap();
            // 恰好两帧：先 WRITE 40=[0]，再 REBOOT 0x08，同一 id。
            assert_eq!(frames.len(), 2);
            assert_eq!(frames[0][4], wire::inst::WRITE);
            assert_eq!(frames[0][2], id);
            assert_eq!(frames[0][5], reg::TORQUE_ENABLE);
            assert_eq!(&frames[0][6..frames[0].len() - 1], &[0u8]);
            assert_eq!(frames[1][4], wire::inst::REBOOT);
            assert_eq!(frames[1][2], id);
        }
        assert!(!io.enabled);
        assert!(!io.gains_verified);
        // IMU 节点（200）不是关节，不能重启，且在写帧之前就拒绝（不追加帧）。
        assert!(io.reboot(200).is_err());
        assert_eq!(log.lock().unwrap().len(), 2);
    }

    #[test]
    fn 使能后write_一帧同步目标且换算正确() {
        let cal = Calibration::default();
        let (mut io, log) = io_with_log(vec![], cal);
        io.enabled = true;
        let mut targets = JointTargets::new([0.0; NUM_JOINTS]);
        targets.positions[0] = 100.0 * wire::RAD_PER_COUNT; // +100 计数 → 2148
        io.write(&targets).unwrap();
        let sw = sync_write_frames(&log);
        assert_eq!(sw.len(), 1);
        let (addr, entries) = parse_sync_write(&sw[0]);
        assert_eq!(addr, reg::GOAL_POSITION_L);
        assert_eq!(entries[0].1, 2148u16.to_le_bytes().to_vec());
        assert_eq!(entries[1].1, 2048u16.to_le_bytes().to_vec());
    }

    // ── #19 IMU ft6 人格：就绪 / 形状 / 陈旧 / 身份窗 ─────────────────────────

    /// 构造 ft6 后端，准备一个「1 IMU + 15 舵机」读 burst（位置全零位），不预先 read。
    fn ft6_io_one(status: u8, quat: [u16; 3], reserved: u8, count: u8) -> Ft1910Io {
        let servo = servo_block(2048, 0, 0);
        let mut imu = imu_block([0, 0, 0], quat, count, status);
        imu[imu_block::RESERVED] = reserved;
        backend_with(vec![imu], &servo, Calibration::default())
    }

    // ── StaleTracker 计数口径（直接单测，对齐官方 StaleImuTracker）────────────

    /// 造一个只改 quat x（byte6..8）的前 12 字节窗口。
    fn window(quat_x: u16) -> [u8; IMU_BLOCK_LEN] {
        let mut w = [0u8; IMU_BLOCK_LEN];
        w[6..8].copy_from_slice(&quat_x.to_le_bytes());
        w
    }

    #[test]
    fn stale_第一块不计数_后续新鲜保持零() {
        let mut t = StaleTracker::new();
        assert_eq!(t.observe(&window(0x3000), 1), (0, 0));
        assert_eq!(t.snapshot(), ImuStale { total: 0, run: 0 });
        assert_eq!(t.observe(&window(0x3001), 2), (0, 0));
        assert_eq!(t.snapshot(), ImuStale { total: 0, run: 0 });
    }

    #[test]
    fn stale_窗口重复累计total_新鲜块只把run归零() {
        let mut t = StaleTracker::new();
        t.observe(&window(0x3000), 1);
        // 第二块窗口相同、序号不同：窗口重复 1 次，序号仍为 0。
        assert_eq!(t.observe(&window(0x3000), 2), (0, 1));
        assert_eq!(t.snapshot(), ImuStale { total: 1, run: 1 });
        // 再来一块新窗口：run 归零，total 保留累计（不复位）。
        assert_eq!(t.observe(&window(0x3005), 3), (0, 0));
        assert_eq!(t.snapshot(), ImuStale { total: 1, run: 0 });
    }

    #[test]
    fn stale_ft6序号连续三次停滞_窗口同步重复() {
        let mut t = StaleTracker::new();
        let w = window(0x3000);
        assert_eq!(t.observe(&w, 5), (0, 0));
        assert_eq!(t.observe(&w, 5).0, 1);
        // 第 3 个相同序号：seq_run=2（= FT6_SEQ_STALE_RUN），板端重发时窗口也同步重复。
        let (seq, win) = t.observe(&w, 5);
        assert_eq!(seq, FT6_SEQ_STALE_RUN);
        assert_eq!(win, FT6_SEQ_STALE_RUN);
    }

    #[test]
    fn stale_窗口连续重复到冻结阈值() {
        let mut t = StaleTracker::new();
        let w = window(0x3000);
        t.observe(&w, 1); // 基线，不计数
        // 序号逐帧递增（只测窗口、不触发 seq）；第 26 个相同块即 FROZEN_RUN 次重复到达阈值。
        for k in 1u64..=FROZEN_RUN {
            let (seq, win) = t.observe(&w, (k as u8) + 1);
            assert_eq!(seq, 0, "序号递增，seq_run 恒为 0");
            assert_eq!(win, k);
        }
        assert_eq!(
            t.snapshot(),
            ImuStale {
                total: FROZEN_RUN,
                run: FROZEN_RUN
            }
        );
    }

    #[test]
    fn ft6_就绪只看状态位低三位() {
        let mut io = ft6_io_one(0x00, [0x3000, 0, 0], 0, 1);
        io.read().ok();
        assert!(io.imu_ready(), "状态位全 0 且四元数有效应就绪");

        // bit3 READER_SLOW 是「主机读太慢」信息位，不参与就绪。
        let mut io = ft6_io_one(ft6_status::READER_SLOW, [0x3000, 0, 0], 0, 1);
        io.read().ok();
        assert!(io.imu_ready(), "READER_SLOW 不应判未就绪");

        // bit0/1/2 任一置位都不就绪。
        for status in [
            ft6_status::NOT_READY,
            ft6_status::STALE,
            ft6_status::SENSOR_ERR,
            0x07,
        ] {
            let mut io = ft6_io_one(status, [0x3000, 0, 0], 0, 1);
            io.read().ok();
            assert!(!io.imu_ready(), "status=0x{status:02x} 不应就绪");
        }

        // 状态位全 0 但四元数全 0：板端未出表，仍不就绪。
        let mut io = ft6_io_one(0x00, [0, 0, 0], 0, 1);
        io.read().ok();
        assert!(!io.imu_ready(), "四元数全 0 不应就绪");
    }

    #[test]
    fn ft6_形状错误时read报错() {
        let mut io = ft6_io_one(0x00, [0x3000, 0, 0], 0xAA, 1);
        assert!(io.read().is_err(), "reserved 非 0 必须拒绝整块");
        let mut io = ft6_io_one(0x10, [0x3000, 0, 0], 0, 1);
        assert!(io.read().is_err(), "状态高 4 位非 0 必须拒绝整块");
    }

    #[test]
    fn ft6_逐帧新姿态无陈旧_重发同帧才累计() {
        let servo = servo_block(2048, 0, 0);
        // 序号递增、quat 低位微抖（真芯片静置低位也跳）：窗口每帧不同，零陈旧。
        let fresh: Vec<Vec<u8>> = (1u8..=3)
            .map(|c| imu_block([0, 0, 0], [0x3000 + u16::from(c - 1), 0, 0], c, 0x00))
            .collect();
        let mut io = backend_with(fresh, &servo, Calibration::default());
        for _ in 0..3 {
            io.read().ok();
        }
        assert_eq!(io.imu_stale(), ImuStale { total: 0, run: 0 });

        // 板端重发同一帧（3 个块前 12 字节与序号完全相同）：第 1 块是基线，后 2 块各算一次
        // 窗口重复，total/run 都是 2（序号停滞的 warn 阈值由下方 tracker 单测锁定）。
        let stuck: Vec<Vec<u8>> = (0..3)
            .map(|_| imu_block([0, 0, 0], [0x3000, 0, 0], 5, 0x00))
            .collect();
        let mut io = backend_with(stuck, &servo, Calibration::default());
        for _ in 0..3 {
            io.read().ok();
        }
        assert_eq!(io.imu_stale(), ImuStale { total: 2, run: 2 });
    }

    #[test]
    fn native_就绪靠有效四元数计数而非状态位() {
        let mut cal = Calibration::default();
        cal.imu_personality = ImuPersonality::Native;
        let servo = servo_block(2048, 0, 0);
        // 即使状态位亮着 SFLP_VALID+FUSION_OK，首帧也不就绪（解码器计数不足）。
        let mut io = backend_with(
            vec![imu_block([0, 0, 0], [0x3000, 0, 0], 1, 0x81)],
            &servo,
            cal,
        );
        io.read().ok();
        assert!(!io.imu_ready(), "native 首帧不应就绪");

        // 累计 25 个有效四元数（序号递增，避免被序号判陈旧）后就绪。
        let many: Vec<Vec<u8>> = (1u8..=25)
            .map(|c| imu_block([0, 0, 0], [0x3000, 0, 0], c, 0x81))
            .collect();
        let mut io = backend_with(many, &servo, cal);
        for _ in 0..25 {
            io.read().ok();
        }
        assert!(io.imu_ready(), "native 累计 25 个有效四元数后应就绪");
    }

    #[test]
    fn native_传感器错误常亮时不就绪() {
        let mut cal = Calibration::default();
        cal.imu_personality = ImuPersonality::Native;
        let servo = servo_block(2048, 0, 0);
        // 25 帧有效四元数（低位微抖），但 SENSOR_ERR(0x08) 常亮：即使解码器累计满也不就绪。
        let many: Vec<Vec<u8>> = (1u8..=25)
            .map(|c| {
                imu_block(
                    [0, 0, 0],
                    [0x3000 + u16::from(c - 1), 0, 0],
                    c,
                    0x81 | native_status::SENSOR_ERR,
                )
            })
            .collect();
        let mut io = backend_with(many, &servo, cal);
        for _ in 0..25 {
            io.read().ok();
        }
        assert!(!io.imu_ready(), "native SENSOR_ERR 应和 ft6 一样 gate 就绪");
    }

    #[test]
    fn detect_personality_读身份窗识别() {
        fn io_with_identity(data: &[u8]) -> Ft1910Io {
            let burst = ack_frame(200, data);
            Ft1910Io::with_transport(
                Box::new(ScriptedTransport::new(vec![burst])),
                "synthetic".to_owned(),
                Calibration::default(),
            )
            .expect("标定合法")
        }
        assert_eq!(
            io_with_identity(&imu_identity::FT6).detect_personality().unwrap(),
            ImuPersonality::Ft6
        );
        assert_eq!(
            io_with_identity(&imu_identity::NATIVE)
                .detect_personality()
                .unwrap(),
            ImuPersonality::Native
        );
        assert!(
            io_with_identity(&[0xAA; 5]).detect_personality().is_err(),
            "身份窗无法识别应报错且不改变人格"
        );
    }

    /// 仓库随附的标定模板 config/ft1910.toml 必须能解析、通过校验，且与本机实测一致。
    /// 用 include_str! 把模板编译进测试：模板漏关节、方向写错或越界，这里立刻挂。
    #[test]
    fn 标定模板_ft1910_toml_解析且匹配实测() {
        const TEXT: &str = include_str!("../config/ft1910.toml");
        let cal: Calibration = toml::from_str(TEXT).expect("标定模板必须能解析");
        cal.validate().expect("标定模板必须通过校验");

        assert_eq!(cal.imu_id, 200);
        assert_eq!(cal.baud_rate, 1_000_000);
        assert_eq!(cal.imu_personality, ImuPersonality::Ft6);
        assert_eq!(cal.imu_mount, [1.0, 0.0, 0.0, 0.0]);
        assert_eq!(cal.pd, PdProfile { kp: 6, kd: 20, mouth_kp: 10 });

        // 15 个关节零位/限位一致；方向仅 neck_pitch(id30)、head_pitch(id31) 为 +1，其余 −1。
        assert_eq!(cal.joints.len(), 15);
        for (i, joint) in cal.joints.iter().enumerate() {
            assert_eq!(joint.zero_ticks, 2048, "joint {i} zero_ticks");
            assert_eq!(joint.limits, (0, 4095), "joint {i} limits");
            let expect_dir = if matches!(JOINT_NAMES[i], "neck_pitch" | "head_pitch") {
                1.0
            } else {
                -1.0
            };
            assert_eq!(joint.direction, expect_dir, "joint {} direction", JOINT_NAMES[i]);
        }
    }

    /// 序列化再反序列化应得到同一张标定（TOML round-trip）。
    #[test]
    fn 标定_toml_往返一致() {
        let cal = Calibration::default();
        let text = toml::to_string(&cal).expect("序列化");
        let back: Calibration = toml::from_str(&text).expect("反序列化");
        assert_eq!(cal, back);
    }
}
