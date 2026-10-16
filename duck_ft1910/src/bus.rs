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
//! 假传输层，所以 `read()` 的整条解析链在开发机上就能验证，不必上板。
//!
//! 当前阶段（#17）只实现了读路径；写、增益、扭矩、重启、慢变量留待后续阶段，
//! 现阶段调用会直接返回一个「尚未实现」的总线错误而不是静默写错。

use std::array;
use std::time::{Duration, Instant};

use duck_control::imu::{SflpDecoder, IMU_BLOCK_LEN};
use duck_control::io::{ImuStale, IoError, JointTargets, Result, RobotIo, Sensors, SlowSensors};
use duck_control::model::{JOINT_IDS, JOINT_NAMES, NUM_JOINTS};

use crate::wire::{self, block, imu_block, reg, ExpectedReply, FrameReader};

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
#[derive(Debug, Clone, Copy)]
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

/// 整机标定：IMU 地址、15 个关节的零位/方向、IMU 安装朝向、波特率。
#[derive(Debug, Clone, Copy)]
pub struct Calibration {
    /// IMU 在总线上的舵机 id（伪装成舵机）。
    pub imu_id: u8,
    pub joints: [JointCalibration; NUM_JOINTS],
    /// IMU 安装四元数 `[w, x, y, z]`，把板载坐标系转到躯干坐标系。
    ///
    /// ft6 固件（v1.6.0）已在**板端**做了安装预旋转（X、Y 反向，输出即躯干坐标），
    /// 主机端不再施加旋转，所以用单位四元数。该结论与 servo-web 前端直读板端四元数、
    /// 3D 姿态正确一致；#19 上板再做最终复核。
    pub imu_mount: [f64; 4],
    pub baud_rate: u32,
}

impl Default for Calibration {
    fn default() -> Self {
        Self {
            imu_id: 200,
            joints: array::from_fn(|_| JointCalibration::default()),
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

/// 跟踪 IMU 帧序号：序号原地踏步说明姿态没有刷新（板端冻结或读取太慢）。
///
/// 这里先用相邻两帧序号相等即计一次陈旧（社区版做法）；ft6 要求连续 3 次相同才判 stale
/// 的更细判据在 #19 落地。
#[derive(Debug, Default)]
struct StaleTracker {
    count: Option<u8>,
    total: u64,
    run: u64,
}

impl StaleTracker {
    fn new() -> Self {
        Self::default()
    }

    fn observe(&mut self, _raw: &[u8; IMU_BLOCK_LEN], count: Option<u8>) {
        self.total += 1;
        if let Some(count) = count {
            if self.count == Some(count) {
                self.run += 1;
            } else {
                self.count = Some(count);
                self.run = 0;
            }
        }
    }

    fn snapshot(&self) -> ImuStale {
        ImuStale {
            total: self.total,
            run: self.run,
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
    imu_status: u8,
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
            imu_status: 0,
            io_timeout: BURST_LIMIT,
        })
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
}

impl RobotIo for Ft1910Io {
    fn read(&mut self) -> Result<Sensors> {
        // IMU 在前、15 颗舵机随后，地址 56、读 15 字节。
        let frame = wire::sync_read(&self.ids, reg::PRESENT_POSITION_L, block::LEN as u8);
        let expected = ExpectedReply::new(&self.ids, block::LEN);
        let blocks = self.request(&frame, &expected)?;

        let mut sensors = Sensors::default();

        // 第一个块是 IMU：前 12 字节直接交给官方解码器（陀螺 + 半精度四元数）。
        let imu = &blocks[0];
        if imu.len() < IMU_BLOCK_LEN {
            return Err(IoError::ShortRead {
                what: "IMU 控制块",
                expected: IMU_BLOCK_LEN,
                got: imu.len(),
            });
        }
        let mut raw = [0u8; IMU_BLOCK_LEN];
        raw.copy_from_slice(&imu[..IMU_BLOCK_LEN]);
        let status = imu[imu_block::STATUS];
        let count = imu[imu_block::COUNT];
        self.stale.observe(&raw, Some(count));
        sensors.imu = self.decoder.decode(&raw);
        self.imu_status = status;

        // 其余 15 块按关节表顺序填充。
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
        }

        Ok(sensors)
    }

    // ── 以下方法在写路径（#18）/时序（#20）落地，现阶段一律明确报「未实现」，不静默误动 ──

    fn write(&mut self, _targets: &JointTargets) -> Result<()> {
        Err(IoError::Bus("Ft1910Io::write 尚未实现（写路径 #18）".into()))
    }

    fn set_gain(&mut self, _kp: u16) -> Result<()> {
        Err(IoError::Bus("Ft1910Io::set_gain 尚未实现（写路径 #18）".into()))
    }

    fn set_torque(&mut self, _on: bool) -> Result<()> {
        Err(IoError::Bus("Ft1910Io::set_torque 尚未实现（写路径 #18）".into()))
    }

    fn reboot(&mut self, _id: u8) -> Result<()> {
        Err(IoError::Bus("Ft1910Io::reboot 尚未实现（写路径 #18）".into()))
    }

    fn slow_sensors(&mut self) -> Result<SlowSensors> {
        Err(IoError::Bus("Ft1910Io::slow_sensors 尚未实现（时序 #20）".into()))
    }

    // IMU 已收到足够多的有效四元数才算 ready；ft6 的状态位判据（byte13 & 0x07 == 0）在 #19 补。
    fn imu_ready(&self) -> bool {
        self.decoder.ready()
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

    /// 回放预设字节的假传输层。关键是按「一次请求 → 一个 burst」的边界供数：
    /// 每次 `write_all`（一帧请求）激活一个 burst，`read` 只在该 burst 内逐块返回字节，
    /// 该 burst 放完就超时——和真实半双工总线「设备只应答一次、随后空闲」一致。否则一
    /// 次 request 会把后面几个 burst 一并读光，下一次请求就拿不到数据。
    struct ScriptedTransport {
        bursts: Vec<Vec<u8>>,
        next_burst: usize,
        active: Option<(usize, usize)>,
        #[allow(dead_code)]
        written: Vec<u8>,
    }
    impl ScriptedTransport {
        fn new(bursts: Vec<Vec<u8>>) -> Self {
            Self {
                bursts,
                next_burst: 0,
                active: None,
                written: Vec::new(),
            }
        }
    }
    impl Transport for ScriptedTransport {
        fn write_all(&mut self, buf: &[u8]) -> std::io::Result<()> {
            self.written.extend_from_slice(buf);
            // 每发一帧请求，激活它对应的下一个 burst。
            if self.next_burst < self.bursts.len() {
                self.active = Some((self.next_burst, 0));
                self.next_burst += 1;
            } else {
                self.active = None;
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
        // 陀螺全 0；四元数 x=0.125（半精度 0x3000），y=z=0（绕 x 前倾）。
        // 连续 3 个相同姿态块、序号递增，让官方解码器的重力 3 点中值 settle 到该姿态。
        let imu_blocks: Vec<Vec<u8>> = (1u8..=3)
            .map(|count| imu_block([0, 0, 0], [0x3000, 0, 0], count, 0x00))
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

        // 读到 3 帧，序号各不相同：总数 3、连续陈旧为 0。
        let stale = io.imu_stale();
        assert_eq!(stale.total, 3);
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
}
