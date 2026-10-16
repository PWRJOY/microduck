//! 飞特 SCS/HLS 半双工协议的帧编解码、寄存器与单位换算——只管字节，不管别的。
//!
//! 这个模块故意不碰串口、不碰时钟、不碰 `RobotIo`，所以每一个字节都能在开发机上离线验证。
//! 参考来源有三处，且彼此独立，逐字节对齐才算对：
//! - 飞特官方库 `FTServo_Linux-main/src/SCS.cpp`（writeBuf / Ack / Read / syncReadPacketTx / syncWrite）
//! - 我们已在真机上跑通的 `servo-web/feetech.py`
//! - 飞特协议手册与磁编码 STS 内存表
//!
//! ```text
//! 指令帧: FF FF ID LEN INST [ADDR] [DATA...] CHK
//!         LEN = 1(INST) + ADDR? + DATA + 1(CHK)
//! 应答帧: FF FF ID LEN STATUS [DATA...] CHK
//!         LEN = 1(STATUS) + DATA + 1(CHK)
//! 校验和: CHK = ~(ID + LEN + INST + ADDR + ΣDATA)   （没有 ADDR 时 ADDR 按 0 计入）
//! ```
//! 所有加法都是字节回绕（u8 wrapping）。

// ── 校验和 ───────────────────────────────────────────────────────────────────

/// 参与校验和的累加：从 ID 开始、到校验和之前的每个字节。
/// 帧里带 ADDR 时它计入，不带 ADDR 的指令（PING/REBOOT/CAL）按 0 计入，与官方库一致。
fn checksum_sum(id: u8, len: u8, inst: u8, addr: u8, data: &[u8]) -> u8 {
    let mut sum = id.wrapping_add(len).wrapping_add(inst).wrapping_add(addr);
    for b in data {
        sum = sum.wrapping_add(*b);
    }
    sum
}

/// 实际发送的校验字节：[`checksum_sum`] 的按位取反。
pub fn checksum(id: u8, len: u8, inst: u8, addr: u8, data: &[u8]) -> u8 {
    !checksum_sum(id, len, inst, addr, data)
}

// ── 指令码（INST.h），外加手册表里漏写、但 HD-1910 固件实测支持的 REBOOT ───────

pub mod inst {
    pub const PING: u8 = 0x01;
    pub const READ: u8 = 0x02;
    pub const WRITE: u8 = 0x03;
    pub const REG_WRITE: u8 = 0x04;
    pub const ACTION: u8 = 0x05;
    pub const RECOVERY: u8 = 0x06;
    /// 重启单颗舵机。HD-1910-C001 固件 3.46 实测：不回包，约 800ms 后回来，EEPROM 保留、
    /// RAM 里的 P/D 回到 EEPROM 值——所以重启后需要重新下发 RAM 增益。
    pub const REBOOT: u8 = 0x08;
    pub const RESET: u8 = 0x0A;
    /// 位置校准：把舵机当前位置的读数改成给定值（默认中位 2048），偏移由舵机自己算、自己存。
    pub const CAL: u8 = 0x0B;
    pub const SYNC_READ: u8 = 0x82;
    pub const SYNC_WRITE: u8 = 0x83;
}

// ── HLS 内存表寄存器地址（与 HD-1910 实测核对） ─────────────────────────────

pub mod reg {
    pub const EEPROM_KP: u8 = 21;
    pub const EEPROM_KD: u8 = 22;
    pub const OFS_L: u8 = 31;
    pub const MODE: u8 = 33;
    pub const TORQUE_ENABLE: u8 = 40;
    pub const ACC: u8 = 41;
    pub const GOAL_POSITION_L: u8 = 42;
    pub const GOAL_SPEED_L: u8 = 46;
    pub const TORQUE_LIMIT_L: u8 = 48;
    /// RAM 易失位置 P/D，手册未列、固件支持，运行中改增益只动这两个；重启会从 EEPROM 21/22 重载。
    pub const KP: u8 = 50;
    pub const KD: u8 = 51;
    pub const LOCK: u8 = 55;
    pub const PRESENT_POSITION_L: u8 = 56;
    pub const PRESENT_SPEED_L: u8 = 58;
    pub const PRESENT_LOAD_L: u8 = 60;
    pub const PRESENT_VOLTAGE: u8 = 62;
    pub const PRESENT_TEMPERATURE: u8 = 63;
    /// 舵机自身的报警/状态字节，非零即故障，是锁存过载、过温唯一可见的地方。
    pub const PRESENT_STATUS: u8 = 65;
    pub const MOVING: u8 = 66;
    pub const PRESENT_CURRENT_L: u8 = 69;
}

/// 56 起的 15 字节状态块内的字节偏移（都等于 寄存器 − 56）。
///
/// 写死而不是推导，是为了让多字节字段的宽度一目了然——偏移错一位会从错误的寄存器读出一个
/// 看起来还挺合理的数。
pub mod block {
    pub const POSITION: usize = 0;
    pub const SPEED: usize = 2;
    pub const LOAD: usize = 4;
    pub const VOLTAGE: usize = 6;
    pub const TEMPERATURE: usize = 7;
    pub const STATUS: usize = 9;
    pub const MOVING: usize = 10;
    pub const GOAL: usize = 11;
    pub const CURRENT: usize = 13;
    /// 块字节数。56..70 共 15 字节，200 号 IMU 节点也按同样 15 字节应答，一次 sync_read 通吃。
    pub const LEN: usize = 15;
}

/// 50 起的 14 字节增益/慢变量块（slow_sensors 用）：
/// `Kp Kd Ki · · lock 位置(2) 速度(2) 负载(2) 电压 温度`。
///
/// 一次连续读就把增益和电池电压、机壳温度都拿回来，15 颗舵机大约 1ms，每秒一次。
pub mod gains_block {
    pub const KP: usize = 0;
    pub const KD: usize = 1;
    pub const LOCK: usize = 5;
    pub const VOLTAGE: usize = 12;
    pub const TEMPERATURE: usize = 13;
    pub const LEN: usize = 14;
}

/// 200 号 IMU 节点在同一地址（56）应答的 15 字节块内的偏移。
///
/// 前 12 字节是飞特人格下的陀螺 xyz + SFLP 四元数 xyz，再后面是帧计数和状态字节。
/// 具体解码在后续的 `imu` 模块，这里先把协议布局钉死。
pub mod imu_block {
    /// 前 12 字节：陀螺仪 i16×3（0..6）+ 半精度四元数 x/y/z（6..12），与官方 v2 板同格式。
    pub const CTRL_LEN: usize = 12;
    /// 帧计数，回绕；ft6 语义是「四元数序号」，native 是「采样计数」。不变代表这一帧没刷新。
    pub const COUNT: usize = 12;
    /// 状态字节，位含义随固件人格而不同（见 [`ft6_status`] / [`native_status`]）。
    pub const STATUS: usize = 13;
    /// 保留字节，规格要求恒为 0。
    pub const RESERVED: usize = 14;
    /// 飞特人格整块长度（地址 56 读 15 字节）。
    pub const LEN: usize = 15;
}

/// ID 200 寄存器 0..4 的身份窗，用来区分固件人格（固件 v1.6.0 起默认 ft6，v1.4.x 是 native）。
///
/// 判据照 servo-web `imu_bus.py` 的 `detect_personality`：ft6 应答 `06 00 00 00 F2`
/// （FW6.0 + 型号 0xF200），native 应答 `01 00 00 49 4D`。
pub mod imu_identity {
    /// 身份窗起始寄存器。
    pub const ADDR: u8 = 0;
    /// 身份窗长度（寄存器 0..4）。
    pub const LEN: u8 = 5;
    /// ft6 人格应答。
    pub const FT6: [u8; LEN as usize] = [0x06, 0x00, 0x00, 0x00, 0xF2];
    /// community-kit native 人格应答。
    pub const NATIVE: [u8; LEN as usize] = [0x01, 0x00, 0x00, 0x49, 0x4D];
}

/// ft6 人格 byte13 状态位（FEE_ST_*，协议 §16）。
///
/// 注意 bit0 极性与 native 相反：ft6 置 1 表示「未就绪」。
pub mod ft6_status {
    /// 融合未出数 / 四元数全 0（置 1 = 未就绪）。
    pub const NOT_READY: u8 = 0x01;
    /// 四元数年龄 > 30ms（发送时计算）。
    pub const STALE: u8 = 0x02;
    /// 传感器错误（真故障：LSM6DSV16X 通信/数据异常）。
    pub const SENSOR_ERR: u8 = 0x04;
    /// 距上次读取新增 ≥5 个四元数（信息位：主机读太慢，不致命，不参与就绪判定）。
    pub const READER_SLOW: u8 = 0x08;
    /// bit0/1/2 全为 0 才算就绪；bit3 不参与。
    pub const READY_MASK: u8 = 0x07;
    /// bits4-7 恒为 0，非 0 即块形状错误。
    pub const SHAPE_MASK: u8 = 0xF0;
}

/// community-kit native 人格 byte13 状态位（TELEM_FLAG_*，GD32 固件 board.h）。
///
/// 与 ft6 极性相反：0x01 / 0x80 是好事，0x08 才是真故障。
pub mod native_status {
    /// 本块带有效姿态（好事）。
    pub const SFLP_VALID: u8 = 0x01;
    /// u8 采样计数刚回绕（100Hz 下每 2.55s 一次，正常）。
    pub const CNT_WRAP: u8 = 0x02;
    /// 陀螺仪饱和（瞬时过载，不致命）。
    pub const GYRO_SAT: u8 = 0x04;
    /// 传感器错误（真故障）。
    pub const SENSOR_ERR: u8 = 0x08;
    /// 数据来自模拟器。
    pub const SIMULATED: u8 = 0x10;
    /// 固件认为数据不再变化。
    pub const FROZEN: u8 = 0x20;
    /// 配置未保存。
    pub const CFG_DIRTY: u8 = 0x40;
    /// 融合算法运行中（好事）。
    pub const FUSION_OK: u8 = 0x80;
}

pub const BROADCAST_ID: u8 = 0xFE;
/// 协议能产生的最大帧（LEN 是 u8）。
pub const MAX_FRAME: usize = 4 + 255;

// ── 错误 ─────────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error {
    /// 帧到了但帧头、长度或校验和对不上。
    BadFrame(&'static str),
    /// 收到一个没被点名的设备的应答。
    UnexpectedId(u8),
    /// 设备以非零状态字节应答。它与丢帧不同——这是舵机在报告过载/过温。
    DeviceStatus { id: u8, status: u8 },
    /// 应答比请求要求的短。
    ShortData { got: usize, want: usize },
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::BadFrame(why) => write!(f, "飞特帧错误: {why}"),
            Error::UnexpectedId(id) => write!(f, "收到了未点名的 id {id} 的应答"),
            Error::DeviceStatus { id, status } => write!(f, "舵机 {id} 上报状态 {status:#04x}"),
            Error::ShortData { got, want } => write!(f, "应答只有 {got} 字节，期望 {want}"),
        }
    }
}

impl std::error::Error for Error {}

// ── 帧构造 ───────────────────────────────────────────────────────────────────

/// 没有 ADDR 的指令（PING / REBOOT / CAL），可带 `data`（如 CAL 的目标值）：
/// 帧形如 `FF FF ID LEN INST [DATA...] CHK`，LEN = DATA + 2，ADDR 按 0 计入校验。
pub fn instruction_no_addr(id: u8, instruction: u8, data: &[u8]) -> Vec<u8> {
    let len = data.len() as u8 + 2;
    let chk = checksum(id, len, instruction, 0, data);
    let mut frame = Vec::with_capacity(5 + data.len());
    frame.extend_from_slice(&[0xFF, 0xFF, id, len, instruction]);
    frame.extend_from_slice(data);
    frame.push(chk);
    frame
}

/// 带 ADDR + DATA 的指令（READ / WRITE）：LEN = DATA + 3。
pub fn instruction(id: u8, instruction: u8, addr: u8, data: &[u8]) -> Vec<u8> {
    let len = data.len() as u8 + 3;
    let chk = checksum(id, len, instruction, addr, data);
    let mut frame = Vec::with_capacity(6 + data.len());
    frame.extend_from_slice(&[0xFF, 0xFF, id, len, instruction, addr]);
    frame.extend_from_slice(data);
    frame.push(chk);
    frame
}

pub fn ping(id: u8) -> Vec<u8> {
    instruction_no_addr(id, inst::PING, &[])
}

/// READ：唯一的 DATA 字节是要读的长度，所以 LEN = 4。
pub fn read(id: u8, addr: u8, len: u8) -> Vec<u8> {
    instruction(id, inst::READ, addr, &[len])
}

pub fn write(id: u8, addr: u8, data: &[u8]) -> Vec<u8> {
    instruction(id, inst::WRITE, addr, data)
}

pub fn reboot(id: u8) -> Vec<u8> {
    instruction_no_addr(id, inst::REBOOT, &[])
}

/// 位置校准 0x0B：不给目标值就把当前位置标成中位，给了就标成给定值。
pub fn calibrate(id: u8, value: Option<u16>) -> Vec<u8> {
    match value {
        None => instruction_no_addr(id, inst::CAL, &[]),
        Some(v) => instruction_no_addr(id, inst::CAL, &v.to_le_bytes()),
    }
}

/// SYNC_READ：`FF FF FE (N+4) 82 ADDR LEN ID... CHK`。
///
/// 每个被点名的 id 各回一个状态帧，且按它在列表里的位置占位——缺席的设备也会空出一个槽位，
/// 所以收集方必须按应答里的 id 归档，而不能假定到达顺序。200 号 IMU 放最前，让它先答。
pub fn sync_read(ids: &[u8], addr: u8, len: u8) -> Vec<u8> {
    let frame_len = ids.len() as u8 + 4;
    let mut sum = BROADCAST_ID
        .wrapping_add(frame_len)
        .wrapping_add(inst::SYNC_READ)
        .wrapping_add(addr)
        .wrapping_add(len);
    let mut frame = Vec::with_capacity(7 + ids.len());
    frame.extend_from_slice(&[0xFF, 0xFF, BROADCAST_ID, frame_len, inst::SYNC_READ, addr, len]);
    for id in ids {
        frame.push(*id);
        sum = sum.wrapping_add(*id);
    }
    frame.push(!sum);
    frame
}

/// SYNC_WRITE：`FF FF FE ((W+1)*N+4) 83 ADDR W ID DATA... CHK`。
///
/// 一帧下发全部目标，这是「一次总线往返」和「十五次往返」的区别。长度不合法直接报错，
/// 绝不截断——截断会让一部分舵机动、一部分不动，看起来就像标定错误。
pub fn sync_write(ids: &[u8], addr: u8, payloads: &[&[u8]]) -> Result<Vec<u8>, Error> {
    if ids.is_empty() || ids.len() != payloads.len() {
        return Err(Error::BadFrame("sync_write 的 id 数与数据数不一致"));
    }
    let width = payloads[0].len();
    if payloads.iter().any(|p| p.len() != width) {
        return Err(Error::BadFrame("sync_write 各数据宽度不一致"));
    }
    let width_u8 = u8::try_from(width).map_err(|_| Error::BadFrame("sync_write 数据过宽"))?;
    let frame_len = ((width + 1) * ids.len() + 4) as u8;
    let mut sum = BROADCAST_ID
        .wrapping_add(frame_len)
        .wrapping_add(inst::SYNC_WRITE)
        .wrapping_add(addr)
        .wrapping_add(width_u8);
    let mut frame = vec![0xFF, 0xFF, BROADCAST_ID, frame_len, inst::SYNC_WRITE, addr, width_u8];
    for (id, payload) in ids.iter().zip(payloads) {
        frame.push(*id);
        sum = sum.wrapping_add(*id);
        for b in *payload {
            frame.push(*b);
            sum = sum.wrapping_add(*b);
        }
    }
    frame.push(!sum);
    Ok(frame)
}

// ── 帧解析 ───────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Ack {
    pub id: u8,
    pub status: u8,
    pub data: Vec<u8>,
}

/// 解析一个完整的状态帧。`frame` 必须恰好是一帧：这里不在字节流里找帧头，
/// 需要在噪声流里重同步的调用方用 [`FrameReader`]。
pub fn parse_ack(frame: &[u8]) -> Result<Ack, Error> {
    if frame.len() < 6 {
        return Err(Error::BadFrame("比最短的 ping 应答还短"));
    }
    if frame[0] != 0xFF || frame[1] != 0xFF {
        return Err(Error::BadFrame("缺少帧头"));
    }
    let len = frame[3] as usize;
    if len < 2 {
        return Err(Error::BadFrame("LEN < 2"));
    }
    if frame.len() != len + 4 {
        return Err(Error::BadFrame("LEN 与帧长度不符"));
    }
    let body = &frame[2..frame.len() - 1];
    let expect = !body.iter().fold(0u8, |acc, b| acc.wrapping_add(*b));
    if expect != frame[frame.len() - 1] {
        return Err(Error::BadFrame("校验和不符"));
    }
    Ok(Ack {
        id: frame[2],
        status: frame[4],
        data: frame[5..frame.len() - 1].to_vec(),
    })
}

/// 若 `buf` 开头是一个完整帧，返回它的总长度；否则返回 None。
pub fn frame_len(buf: &[u8]) -> Option<usize> {
    if buf.len() < 4 {
        return None;
    }
    if buf[0] != 0xFF || buf[1] != 0xFF || buf[3] < 2 {
        return None;
    }
    let total = buf[3] as usize + 4;
    (buf.len() >= total).then_some(total)
}

/// 读取方愿意接受哪个 id 的应答、该应答多长。
///
/// 只信长度字节分不清「应答」和「我们自己指令的回声」：半双工总线上发给舵机的写帧同样有
/// `FF FF` 和看起来合理的长度，只找帧头会傻等一帧几百字节、永远不来的包，把真正的应答吞掉。
/// 同时知道 id 和确切应答长度，才能立刻重同步。
pub trait Expects {
    /// 该 id 应答的总字节数；没点过这个 id 则返回 None。
    fn reply_len(&self, id: u8) -> Option<usize>;
}

/// 一次请求点名的 id 列表，以及它们共同的应答长度。
#[derive(Debug, Clone)]
pub struct ExpectedReply {
    ids: Vec<u8>,
    data_len: usize,
    /// 总帧长 = LEN + 4，其中 LEN = data + 2。
    total: usize,
}

impl ExpectedReply {
    pub fn new(ids: &[u8], data_len: usize) -> Self {
        Self {
            ids: ids.to_vec(),
            data_len,
            total: data_len + 6,
        }
    }

    pub fn is_empty(&self) -> bool {
        self.ids.is_empty()
    }

    pub fn ids(&self) -> &[u8] {
        &self.ids
    }

    /// 每帧应带的数据字节数，用于把「答非所问但帧合法」的包也识别出来。
    pub fn reply_data_len(&self) -> usize {
        self.data_len
    }
}

impl Expects for ExpectedReply {
    fn reply_len(&self, id: u8) -> Option<usize> {
        self.ids.contains(&id).then_some(self.total)
    }
}

/// 从字节流里重组状态帧。
///
/// 容忍半双工总线实际会出现的情况：收发器回显我们自己写的帧、读取被截断、舵机应答晚到
/// 落在下一帧之后。任何「不是我们正在等的应答」都按字节丢弃，载荷里出现的假 `FF FF`
/// 最多引发一次重同步，而不是搞坏后面的整个运行。
#[derive(Debug, Default)]
pub struct FrameReader {
    buf: Vec<u8>,
    /// 重同步过程中丢弃的字节数（健康统计）。
    discarded: u64,
}

impl FrameReader {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push(&mut self, bytes: &[u8]) {
        self.buf.extend_from_slice(bytes);
    }

    pub fn clear(&mut self) {
        self.buf.clear();
    }

    /// 距上次调用以来丢弃的字节数。
    pub fn take_discarded(&mut self) -> u64 {
        std::mem::take(&mut self.discarded)
    }

    pub fn buffered(&self) -> usize {
        self.buf.len()
    }

    /// 取出下一个调用方正在等的、完整且校验通过的应答帧。
    ///
    /// 返回 None 表示「还需要更多字节」；校验失败的帧也会以 None 结束——在回声和真应答共用
    /// 一条线的总线上，损坏的帧就是丢一个字节，而不是要上报给上层的错误。
    pub fn next_frame(&mut self, expected: &impl Expects) -> Option<Vec<u8>> {
        loop {
            let Some(start) = self.buf.windows(2).position(|w| w == [0xFF, 0xFF]) else {
                // 没有帧头：保留最后一个字节（它可能是半个 FF），其余丢弃。
                let drop = self.buf.len().saturating_sub(1);
                self.discarded += drop as u64;
                self.buf.drain(..drop);
                return None;
            };
            if start > 0 {
                self.discarded += start as u64;
                self.buf.drain(..start);
            }
            if self.buf.len() < 4 {
                return None;
            }
            // 在 id 确认是我们点名的设备之前，长度字节只是个提示。
            let Some(total) = expected.reply_len(self.buf[2]) else {
                self.discarded += 1;
                self.buf.remove(0);
                continue;
            };
            if self.buf[3] as usize + 4 != total || total > MAX_FRAME {
                self.discarded += 1;
                self.buf.remove(0);
                continue;
            }
            if self.buf.len() < total {
                return None;
            }
            let frame: Vec<u8> = self.buf.drain(..total).collect();
            match parse_ack(&frame) {
                Ok(_) => return Some(frame),
                Err(_) => {
                    self.discarded += total as u64;
                    continue;
                }
            }
        }
    }
}

// ── 单位换算 ─────────────────────────────────────────────────────────────────

/// 15 位符号-幅值（bit15 方向位）：位置、速度、电流都用它。
///
/// 注意不是二进制补码——这在关节越过编码器中点之前看不出差别，越过之后会被读成一个巨大的
/// 正角度（0x8001 在这里是 −1，在补码里是 −32767）。
pub fn sign15(raw: u16) -> i32 {
    let magnitude = (raw & 0x7FFF) as i32;
    if raw & 0x8000 != 0 {
        -magnitude
    } else {
        magnitude
    }
}

/// 10 位符号-幅值（bit10 方向位）：负载。
pub fn sign10(raw: u16) -> i32 {
    let magnitude = (raw & 0x3FF) as i32;
    if raw & 0x400 != 0 {
        -magnitude
    } else {
        magnitude
    }
}

/// 把带符号的多圈计数编码成飞特位置字（bit15 方向 + 低 15 位幅值）。
pub fn position_counts_to_raw(counts: i32) -> u16 {
    let magnitude = counts.unsigned_abs().min(0x7FFF) as u16;
    if counts < 0 {
        magnitude | 0x8000
    } else {
        magnitude
    }
}

/// 每圈计数：单位 0.087°，即 360° 对应 4096 计数。
/// 16 位字段范围 −32767..32767，是多圈（约 ±8 圈），所以关节转过一圈读数会超过 4095 而不回绕。
pub const POSITION_COUNTS_PER_REV: f64 = 4096.0;

/// 每计数对应弧度：2π / 4096。
pub const RAD_PER_COUNT: f64 = std::f64::consts::TAU / POSITION_COUNTS_PER_REV;

/// 速度每计数对应 RPM（手册示例 60 * 0.732 = 43.92 rpm）。
pub const RPM_PER_SPEED_COUNT: f64 = 0.732;

pub const RAD_PER_SEC_PER_SPEED_COUNT: f64 = RPM_PER_SPEED_COUNT * std::f64::consts::TAU / 60.0;

/// 电流每计数对应毫安（示例 300 * 6.5 = 1950 mA）。
pub const MA_PER_CURRENT_COUNT: f64 = 6.5;

/// 电压寄存器每计数对应伏特。
pub const VOLTS_PER_COUNT: f64 = 0.1;

pub fn le_u16(low: u8, high: u8) -> u16 {
    u16::from_le_bytes([low, high])
}

/// 从两个字节读位置（弧度）。
pub fn position_rad(low: u8, high: u8) -> f64 {
    sign15(le_u16(low, high)) as f64 * RAD_PER_COUNT
}

/// 把目标弧度编码成两个小端字节。
pub fn position_from_rad(rad: f64) -> [u8; 2] {
    let counts = (rad / RAD_PER_COUNT).round() as i32;
    position_counts_to_raw(counts).to_le_bytes()
}

// ── 离线单测：逐字节对齐真机与官方库 ─────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// 构造一个 id 的应答帧（带 data），校验和算好。测试帧统一在这里生成，避免手写时
    /// 校验和与 id 对不上。
    fn ack_frame(id: u8, status: u8, data: &[u8]) -> Vec<u8> {
        let len = data.len() as u8 + 2;
        let chk = checksum(id, len, status, 0, data);
        let mut frame = vec![0xFF, 0xFF, id, len, status];
        frame.extend_from_slice(data);
        frame.push(chk);
        frame
    }

    // 参考向量：来自真机实测，且与官方库 / 节点 C 测试一致。

    #[test]
    fn ping_200_参考向量() {
        // ~(200+2+1) = 0x34
        assert_eq!(ping(200), vec![0xFF, 0xFF, 0xC8, 0x02, 0x01, 0x34]);
    }

    #[test]
    fn read_200_56_12_参考向量() {
        // ~(200+4+2+56+12) = 0xED
        assert_eq!(
            read(200, reg::PRESENT_POSITION_L, 12),
            vec![0xFF, 0xFF, 0xC8, 0x04, 0x02, 0x38, 0x0C, 0xED]
        );
    }

    #[test]
    fn reboot_1_参考向量() {
        assert_eq!(reboot(1), vec![0xFF, 0xFF, 0x01, 0x02, 0x08, 0xF4]);
    }

    #[test]
    fn 整机16设备_sync_read_实测布局() {
        // IMU 节点在前，再按 id 排列 15 颗舵机，地址 56、读 15 字节。
        let ids = [200, 10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34];
        assert_eq!(
            sync_read(&ids, reg::PRESENT_POSITION_L, block::LEN as u8),
            vec![
                0xFF, 0xFF, 0xFE, 0x14, 0x82, 0x38, 0x0F, 0xC8, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x14,
                0x15, 0x16, 0x17, 0x18, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x12
            ]
        );
    }

    #[test]
    fn sync_write_15目标_单帧() {
        let ids: Vec<u8> = (1..=15).collect();
        let payloads: Vec<[u8; 2]> = (0..15).map(|i| (1000 + i as u16).to_le_bytes()).collect();
        let refs: Vec<&[u8]> = payloads.iter().map(|p| p.as_slice()).collect();
        let frame = sync_write(&ids, reg::GOAL_POSITION_L, &refs).unwrap();
        // (2+1)*15 + 4 = 49
        assert_eq!(frame[3] as usize, 3 * 15 + 4);
        assert_eq!(frame.len(), frame[3] as usize + 4);
    }

    #[test]
    fn sync_write_拒绝参差不齐的输入() {
        assert!(sync_write(&[1, 2], reg::GOAL_POSITION_L, &[&[0, 0]]).is_err());
        assert!(sync_write(&[1, 2], reg::GOAL_POSITION_L, &[&[0, 0], &[0, 0, 0]]).is_err());
        assert!(sync_write(&[], reg::GOAL_POSITION_L, &[]).is_err());
    }

    #[test]
    fn calibrate_帧形状() {
        // 无参：LEN=2
        assert_eq!(calibrate(31, None), vec![0xFF, 0xFF, 31, 0x02, 0x0B, checksum(31, 2, 0x0B, 0, &[])]);
        // 带目标值 2048 = 0x0800，小端 00 08，LEN=4
        let frame = calibrate(31, Some(2048));
        assert_eq!(&frame[0..5], &[0xFF, 0xFF, 31, 0x04, 0x0B]);
        assert_eq!(&frame[5..7], &[0x00, 0x08]);
    }

    #[test]
    fn ack_往返且校验和是关键() {
        // FF FF C8 11 00 <00..0E> BD
        let mut frame = vec![0xFF, 0xFF, 200, 0x11, 0x00];
        frame.extend(0u8..=0x0E);
        frame.push(0xBD);
        let ack = parse_ack(&frame).unwrap();
        assert_eq!(ack.id, 200);
        assert_eq!(ack.status, 0);
        assert_eq!(ack.data.len(), 15);

        let mut bad = frame.clone();
        *bad.last_mut().unwrap() ^= 0x01;
        assert_eq!(parse_ack(&bad), Err(Error::BadFrame("校验和不符")));
    }

    #[test]
    fn 读取器_穿过噪声与短帧重同步() {
        let expected = ExpectedReply::new(&[200], 15);
        let good = ack_frame(200, 0x00, &(0u8..=0x0E).collect::<Vec<_>>());
        assert_eq!(*good.last().unwrap(), 0xBD);

        let mut reader = FrameReader::new();
        reader.push(b"debug text\r\n");
        reader.push(&[0xFF, 0xFF, 0xC8]);
        assert_eq!(reader.next_frame(&expected), None, "残缺帧不算帧");

        reader.push(&good);
        assert_eq!(reader.next_frame(&expected).as_deref(), Some(good.as_slice()));
        assert!(reader.take_discarded() >= 14, "调试文本应被丢弃");

        // 帧头像但不是有效帧，后面跟真帧，真帧要能取出。
        reader.push(&[0xFF, 0xFF, 0x01, 0x02, 0x03, 0xFF]);
        reader.push(&good);
        assert_eq!(reader.next_frame(&expected).as_deref(), Some(good.as_slice()));

        // 校验失败的帧被丢弃，不是被返回。
        let mut corrupt = good.clone();
        *corrupt.last_mut().unwrap() ^= 0x01;
        reader.push(&corrupt);
        assert_eq!(reader.next_frame(&expected), None);
        reader.push(&good);
        assert_eq!(reader.next_frame(&expected).as_deref(), Some(good.as_slice()));

        // 比请求长的应答判为错误帧，而不是被当成更长的一帧。
        let long = ack_frame(200, 0x00, &[0u8; 20]);
        reader.push(&long);
        assert_eq!(reader.next_frame(&expected), None);
        reader.push(&good);
        assert_eq!(reader.next_frame(&expected).as_deref(), Some(good.as_slice()));
    }

    #[test]
    fn 读取器_忽略我们自己的指令回声() {
        // 长度盲读取器最怕的情况：我们自己的写帧挡在应答前面，应答仍要被取出。
        let expected = ExpectedReply::new(&[10], 15);
        let good = ack_frame(10, 0x00, &(0u8..=0x0E).collect::<Vec<_>>());
        let mut reader = FrameReader::new();
        // 给 id 10 上电：FF FF 0A 04 03 28 01 C5，同 id、长度也看起来合法。
        reader.push(&write(10, reg::TORQUE_ENABLE, &[1]));
        reader.push(&good);
        assert_eq!(reader.next_frame(&expected).as_deref(), Some(good.as_slice()));
    }

    #[test]
    fn 位置是符号幅值而非补码() {
        assert_eq!(sign15(0x0000), 0);
        assert_eq!(sign15(0x0001), 1);
        assert_eq!(sign15(0x8001), -1);
        assert_eq!(sign15(0x7FFF), 32767);
        assert_eq!(sign15(0xFFFF), -32767);
        // 多圈：转过一圈读数为 6289 而不是回绕。
        assert_eq!(sign15(6289), 6289);
    }

    #[test]
    fn 负载是bit10符号幅值() {
        assert_eq!(sign10(0x0000), 0);
        assert_eq!(sign10(0x0005), 5);
        assert_eq!(sign10(0x0405), -5);
        assert_eq!(sign10(0x03FF), 1023);
        assert_eq!(sign10(0x07FF), -1023);
    }

    #[test]
    fn 位置弧度往返() {
        for counts in [-32767i32, -4096, -1, 0, 1, 2048, 4096, 32767] {
            let raw = position_counts_to_raw(counts);
            let rad = sign15(raw) as f64 * RAD_PER_COUNT;
            let bytes = position_from_rad(rad);
            assert_eq!(sign15(le_u16(bytes[0], bytes[1])), counts, "计数 {counts} 往返失败");
        }
    }

    #[test]
    fn 一圈就是2pi() {
        assert!((RAD_PER_COUNT * POSITION_COUNTS_PER_REV - std::f64::consts::TAU).abs() < 1e-12);
        assert!((RPM_PER_SPEED_COUNT * 60.0 - 43.92).abs() < 1e-9);
    }

    #[test]
    fn tick块偏移与寄存器对齐() {
        let base = reg::PRESENT_POSITION_L;
        assert_eq!(base + block::POSITION as u8, reg::PRESENT_POSITION_L);
        assert_eq!(base + block::SPEED as u8, reg::PRESENT_SPEED_L);
        assert_eq!(base + block::LOAD as u8, reg::PRESENT_LOAD_L);
        assert_eq!(base + block::VOLTAGE as u8, reg::PRESENT_VOLTAGE);
        assert_eq!(base + block::TEMPERATURE as u8, reg::PRESENT_TEMPERATURE);
        assert_eq!(base + block::STATUS as u8, reg::PRESENT_STATUS);
        assert_eq!(base + block::CURRENT as u8, reg::PRESENT_CURRENT_L);
        assert_eq!(block::LEN, 15);
        assert_eq!(block::CURRENT + 2, block::LEN);
    }

    #[test]
    fn 增益块一次覆盖增益和慢变量() {
        let base = reg::KP;
        assert_eq!(base + gains_block::KP as u8, reg::KP);
        assert_eq!(base + gains_block::KD as u8, reg::KD);
        assert_eq!(base + gains_block::LOCK as u8, reg::LOCK);
        assert_eq!(base + gains_block::VOLTAGE as u8, reg::PRESENT_VOLTAGE);
        assert_eq!(base + gains_block::TEMPERATURE as u8, reg::PRESENT_TEMPERATURE);
        assert_eq!(gains_block::LEN as u8, reg::PRESENT_TEMPERATURE - base + 1);
    }
}
