//! 飞特扩展数据通道：逐颗电压 / 温度 / 负载。
//!
//! 官方 robotd 的 monitor 帧只透传位置/速度/电流；电压（仅平均）、温度（仅聚合）
//! 走 health——逐颗的电压、温度、负载在官方接口层根本没有字段。本模块在本 crate
//! 内闭环补齐：[`Ft1910Io::read`] 每次把逐颗样本写进共享缓存，一个后台线程每 100ms
//! 把缓存原子写到板子本地文件 `/run/microduck_ext.json`。主机侧 bridge 复用已有 SSH
//! 通道每 ~1.5s 拉一次该文件即可——官方代码一行不改。
//!
//! 写文件而不是开 TCP 端口：板子侧不新增任何网络服务，主机侧也不新增端口，完全复用
//! bridge 的 SSH 通道（与 health 采样同一模式）。文件落在 `/run`（tmpfs 内存盘），
//! 几百字节、无 SD 卡磨损，重启自动清空重写。

use std::fs;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use duck_control::model::NUM_JOINTS;

/// 逐颗扩展样本（一次 SYNC_READ 顺带解出的值）。
#[derive(Debug, Clone, Copy, Default, serde::Serialize)]
pub struct ExtSample {
    /// 最近成功解析帧的主机侧 Unix 时间戳（秒）。
    pub ts: f64,
    /// 每颗电压 V（寄存器 62 × 0.1V）。
    pub volts: [f64; NUM_JOINTS],
    /// 每颗温度 ℃（寄存器 63，直接是摄氏度）。
    pub temps_c: [f64; NUM_JOINTS],
    /// 每颗负载 %（寄存器 60 ÷ 10，带符号：正=堵转方向，负=反向）。
    pub loads: [f64; NUM_JOINTS],
}

/// 共享缓存句柄（bus.rs 持有，`read()` 每帧写入，后台线程只读）。
pub type ExtShared = Arc<Mutex<ExtSample>>;

/// 默认落盘路径（tmpfs 内存盘，重启清空）。可用环境变量 `FT1910_EXT_PATH` 覆盖。
pub fn default_path() -> PathBuf {
    std::env::var_os("FT1910_EXT_PATH")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("/run/microduck_ext.json"))
}

fn now_secs() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}

/// 后台线程：每 `interval` 读一次缓存，把样本原子写到 `path`。
///
/// 原子写 = 先写 `path.tmp` 再 rename，bridge 的 cat 绝不会读到半行 JSON。
/// 写失败（路径不存在/权限/磁盘满）静默跳过本轮并每 5s 限速报一次错，不 panic——
/// 扩展通道绝不能影响 50Hz 控制环。
pub fn spawn_writer(shared: ExtShared, interval: Duration, path: PathBuf) -> Arc<PathBuf> {
    let path = Arc::new(path);
    let p = path.clone();
    std::thread::spawn(move || {
        let mut last_err_log = 0.0f64;
        loop {
            std::thread::sleep(interval);
            let sample = shared.lock().map(|s| *s).unwrap_or_default();
            let json = serde_json::to_string(&sample).unwrap_or_default();
            let tmp = p.as_ref().with_extension("json.tmp");
            let ok = fs::write(&tmp, json.as_bytes())
                .and_then(|_| fs::rename(&tmp, &*p))
                .is_ok();
            if !ok {
                let now = now_secs();
                if now - last_err_log > 5.0 {
                    last_err_log = now;
                    tracing::warn!(path = %p.display(), "ft1910 ext 落盘失败（忽略，通道继续）");
                }
            }
        }
    });
    path
}
