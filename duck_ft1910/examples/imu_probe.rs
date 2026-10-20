//! IMU 探针：只读飞特总线上 200 号 IMU 板的三轴姿态，验证安装方向与 imu_mount 配置。
//!
//! 用法（板子上，先停掉 robotd 避免抢串口）：
//! ```text
//! sudo /tmp/imu_probe [/dev/ttyS2] [/etc/robot/ft1910.toml]
//! ```
//!
//! 摆好姿势后读到的 gravity 若与上表不符，说明 imu_mount 或安装方向需要调整。

use std::env;
use std::thread::sleep;
use std::time::{Duration, Instant};

use duck_control::io::RobotIo;
use duck_ft1910::bus::Calibration;
use duck_ft1910::Ft1910Io;

const DEFAULT_PORT: &str = "/dev/ttyS2";
const DEFAULT_CONFIG: &str = "/etc/robot/ft1910.toml";

fn main() {
    let args: Vec<String> = env::args().collect();
    let port = args.get(1).map(String::as_str).unwrap_or(DEFAULT_PORT);
    let config = args.get(2).map(String::as_str).unwrap_or(DEFAULT_CONFIG);

    let cal = match Calibration::from_file(config) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("读取标定文件 {config} 失败: {e}");
            std::process::exit(1);
        }
    };
    println!(
        "标定: imu_id={} baud={} imu_mount=[{:+.3}, {:+.3}, {:+.3}, {:+.3}]",
        cal.imu_id, cal.baud_rate, cal.imu_mount[0], cal.imu_mount[1], cal.imu_mount[2], cal.imu_mount[3]
    );

    let mut io = match Ft1910Io::open(port, cal) {
        Ok(io) => io,
        Err(e) => {
            eprintln!("打开总线 {port} 失败: {e}");
            std::process::exit(1);
        }
    };

    println!("IMU 探针就绪，总线 {port}，每 1000 ms 打印一行，Ctrl+C 退出。");
    println!("对照参考：");
    println!("  立正   [ 0,  0, -1]");
    println!("  仰躺   [ -1, 0,  0]");
    println!("  左侧倒 [ 0,  1,  0]");
    
    println!("---");

    let start = Instant::now();
    loop {
        match io.read() {
            Ok(sensors) => {
                let g = sensors.imu.gravity;
                let q = sensors.imu.quat;
                let gy = sensors.imu.gyro;
                println!(
                    "t={:6.1}s gravity=[{:+.3},{:+.3},{:+.3}] quat=[{:+.3},{:+.3},{:+.3},{:+.3}] gyro=[{:+.2},{:+.2},{:+.2}]",
                    start.elapsed().as_secs_f64(),
                    g[0], g[1], g[2],
                    q[0], q[1], q[2], q[3],
                    gy[0], gy[1], gy[2],
                );
            }
            Err(e) => {
                println!("read 失败: {e}");
            }
        }
        sleep(Duration::from_secs(1));
    }
}
