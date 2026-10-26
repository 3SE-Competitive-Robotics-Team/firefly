//! Linux 板端的 configfs/`FunctionFS` 准备与绑定。
//!
//! 每次 `up`/`down` 都**整块重建** `g1`：configfs 在绑定失败后会保留
//! `udc_name`，重试只会得到 `EBUSY`，反复失败还会推进 `next_interface_id`。
//! 先删除 gadget 再按固定顺序创建，计数器与接口号才是干净、可复现的。
//!
//! HIL 与 `adb` 二选一：这块 UDC 的端点数量不足以同时容纳 adb 与 HIL 双向
//! 端点，绑定失败会一并带走 adb。所以 `up` 只创建单一的 `hil` 功能，`down`
//! 与看门狗重建 adb-only gadget 恢复远程访问。
//!
//! 所有步骤写 [`UP_LOG`] 并 `fsync`：TF 卡 `commit=600`，断电会丢未落盘写入，
//! 而排障恰恰发生在 USB 已经不可用的时刻。

use std::fs;
use std::io::{Read, Write};
use std::path::Path;
use std::process::{Command, Stdio};
use std::thread;
use std::time::{Duration, Instant};

use firefly_hil::gadget::{HIL, write_descriptors};

/// configfs gadget 目录（可整块删除重建）。
const GADGET: &str = "/sys/kernel/config/usb_gadget/g1";
/// `FunctionFS` 挂载根。
const FFS_ROOT: &str = "/dev/usb-ffs";
/// 运行时状态与日志目录（板载安装布局，落在磁盘）。
const STATE_DIR: &str = "/opt/firefly/logs/hil";
/// `up` 的进度日志。
const UP_LOG: &str = "/opt/firefly/logs/hil/up.log";
/// `serve` 写完描述符后创建的就绪标记。
const READY: &str = "/opt/firefly/logs/hil/descriptors_ready";
/// `up` 完成绑定后创建的标记。
const UP_MARKER: &str = "/opt/firefly/logs/hil/up";
/// adb 恢复成功标记（看门狗据此避免重复重建）。
const RESTORE_MARKER: &str = "/opt/firefly/logs/hil/restored";
/// `serve` 进程号文件（`start-stop-daemon` 写）。
const SERVE_PID: &str = "/opt/firefly/logs/hil/serve.pid";
/// 看门狗进程号文件。
const WATCHDOG_PID: &str = "/opt/firefly/logs/hil/watchdog.pid";
/// adbd 可执行文件。
const ADBD: &str = "/usr/bin/adbd";
/// Debian 的后台进程启动工具。
const START_STOP_DAEMON: &str = "/usr/sbin/start-stop-daemon";
/// adb 功能实例名。
const ADB: &str = "adb";
/// 启动后等待 adbd 写完描述符的时间。
const ADBD_DESCRIPTOR_WAIT: Duration = Duration::from_secs(1);
/// 等待 `serve` 写描述符的上限。
const READY_TIMEOUT: Duration = Duration::from_secs(5);
/// 等待 UDC 解绑/绑定回读的上限。
const BIND_TIMEOUT: Duration = Duration::from_secs(3);
/// 看门狗等待 `up` 完成绑定的上限；超时即重建 adb-only gadget。
const WATCHDOG_TIMEOUT: Duration = Duration::from_secs(60);
/// 活动进程清单。
const PROCESSES: &str = "/opt/firefly/configs/processes.toml";
/// HIL 进程清单（deploy 随 configs 一起发布）。
const PROCESSES_HIL: &str = "/opt/firefly/configs/processes-hil.toml";
/// 切换前的进程清单备份，恢复时原样写回。
const PROCESSES_BACKUP: &str = "/opt/firefly/configs/processes-normal.toml";
/// 主机心跳文件：板端 HIL 应用收到任意上行数据后刷新。
const HOST_SEEN: &str = "/opt/firefly/logs/hil/host_seen";
/// 主机无数据超过该时长即恢复 adb，避免 HIL 失败后失去远程访问。
const HOST_IDLE_TIMEOUT: Duration = Duration::from_secs(60);

/// 所有可能出现的功能实例名。
const FUNCTIONS: [&str; 2] = [ADB, HIL.name];

pub fn up() -> Result<(), String> {
    let _ = fs::create_dir_all(STATE_DIR);
    reset_log();
    note("开始：停止 firefly 与 adbd，整块重建 gadget");
    if let Err(error) = up_inner() {
        note(&format!("up 失败：{error}"));
        eprintln!("hil-gadget up 失败：{error}；尝试恢复 adb");
        if let Err(restore) = restore_adb() {
            eprintln!("恢复 adb 失败：{restore}；需要重启板子");
        }
        return Err(error);
    }
    Ok(())
}

fn up_inner() -> Result<(), String> {
    remove_file(READY)?;
    remove_file(UP_MARKER)?;
    remove_file(RESTORE_MARKER)?;
    stop_firefly();
    start_watchdog()?;
    stop_adb_rebind_loop();
    kill_serve();
    kill_adbd();
    set_device_role();
    switch_processes(true)?;
    rebuild(true)?;
    start_firefly();
    fs::write(UP_MARKER, format!("{}\n", now_seconds()))
        .map_err(|error| format!("write {UP_MARKER}: {error}"))?;
    sync_disk();
    note("HIL gadget 已绑定：单个 hil 功能（OUT + IN）");
    println!("HIL gadget 已绑定：单个 hil 功能（OUT + IN）");
    Ok(())
}

pub fn serve() -> Result<(), String> {
    note("serve：打开 ep0 并写描述符");
    let mut ep0 = open_ep0(HIL.name)?;
    write_descriptors(&mut ep0, &HIL)
        .map_err(|error| format!("write {} descriptors: {error}", HIL.name))?;
    fs::write(READY, format!("{}\n", std::process::id()))
        .map_err(|error| format!("write {READY}: {error}"))?;
    note("serve：描述符已写入，保持 ep0");
    if let Err(error) = hold(&mut ep0) {
        note(&format!("serve 退出：{error}"));
    }
    Ok(())
}

/// 按 `sensor` / `actuator` / `both` 建功能并记录绑定结果，用于定位 UDC 绑定失败。
pub fn diag(mode: &str) -> Result<(), String> {
    if mode != "hil" {
        return Err(format!("diag 只支持 hil，实际 {mode}"));
    }
    reset_log();
    note("diag：单个 hil 功能");
    stop_firefly();
    stop_adb_rebind_loop();
    kill_serve();
    kill_adbd();
    let result = diag_inner(&[&HIL]);
    if let Err(error) = &result {
        note(&format!("diag 失败：{error}"));
    }
    note("diag：恢复 adb-only gadget");
    let restore = restore_adb();
    result.and(restore)
}

fn diag_inner(functions: &[&firefly_hil::gadget::Function]) -> Result<(), String> {
    destroy_gadget()?;
    ensure_gadget()?;
    for function in functions {
        create_function(function.name)?;
        mount_function(function.name)?;
        link_function(function.name)?;
    }
    start_serve()?;
    wait_ready()?;
    for function in functions {
        let entries = describe_dir(&Path::new(FFS_ROOT).join(function.name));
        note(&format!("diag：{} 端点文件 {entries}", function.name));
    }
    note(&format!(
        "diag：UDC 绑定前 UDC={:?} 状态={:?}",
        read_trimmed(Path::new(GADGET).join("UDC").as_path()),
        read_trimmed(Path::new("/sys/class/udc/4100000.udc-controller/state"))
    ));
    match bind_udc() {
        Ok(()) => {
            for function in functions {
                let entries = describe_dir(&Path::new(FFS_ROOT).join(function.name));
                note(&format!(
                    "diag：绑定成功，{} 端点文件 {entries}",
                    function.name
                ));
            }
            Ok(())
        }
        Err(error) => {
            note(&format!("diag：绑定失败 {error}"));
            Err(error)
        }
    }
}

fn describe_dir(path: &Path) -> String {
    fs::read_dir(path).map_or_else(
        |error| format!("读目录失败 {error}"),
        |dir| {
            let mut names: Vec<String> = dir
                .filter_map(std::result::Result::ok)
                .map(|entry| entry.file_name().to_string_lossy().into_owned())
                .collect();
            names.sort();
            format!("{names:?}")
        },
    )
}

pub fn down() -> Result<(), String> {
    note("down：停止 firefly，恢复 adb-only gadget");
    stop_firefly();
    kill_serve();
    restore_adb()?;
    remove_file(UP_MARKER)?;
    println!("HIL gadget 已移除，adb gadget 已恢复");
    Ok(())
}

pub fn watchdog() -> Result<(), String> {
    thread::sleep(WATCHDOG_TIMEOUT);
    loop {
        if !Path::new(UP_MARKER).is_file() || Path::new(RESTORE_MARKER).is_file() {
            note("watchdog：未完成绑定，重建 adb-only gadget");
            return restore_adb();
        }
        let idle = fs::metadata(HOST_SEEN)
            .and_then(|meta| meta.modified())
            .ok()
            .and_then(|modified| modified.elapsed().ok())
            .unwrap_or(Duration::MAX);
        if idle > HOST_IDLE_TIMEOUT {
            note(&format!(
                "watchdog：主机 {idle:?} 无数据，重建 adb-only gadget"
            ));
            return restore_adb();
        }
        thread::sleep(Duration::from_secs(5));
    }
}

pub fn status() {
    println!(
        "UDC: {}",
        read_trimmed(Path::new(GADGET).join("UDC").as_path()).unwrap_or_default()
    );
    for name in FUNCTIONS {
        let mount = Path::new(FFS_ROOT).join(name);
        let entries: Vec<String> = fs::read_dir(&mount)
            .map(|dir| {
                dir.filter_map(std::result::Result::ok)
                    .map(|entry| entry.file_name().to_string_lossy().into_owned())
                    .collect()
            })
            .unwrap_or_default();
        println!("{}: {entries:?}", mount.display());
    }
}

/// 重建 gadget：`hil = true` 只建 `hil` 功能，`false` 只建 adb。
fn rebuild(hil: bool) -> Result<(), String> {
    match rebuild_once(hil) {
        Ok(()) => Ok(()),
        Err(error) => {
            // 首次绑定可能撞上 UDC 异步解绑的尾巴；整块重建一次是干净的复位，
            // 与「重试写 UDC」不同，不会留下卡住的 udc_name/接口计数。
            note(&format!("rebuild 首次失败：{error}；整块重建后重试"));
            thread::sleep(Duration::from_millis(500));
            rebuild_once(hil)
        }
    }
}

fn rebuild_once(hil: bool) -> Result<(), String> {
    destroy_gadget()?;
    ensure_gadget()?;
    if hil {
        create_function(HIL.name)?;
        mount_function(HIL.name)?;
        link_function(HIL.name)?;
        start_serve()?;
        wait_ready()?;
    } else {
        create_function(ADB)?;
        mount_function(ADB)?;
        link_function(ADB)?;
        kill_adbd();
        spawn_adbd()?;
        thread::sleep(ADBD_DESCRIPTOR_WAIT);
    }
    bind_udc()?;
    cycle_link();
    if hil {
        note(&format!(
            "绑定后 {} 端点文件 {:?}",
            HIL.name,
            describe_dir(&Path::new(FFS_ROOT).join(HIL.name))
        ));
    }
    note(&format!(
        "绑定后 UDC 状态 {:?}",
        read_trimmed(Path::new("/sys/class/udc/4100000.udc-controller/state"))
    ));
    Ok(())
}

/// 解绑并删除整个 `g1`，清掉残留的 `udc_name` 与接口计数。
fn destroy_gadget() -> Result<(), String> {
    let udc_path = Path::new(GADGET).join("UDC");
    if udc_path.exists() {
        match fs::write(&udc_path, "\n") {
            Ok(()) => {}
            Err(error) if error.raw_os_error() == Some(19) => {
                note("destroy：UDC 已处于解绑状态");
            }
            Err(error) => note(&format!("destroy：解绑 UDC 失败（继续删除）：{error}")),
        }
        wait_udc_empty(&udc_path, BIND_TIMEOUT);
    }
    for name in FUNCTIONS {
        let link = Path::new(GADGET)
            .join("configs/c.1")
            .join(format!("ffs.{name}"));
        remove_file(&link)?;
        umount(&Path::new(FFS_ROOT).join(name))?;
    }
    for name in FUNCTIONS {
        let function = Path::new(GADGET)
            .join("functions")
            .join(format!("ffs.{name}"));
        if function.exists() {
            remove_dir_with_retry(&function)?;
        }
    }
    for path in [
        "configs/c.1/strings/0x409",
        "configs/c.1",
        "strings/0x409",
        "",
    ] {
        let dir = Path::new(GADGET).join(path);
        if dir.exists() {
            remove_dir_with_retry(&dir).map_err(|error| format!("destroy g1/{path}: {error}"))?;
        }
    }
    note("destroy：g1 已删除，计数器清零");
    Ok(())
}

fn ensure_gadget() -> Result<(), String> {
    for path in [
        "",
        "strings/0x409",
        "configs/c.1",
        "configs/c.1/strings/0x409",
        "functions",
    ] {
        let dir = Path::new(GADGET).join(path);
        fs::create_dir_all(&dir).map_err(|error| format!("create {}: {error}", dir.display()))?;
    }
    // configfs 新建 gadget 的 idVendor/idProduct 默认是 "0x0000"（不是空串），
    // 必须无条件写入，否则设备描述符会以 0000:0000 枚举。
    write_value(&Path::new(GADGET).join("idVendor"), "0x18d1")?;
    write_value(&Path::new(GADGET).join("idProduct"), "0x0002")?;
    write_value(
        &Path::new(GADGET).join("strings/0x409/manufacturer"),
        "Google.Inc",
    )?;
    write_value(
        &Path::new(GADGET).join("strings/0x409/product"),
        "Configfs ffs gadget",
    )?;
    write_value(
        &Path::new(GADGET).join("strings/0x409/serialnumber"),
        &adb_serial(),
    )?;
    write_value(&Path::new(GADGET).join("configs/c.1/bmAttributes"), "0xc0")?;
    write_value(&Path::new(GADGET).join("configs/c.1/MaxPower"), "500")
}

fn bind_udc() -> Result<(), String> {
    let udc = udc_name()?;
    let path = Path::new(GADGET).join("UDC");
    fs::write(&path, format!("{udc}\n")).map_err(|error| {
        format!(
            "写 UDC {udc} 失败：{error}；内核日志：{}",
            kernel_gadget_log()
        )
    })?;
    if wait_udc_value(&path, &udc, BIND_TIMEOUT) {
        return Ok(());
    }
    Err(format!(
        "UDC {udc} 绑定后未生效；内核日志：{}",
        kernel_gadget_log()
    ))
}

fn wait_udc_value(path: &Path, expected: &str, timeout: Duration) -> bool {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        if read_trimmed(path).as_deref() == Some(expected) {
            return true;
        }
        thread::sleep(Duration::from_millis(20));
    }
    false
}

fn wait_udc_empty(path: &Path, timeout: Duration) -> bool {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        if read_trimmed(path).is_some_and(|value| value.is_empty()) {
            return true;
        }
        thread::sleep(Duration::from_millis(20));
    }
    false
}

fn udc_name() -> Result<String, String> {
    let dir =
        fs::read_dir("/sys/class/udc").map_err(|error| format!("read /sys/class/udc: {error}"))?;
    let mut names: Vec<String> = dir
        .filter_map(std::result::Result::ok)
        .map(|entry| entry.file_name().to_string_lossy().into_owned())
        .collect();
    names.sort();
    let [name] = names.as_slice() else {
        return Err(format!("期望唯一 UDC，实际 {names:?}"));
    };
    Ok(name.clone())
}

fn adb_serial() -> String {
    fs::read_to_string("/etc/firefly/adb-serial")
        .ok()
        .and_then(|text| text.lines().next().map(|line| line.trim().to_owned()))
        .filter(|line| !line.is_empty())
        .unwrap_or_else(|| "firefly-0001".to_owned())
}

fn create_function(name: &str) -> Result<(), String> {
    let function = Path::new(GADGET)
        .join("functions")
        .join(format!("ffs.{name}"));
    fs::create_dir(&function).map_err(|error| format!("create {}: {error}", function.display()))
}

fn mount_function(name: &str) -> Result<(), String> {
    let mountpoint = Path::new(FFS_ROOT).join(name);
    fs::create_dir_all(&mountpoint)
        .map_err(|error| format!("create {}: {error}", mountpoint.display()))?;
    if is_mounted(&mountpoint) {
        return Err(format!("{} 仍已挂载", mountpoint.display()));
    }
    let mut command = Command::new("mount");
    command.args(["-t", "functionfs"]);
    if name == ADB {
        command.args(["-o", "uid=2000,gid=2000"]);
    }
    command.arg(name).arg(&mountpoint);
    let output = command
        .output()
        .map_err(|error| format!("spawn mount: {error}"))?;
    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr).trim().to_owned();
        note(&format!("mount {name} 失败：{stderr}"));
        return Err(format!("mount {name}：{stderr}"));
    }
    Ok(())
}

fn link_function(name: &str) -> Result<(), String> {
    let link = Path::new(GADGET)
        .join("configs/c.1")
        .join(format!("ffs.{name}"));
    let target = Path::new(GADGET)
        .join("functions")
        .join(format!("ffs.{name}"));
    std::os::unix::fs::symlink(&target, &link)
        .map_err(|error| format!("link {}: {error}", link.display()))
}

fn restore_adb() -> Result<(), String> {
    note("restore：停止 HIL 进程，重建 adb-only gadget");
    // 先停 HIL 应用再撤 gadget：camera/motor 还持有 ffs 端点时重建 UDC，
    // 会让 adb 绑定随端点释放反复失败（表现为 adb 设备时有时无）。
    stop_firefly();
    kill_serve();
    kill_adbd();
    set_device_role();
    switch_processes(false)?;
    rebuild(false)?;
    start_firefly();
    note("adb gadget 已恢复");
    fs::write(RESTORE_MARKER, format!("{}\n", now_seconds())).ok();
    sync_disk();
    Ok(())
}

fn start_serve() -> Result<(), String> {
    start_daemon("serve", SERVE_PID)
}

fn start_watchdog() -> Result<(), String> {
    start_daemon("watchdog", WATCHDOG_PID)
}

fn start_daemon(command: &str, pidfile: &str) -> Result<(), String> {
    let exe = std::env::current_exe().map_err(|error| format!("current_exe: {error}"))?;
    // 多次 up 会留下多个 watchdog/serve；旧进程醒来会和新会话抢 UDC，
    // 表现为 HIL 与 adb 来回切换。启动前按 pidfile 结束同名旧进程。
    let _ = Command::new(START_STOP_DAEMON)
        .args([
            "--stop",
            "--pidfile",
            pidfile,
            "--retry",
            "TERM/2/KILL/1",
            "--oknodo",
        ])
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status();
    let status = Command::new(START_STOP_DAEMON)
        .args([
            "--start",
            "--background",
            "--make-pidfile",
            "--pidfile",
            pidfile,
            "--startas",
            &exe.to_string_lossy(),
            "--",
            command,
        ])
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map_err(|error| format!("spawn {command}: {error}"))?;
    if !status.success() {
        return Err(format!(
            "start-stop-daemon 启动 {command} 退出 {}",
            status.code().unwrap_or(-1)
        ));
    }
    Ok(())
}

fn wait_ready() -> Result<(), String> {
    let deadline = Instant::now() + READY_TIMEOUT;
    while Instant::now() < deadline {
        if Path::new(READY).is_file() {
            return Ok(());
        }
        thread::sleep(Duration::from_millis(20));
    }
    Err(format!("描述符写入超时；日志：{}", read_log()))
}

fn spawn_adbd() -> Result<(), String> {
    Command::new(ADBD)
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .map_err(|error| format!("spawn {ADBD}: {error}"))?;
    Ok(())
}

fn hold(ep0: &mut fs::File) -> std::io::Result<()> {
    // `usb_functionfs_event` 是 12 字节（8 字节 setup + type + pad）；读长度
    // 必须是它的整数倍，否则内核直接返回 EINVAL 并关闭 ep0。
    let mut event = [0_u8; 12];
    loop {
        match ep0.read(&mut event) {
            Ok(0) => return Ok(()),
            Ok(_) => {}
            Err(error) if error.kind() == std::io::ErrorKind::Interrupted => {}
            Err(error) => return Err(error),
        }
    }
}

fn open_ep0(name: &str) -> Result<fs::File, String> {
    let path = Path::new(FFS_ROOT).join(name).join("ep0");
    fs::OpenOptions::new()
        .read(true)
        .write(true)
        .open(&path)
        .map_err(|error| format!("open {}: {error}", path.display()))
}

/// 镜像启动脚本在后台常驻重绑 UDC；重建前必须让它退出，否则会抢绑。
fn stop_adb_rebind_loop() {
    let _ = fs::write("/etc/.disable_udc", "");
    thread::sleep(Duration::from_millis(2500));
    let _ = Command::new("pkill")
        .args(["-x", "adb_conf.sh"])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status();
    thread::sleep(Duration::from_millis(500));
}

/// 切换活动进程清单；HIL 文件缺失时明确失败，不静默沿用真机进程集。
fn switch_processes(hil: bool) -> Result<(), String> {
    if hil {
        if !Path::new(PROCESSES_HIL).is_file() {
            return Err(format!("缺少 {PROCESSES_HIL}（deploy 未带上）"));
        }
        if !Path::new(PROCESSES_BACKUP).exists() {
            fs::copy(PROCESSES, PROCESSES_BACKUP)
                .map_err(|error| format!("backup {PROCESSES}: {error}"))?;
        }
        fs::copy(PROCESSES_HIL, PROCESSES)
            .map_err(|error| format!("switch {PROCESSES}: {error}"))?;
        note("进程清单已切到 HIL");
    } else if Path::new(PROCESSES_BACKUP).exists() {
        fs::copy(PROCESSES_BACKUP, PROCESSES)
            .map_err(|error| format!("restore {PROCESSES}: {error}"))?;
        note("进程清单已恢复为真机");
    }
    sync_disk();
    Ok(())
}

/// 绑定后强制断开再接入：主机在同一 serial 上快速重枚举时常拒绝注册新配置。
fn cycle_link() {
    let path = "/sys/class/udc/4100000.udc-controller/soft_connect";
    if !Path::new(path).exists() {
        return;
    }
    let _ = fs::write(path, "disconnect\n");
    thread::sleep(Duration::from_secs(2));
    let _ = fs::write(path, "connect\n");
    thread::sleep(Duration::from_millis(800));
    note(&format!(
        "cycle_link 完成，UDC 状态 {:?}",
        read_trimmed(Path::new("/sys/class/udc/4100000.udc-controller/state"))
    ));
}

/// 解绑后 OTG 角色可能退回 host；厂商脚本也靠重写它恢复设备模式。
fn set_device_role() {
    let path = "/sys/devices/platform/soc@3000000/10.usbc0/otg_role";
    let current = read_trimmed(Path::new(path)).unwrap_or_default();
    if Path::new(path).exists() && current != "usb_device" {
        // 写这个属性会 rmmod/insmod USB 控制器；已经处于设备模式时不动它。
        let _ = fs::write(path, "usb_device\n");
    }
    note(&format!(
        "otg_role={:?}",
        read_trimmed(Path::new(path)).unwrap_or_default()
    ));
}

fn start_firefly() {
    let _ = Command::new("systemctl")
        .args(["start", "firefly.service"])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status();
    thread::sleep(Duration::from_millis(500));
}

fn stop_firefly() {
    let _ = Command::new("systemctl")
        .args(["stop", "firefly.service"])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status();
    thread::sleep(Duration::from_millis(300));
}

fn kill_adbd() {
    let _ = Command::new("pkill")
        .args(["-x", "adbd"])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status();
    // 内核在 adbd 关闭 ep0 后异步解绑 UDC；给驱动留出完成时间。
    thread::sleep(Duration::from_millis(700));
}

fn kill_serve() {
    let Some(pid) = read_trimmed(Path::new(SERVE_PID)).and_then(|text| text.parse::<i32>().ok())
    else {
        return;
    };
    let comm = fs::read_to_string(format!("/proc/{pid}/comm"))
        .ok()
        .map(|text| text.trim().to_owned());
    if comm.as_deref() != Some("hil-gadget") {
        remove_file(SERVE_PID).ok();
        return;
    }
    let _ = Command::new("kill").arg(pid.to_string()).status();
    let deadline = Instant::now() + Duration::from_secs(2);
    while Instant::now() < deadline {
        if !Path::new(&format!("/proc/{pid}")).exists() {
            break;
        }
        thread::sleep(Duration::from_millis(20));
    }
    remove_file(SERVE_PID).ok();
}

fn umount(path: &Path) -> Result<(), String> {
    if !is_mounted(path) {
        return Ok(());
    }
    let _ = Command::new("umount")
        .arg(path)
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status();
    if is_mounted(path) {
        note(&format!("umount {} 失败，改用惰性卸载", path.display()));
        let _ = Command::new("umount")
            .args(["-l"])
            .arg(path)
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status();
        thread::sleep(Duration::from_millis(100));
    }
    if is_mounted(path) {
        return Err(format!("{} 无法卸载", path.display()));
    }
    Ok(())
}

fn is_mounted(path: &Path) -> bool {
    let target = path.to_string_lossy();
    fs::read_to_string("/proc/self/mounts").is_ok_and(|mounts| {
        mounts
            .lines()
            .any(|line| line.split_whitespace().nth(1) == Some(target.as_ref()))
    })
}

fn remove_dir_with_retry(path: &Path) -> Result<(), String> {
    let mut last_error = String::new();
    for _ in 0..40 {
        match fs::remove_dir(path) {
            Ok(()) => return Ok(()),
            Err(error) => {
                last_error = format!("{error}");
                thread::sleep(Duration::from_millis(50));
            }
        }
    }
    Err(format!("remove {}: {last_error}", path.display()))
}

fn write_value(path: &Path, value: &str) -> Result<(), String> {
    fs::write(path, format!("{value}\n"))
        .map_err(|error| format!("write {}: {error}", path.display()))
}

fn read_trimmed(path: &Path) -> Option<String> {
    fs::read_to_string(path)
        .ok()
        .map(|text| text.trim().to_owned())
}

fn remove_file(path: impl AsRef<Path>) -> Result<(), String> {
    let path = path.as_ref();
    match fs::remove_file(path) {
        Ok(()) => Ok(()),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(()),
        Err(error) => Err(format!("remove {}: {error}", path.display())),
    }
}

/// 追加一行进度并 `fsync`：TF 卡 `commit=600`，断电会丢未落盘写入。
pub(crate) fn note(message: &str) {
    if let Ok(mut file) = fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(UP_LOG)
    {
        let _ = writeln!(file, "[{:.3}] {message}", now_seconds());
        let _ = file.sync_all();
    }
}

fn reset_log() {
    let _ = fs::write(
        UP_LOG,
        format!("[{:.3}] hil-gadget up 启动\n", now_seconds()),
    );
    sync_disk();
}

fn read_log() -> String {
    fs::read_to_string(UP_LOG).unwrap_or_default()
}

fn kernel_gadget_log() -> String {
    Command::new("dmesg")
        .output()
        .ok()
        .map_or_else(String::new, |output| {
            String::from_utf8_lossy(&output.stdout)
                .lines()
                .filter(|line| line.contains("g1") || line.contains("udc"))
                .rev()
                .take(6)
                .collect::<Vec<_>>()
                .join(" | ")
        })
}

fn sync_disk() {
    let _ = Command::new("sync").status();
}

fn now_seconds() -> f64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0.0, |elapsed| elapsed.as_secs_f64())
}
