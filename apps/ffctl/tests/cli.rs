//! `ffctl` 的 CLI 契约：用法错误、帮助与规格，用 `usage-rs` 自带的测试驱动跑真二进制。
//!
//! 只测不碰 IPC 的面（帮助/用法/规格）——投递路径要真发布消息，放在实测里跑，
//! 免得测试去动一架正在飞的飞机。

/// 顶层只列域与自用命令（不列动词）。
#[test]
fn top_level_lists_domains() {
    let out = usage::test::command!("ffctl", "--help").assert_success();
    let stdout = out.stdout_text();
    assert!(stdout.contains("fc"), "{stdout}");
    assert!(stdout.contains("planner"), "{stdout}");
    assert!(stdout.contains("completion"), "{stdout}");
}

/// 域的下一层是动词，且每个域自带帮助。
#[test]
fn domains_list_verbs() {
    let out = usage::test::command!("ffctl", "fc", "--help").assert_success();
    let stdout = out.stdout_text();
    for verb in ["arm", "disarm", "hold", "land", "takeoff", "track"] {
        assert!(stdout.contains(verb), "缺少 {verb}：{stdout}");
    }

    let out = usage::test::command!("ffctl", "planner", "--help").assert_success();
    assert!(out.stdout_text().contains("goal"), "{}", out.stdout_text());
}

/// 未知子命令：clap 风格错误 + 退出码 2。
#[test]
fn unknown_subcommand_exits_two() {
    let out = usage::test::command!("ffctl", "arm");
    assert_eq!(out.status.code(), Some(2), "{}", out.stderr_text());
    assert!(
        out.stderr_text().contains("unrecognized subcommand"),
        "{}",
        out.stderr_text()
    );
}

/// 参数解析失败（高度不是数字）：退出码 2，错误指向那个位置参数。
#[test]
fn takeoff_rejects_non_numeric_altitude() {
    let out = usage::test::command!("ffctl", "fc", "takeoff", "abc");
    assert_eq!(out.status.code(), Some(2), "{}", out.stderr_text());
    assert!(
        out.stderr_text().contains("[ALTITUDE]"),
        "{}",
        out.stderr_text()
    );
}

/// 规格导出（`usage-rs` 白送）：两个域都在，且带 `effect` 标注。
#[test]
fn usage_spec_exports_domains() {
    let out = usage::test::command!("ffctl", "__usage_spec__").assert_success();
    let spec = out.stdout_text();
    assert!(spec.contains("cmd fc"), "{spec}");
    assert!(spec.contains("cmd planner"), "{spec}");
    assert!(spec.contains("cmd takeoff"), "{spec}");
    assert!(spec.contains("effect=write"), "{spec}");
}
