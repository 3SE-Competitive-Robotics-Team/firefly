//! `ffctl` 的 CLI 契约：用法错误、帮助与规格，用 `usage-rs` 自带的测试驱动跑真二进制。
//!
//! 只测不碰 IPC 的面（帮助/用法/规格）——投递路径要真发布消息，放在实测里跑，
//! 免得测试去动一架正在飞的飞机。

/// 顶层只列域与自用命令（不列动词）；域在前，`completion` 排在最后。
#[test]
fn top_level_lists_domains() {
    let out = usage_rs::test::command!("ffctl", "--help").assert_success();
    let stdout = out.stdout_text();
    let at = |row: &str| {
        stdout
            .find(&format!("\n  {row}"))
            .unwrap_or_else(|| panic!("缺少 {row} 行：{stdout}"))
    };
    assert!(at("fc ") < at("planner "), "{stdout}");
    assert!(at("planner ") < at("process "), "{stdout}");
    assert!(at("process ") < at("completion "), "{stdout}");
}

/// 域的下一层是动词，且每个域自带帮助。
#[test]
fn domains_list_verbs() {
    let out = usage_rs::test::command!("ffctl", "fc", "--help").assert_success();
    let stdout = out.stdout_text();
    for verb in ["arm", "disarm", "hold", "land", "takeoff", "track"] {
        assert!(stdout.contains(verb), "缺少 {verb}：{stdout}");
    }

    let out = usage_rs::test::command!("ffctl", "planner", "--help").assert_success();
    assert!(out.stdout_text().contains("goal"), "{}", out.stdout_text());

    let out = usage_rs::test::command!("ffctl", "process", "--help").assert_success();
    let stdout = out.stdout_text();
    for verb in [
        "start",
        "stop",
        "restart",
        "enable",
        "disable",
        "status",
        "supervise",
    ] {
        assert!(stdout.contains(verb), "缺少 {verb}：{stdout}");
    }
}

/// `status` 的实时刷新与间隔参数都出现在帮助里。
#[test]
fn status_exposes_watch_and_interval() {
    let out = usage_rs::test::command!("ffctl", "process", "status", "--help").assert_success();
    let stdout = out.stdout_text();
    assert!(stdout.contains("--watch"), "{stdout}");
    assert!(stdout.contains("--interval"), "{stdout}");
}

/// 未知子命令：clap 风格错误 + 退出码 2。
#[test]
fn unknown_subcommand_exits_two() {
    let out = usage_rs::test::command!("ffctl", "arm");
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
    let out = usage_rs::test::command!("ffctl", "fc", "takeoff", "abc");
    assert_eq!(out.status.code(), Some(2), "{}", out.stderr_text());
    assert!(
        out.stderr_text().contains("[ALTITUDE]"),
        "{}",
        out.stderr_text()
    );
}

/// 规格导出（`usage-rs` 白送）：三个域都在，且带 `effect` 标注。
#[test]
fn usage_spec_exports_domains() {
    let out = usage_rs::test::command!("ffctl", "__usage_spec__").assert_success();
    let spec = out.stdout_text();
    assert!(spec.contains("cmd fc"), "{spec}");
    assert!(spec.contains("cmd planner"), "{spec}");
    assert!(spec.contains("cmd process"), "{spec}");
    assert!(spec.contains("cmd takeoff"), "{spec}");
    assert!(spec.contains("effect=write"), "{spec}");
}

/// shell 的取值来自 `ValueEnum`：帮助里列全可能值，补全候选同源。
#[test]
fn completion_takes_a_value_enum_shell() {
    let out = usage_rs::test::command!("ffctl", "completion", "--help").assert_success();
    let stdout = out.stdout_text();
    for shell in ["bash", "elvish", "fish", "nu", "powershell", "zsh"] {
        assert!(stdout.contains(shell), "缺少 {shell}：{stdout}");
    }
}

/// `validate` 声明的高度范围：负值是用法错误（退出码 2）。
#[test]
fn negative_altitude_is_a_usage_error() {
    let out = usage_rs::test::command!("ffctl", "fc", "takeoff", "-1");
    assert_eq!(out.status.code(), Some(2), "{}", out.stderr_text());
    assert!(out.stderr_text().contains("高度"), "{}", out.stderr_text());
}

/// 坐标/高度必须是有限数值：`inf` 在解析期被拒（退出码 2）。
#[test]
fn non_finite_goal_is_a_usage_error() {
    let out = usage_rs::test::command!("ffctl", "planner", "goal", "inf", "0", "1");
    assert_eq!(out.status.code(), Some(2), "{}", out.stderr_text());
    assert!(out.stderr_text().contains("有限"), "{}", out.stderr_text());
}
