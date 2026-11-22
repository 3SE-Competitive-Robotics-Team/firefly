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

#[test]
fn domains_list_verbs() {
    let check = |domain: &str, verbs: &[&str]| {
        let out = usage_rs::test::command!("ffctl", domain, "--help").assert_success();
        let stdout = out.stdout_text();
        for verb in verbs {
            assert!(
                stdout
                    .lines()
                    .any(|line| line.split_whitespace().next() == Some(verb)),
                "缺少 {domain} {verb}：{stdout}"
            );
        }
    };
    check("fc", &["arm", "disarm", "hold", "land", "takeoff", "track"]);
    check("planner", &["goal"]);
    check("mission", &["submit", "status", "cancel"]);
    check(
        "process",
        &[
            "start",
            "stop",
            "restart",
            "enable",
            "disable",
            "status",
            "supervise",
        ],
    );
}

#[test]
fn status_exposes_watch_and_interval() {
    let out = usage_rs::test::command!("ffctl", "process", "status", "--help").assert_success();
    let stdout = out.stdout_text();
    assert!(stdout.contains("--watch"), "{stdout}");
    assert!(stdout.contains("--interval"), "{stdout}");
    assert!(stdout.contains("--json"), "{stdout}");
}

#[test]
fn json_status_rejects_watch_before_connecting() {
    let out = usage_rs::test::command!("ffctl", "process", "status", "--json", "--watch");
    assert_eq!(out.status.code(), Some(2), "{}", out.stderr_text());
    assert!(
        out.stderr_text().contains("--json"),
        "{}",
        out.stderr_text()
    );
    assert!(
        out.stderr_text().contains("--watch"),
        "{}",
        out.stderr_text()
    );
}

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

#[test]
fn usage_spec_exports_domains() {
    let out = usage_rs::test::command!("ffctl", "__usage_spec__").assert_success();
    let spec = out.stdout_text();
    for domain in ["fc", "planner", "process"] {
        let prefix = format!("cmd {domain} ");
        assert!(spec.lines().any(|line| line.starts_with(&prefix)), "{spec}");
    }
    let takeoff = spec
        .lines()
        .find(|line| line.trim_start().starts_with("cmd takeoff "))
        .expect("missing takeoff declaration");
    assert!(
        takeoff
            .split_whitespace()
            .any(|field| field == "effect=write"),
        "{takeoff}"
    );
}

#[test]
fn negative_altitude_is_a_usage_error() {
    let out = usage_rs::test::command!("ffctl", "fc", "takeoff", "-1");
    assert_eq!(out.status.code(), Some(2), "{}", out.stderr_text());
    assert!(out.stderr_text().contains("高度"), "{}", out.stderr_text());
}

#[test]
fn non_finite_goal_is_a_usage_error() {
    let out = usage_rs::test::command!("ffctl", "planner", "goal", "inf", "0", "1");
    assert_eq!(out.status.code(), Some(2), "{}", out.stderr_text());
    assert!(out.stderr_text().contains("有限"), "{}", out.stderr_text());
}
