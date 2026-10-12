use std::process::ExitCode;

use usage_rs::{Args, Run, ValueEnum};

use super::Ffctl;

/// 打印某个 shell 的补全脚本（自用命令，不属于任何域）
///
/// 脚本在补全时回调本二进制（`parse()` 拦截隐藏请求），因此改子命令即改补全，无需重发脚本。
#[derive(Args)]
#[usage(effect = "read")]
pub(crate) struct Completion {
    /// 目标 shell
    #[usage(value_enum)]
    shell: ShellKind,
}

impl Run for Completion {
    type Output = ExitCode;

    fn run(self) -> Self::Output {
        print!("{}", Ffctl::completion_script(self.shell.into()));
        ExitCode::SUCCESS
    }
}

/// 补全脚本的目标 shell（与 `usage-rs` 的 `Shell` 一一对应）
#[derive(ValueEnum)]
enum ShellKind {
    Bash,
    Elvish,
    Fish,
    Nu,
    #[usage(name = "powershell")]
    PowerShell,
    Zsh,
}

impl From<ShellKind> for usage_rs::complete::Shell {
    fn from(shell: ShellKind) -> Self {
        match shell {
            ShellKind::Bash => Self::Bash,
            ShellKind::Elvish => Self::Elvish,
            ShellKind::Fish => Self::Fish,
            ShellKind::Nu => Self::Nu,
            ShellKind::PowerShell => Self::PowerShell,
            ShellKind::Zsh => Self::Zsh,
        }
    }
}
