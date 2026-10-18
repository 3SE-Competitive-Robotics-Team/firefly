//! 进程清单：TOML 读取、校验和原子回写。
//!
//! 清单是唯一配置源；`enable`/`disable` 直接改同文件的 `autostart`，不再维护第二份状态。
//! 回写会重新排版 TOML（注释不保留），所以这个文件不放注释。

use std::fs::{self, File, OpenOptions};
use std::io::Write;
use std::path::Path;

use serde::{Deserialize, Serialize};

pub(crate) const DEFAULT_PATH: &str = "configs/processes.toml";

#[derive(Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct Config {
    #[serde(rename = "process", default)]
    pub(crate) processes: Vec<Process>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct Process {
    pub(crate) name: String,
    pub(crate) command: Vec<String>,
    pub(crate) autostart: bool,
}

impl Config {
    pub(crate) fn load(path: &Path) -> Result<Self, String> {
        let raw = fs::read_to_string(path)
            .map_err(|error| format!("读取 {}: {error}", path.display()))?;
        let config: Self =
            toml::from_str(&raw).map_err(|error| format!("解析 {}: {error}", path.display()))?;
        config.validate()?;
        Ok(config)
    }

    pub(crate) fn validate(&self) -> Result<(), String> {
        if self.processes.is_empty() {
            return Err("进程清单为空".to_owned());
        }
        let mut names = std::collections::HashSet::new();
        for process in &self.processes {
            let valid_name = !process.name.is_empty()
                && process
                    .name
                    .chars()
                    .all(|character| character.is_ascii_alphanumeric() || "-_".contains(character));
            if !valid_name {
                return Err(format!("非法进程名：{:?}", process.name));
            }
            if !names.insert(process.name.as_str()) {
                return Err(format!("进程名重复：{}", process.name));
            }
            if process.command.is_empty() || process.command[0].is_empty() {
                return Err(format!("{} 的 command 为空", process.name));
            }
        }
        Ok(())
    }

    pub(crate) fn save(&self, path: &Path) -> Result<(), String> {
        let text =
            toml::to_string_pretty(self).map_err(|error| format!("序列化进程清单: {error}"))?;
        atomic_write(path, text.as_bytes())
    }

    pub(crate) fn set_autostart(
        &mut self,
        names: &[String],
        autostart: bool,
    ) -> Result<Vec<String>, String> {
        if names.is_empty() {
            return Err("至少给一个进程名".to_owned());
        }
        let missing: Vec<&str> = names
            .iter()
            .filter(|name| self.processes.iter().all(|process| process.name != **name))
            .map(String::as_str)
            .collect();
        if !missing.is_empty() {
            return Err(format!("清单里没有：{}", missing.join(", ")));
        }

        let mut changed = Vec::new();
        for process in &mut self.processes {
            if names.contains(&process.name) {
                process.autostart = autostart;
                changed.push(process.name.clone());
            }
        }
        Ok(changed)
    }
}

fn atomic_write(path: &Path, bytes: &[u8]) -> Result<(), String> {
    let parent = path
        .parent()
        .ok_or_else(|| format!("{} 没有父目录", path.display()))?;
    let file_name = path
        .file_name()
        .ok_or_else(|| format!("{} 没有文件名", path.display()))?;
    let temp = parent.join(format!(".{}.tmp", file_name.to_string_lossy()));

    let mut file = OpenOptions::new()
        .write(true)
        .create(true)
        .truncate(true)
        .open(&temp)
        .map_err(|error| format!("写 {}: {error}", temp.display()))?;
    file.write_all(bytes)
        .map_err(|error| format!("写 {}: {error}", temp.display()))?;
    file.sync_all()
        .map_err(|error| format!("fsync {}: {error}", temp.display()))?;
    drop(file);

    fs::rename(&temp, path).map_err(|error| format!("替换 {}: {error}", path.display()))?;
    sync_dir(parent)
}

fn sync_dir(dir: &Path) -> Result<(), String> {
    match File::open(dir).and_then(|directory| directory.sync_all()) {
        Ok(()) => Ok(()),
        // macOS 对目录 fsync 返回 EINVAL；目标板（Linux）会真正落盘。
        Err(error) if error.raw_os_error() == Some(22) => Ok(()),
        Err(error) => Err(format!("fsync {}: {error}", dir.display())),
    }
}

/// 读取指定名字的 `autostart`；名字不存在时返回 `None`。
#[cfg(test)]
pub(crate) fn autostart_of<'a>(config: &'a Config, name: &str) -> Option<&'a Process> {
    config.processes.iter().find(|process| process.name == name)
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::path::PathBuf;
    const SAMPLE: &str = r#"
[[process]]
name = "vio"
command = ["bin/vio"]
autostart = true

[[process]]
name = "fc"
command = ["bin/fc"]
autostart = false
"#;

    fn scratch(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("ffctl-process-{}-{name}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn parses_and_rewrites_one_file() {
        let dir = scratch("rewrite");
        let path = dir.join("processes.toml");
        fs::write(&path, SAMPLE).unwrap();

        let mut config = Config::load(&path).unwrap();
        assert_eq!(config.processes.len(), 2);
        assert!(!config.processes[1].autostart);

        let changed = config.set_autostart(&["fc".to_owned()], true).unwrap();
        assert_eq!(changed, ["fc"]);
        config.save(&path).unwrap();

        let reloaded = Config::load(&path).unwrap();
        assert_eq!(reloaded.processes.len(), 2);
        assert!(reloaded.processes.iter().all(|process| process.autostart));
        assert_eq!(reloaded.processes[0].name, "vio");
        assert_eq!(reloaded.processes[1].name, "fc");
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn rejects_duplicate_names_and_unknown_fields() {
        let dir = scratch("invalid");
        let path = dir.join("processes.toml");

        fs::write(
            &path,
            r#"
[[process]]
name = "vio"
command = ["bin/vio"]
autostart = true

[[process]]
name = "vio"
command = ["bin/fc"]
autostart = true
"#,
        )
        .unwrap();
        assert!(Config::load(&path).is_err());

        fs::write(
            &path,
            r#"
[[process]]
name = "vio"
command = ["bin/vio"]
autostart = true
extra = 1
"#,
        )
        .unwrap();
        assert!(Config::load(&path).is_err());
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn rejects_unknown_process_name_without_writing() {
        let dir = scratch("missing");
        let path = dir.join("processes.toml");
        fs::write(&path, SAMPLE).unwrap();
        let mut config = Config::load(&path).unwrap();
        assert!(config.set_autostart(&["nope".to_owned()], true).is_err());
        assert!(!config.processes[1].autostart);
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn autostart_lookup() {
        let config: Config = toml::from_str(SAMPLE).unwrap();
        assert!(autostart_of(&config, "vio").is_some());
        assert!(autostart_of(&config, "nope").is_none());
    }
}
