use serde::Deserialize;

#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum Team {
    #[default]
    Red,
    Blue,
}

impl Team {
    #[must_use]
    pub const fn code(self) -> u32 {
        match self {
            Self::Red => 0,
            Self::Blue => 1,
        }
    }
}

#[derive(Clone, Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Game {
    pub team: Team,
}

/// # Errors
/// 配置字段或队伍值非法。
pub fn load() -> firefly_error::Result<Game> {
    crate::load(crate::paths::config("game.toml"))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn team_values_are_explicit_and_unknown_settings_fail() {
        assert_eq!(
            toml::from_str::<Game>("team='blue'").unwrap().team,
            Team::Blue
        );
        assert_eq!(toml::from_str::<Game>("").unwrap().team, Team::Red);
        assert!(toml::from_str::<Game>("team='green'").is_err());
        assert!(toml::from_str::<Game>("teams='red'").is_err());
    }
}
