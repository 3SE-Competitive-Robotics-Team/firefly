use crate::flight::Receipt;
use std::{
    fs::{self, File},
    io::{self, Write},
    path::{Path, PathBuf},
};

pub struct Store {
    dir: PathBuf,
    pub run: String,
}
impl Store {
    pub fn open() -> io::Result<Self> {
        #[cfg(target_os = "linux")]
        let run = fs::read_to_string("/proc/sys/kernel/random/boot_id")?;
        #[cfg(not(target_os = "linux"))]
        let run = String::from_utf8(
            std::process::Command::new("sysctl")
                .args(["-n", "kern.bootsessionuuid"])
                .output()?
                .stdout,
        )
        .map_err(io::Error::other)?;
        let run = run.trim().to_owned();
        if !firefly_mission::valid_id(&run) {
            return Err(io::Error::other("missing OS boot identity"));
        }
        let dir = firefly_config::paths::root()
            .join("state/missions")
            .join(&run);
        fs::create_dir_all(&dir)?;
        Ok(Self { dir, run })
    }
    pub fn load(&self, id: &str) -> io::Result<Option<Receipt>> {
        match File::open(self.dir.join(format!("{id}.json"))) {
            Ok(file) => Ok(Some(serde_json::from_reader(file)?)),
            Err(e) if e.kind() == io::ErrorKind::NotFound => Ok(None),
            Err(e) => Err(e),
        }
    }
    pub fn save(&self, r: &Receipt) -> io::Result<()> {
        let tmp = self.dir.join(".pending");
        let mut file = File::create(&tmp)?;
        serde_json::to_writer(&mut file, r)?;
        file.flush()?;
        file.sync_all()?;
        fs::rename(tmp, self.dir.join(format!("{}.json", r.id)))?;
        File::open(&self.dir)?.sync_all()
    }
    pub fn recover(&self) -> io::Result<(Option<u64>, u64, Option<Receipt>)> {
        let mut session = None;
        let mut last_flight = 0;
        let mut active = None;
        for entry in fs::read_dir(&self.dir)? {
            let path = entry?.path();
            if path.extension() != Some(std::ffi::OsStr::new("json")) {
                continue;
            }
            let r: Receipt = serde_json::from_reader(File::open(&path)?)?;
            if !firefly_mission::valid_id(&r.id)
                || path.file_stem() != Some(Path::new(&r.id).as_os_str())
                || r.run != self.run
                || session.is_some_and(|s| s != r.session)
            {
                return Err(io::Error::other("inconsistent mission receipt"));
            }
            session = Some(r.session);
            last_flight = last_flight.max(r.flight_number.unwrap_or(0));
            if !r.phase.terminal() && active.replace(r).is_some() {
                return Err(io::Error::other("multiple active missions"));
            }
        }
        Ok((session, last_flight, active))
    }
}
