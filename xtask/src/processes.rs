use std::{ffi::OsString, io, path::Path};
use sysinfo::{ProcessRefreshKind, RefreshKind, System, UpdateKind};

fn simulation_command(args: &[OsString]) -> bool {
    let entry = args.iter().take(2).any(|arg| {
        matches!(
            Path::new(arg).file_name().and_then(|s| s.to_str()),
            Some("firefly-sim" | "mimir" | "rerun" | "validate_landing.py")
        )
    });
    match (
        args.get(1).and_then(|s| s.to_str()),
        args.get(2).and_then(|s| s.to_str()),
    ) {
        (Some("-m"), Some(module)) => matches!(module, "firefly_sim" | "firefly_sim.main"),
        (Some("-c"), Some(code)) => entry || code.contains("firefly_sim"),
        _ => entry,
    }
}

/// Return live Firefly process IDs owned by this checkout.
///
/// # Errors
/// Returns an error if the checkout or the host process table cannot be read.
pub fn active(root: &Path) -> io::Result<Vec<u32>> {
    let root = root.canonicalize()?;
    let target = root.join("target");
    let fields = ProcessRefreshKind::nothing()
        .without_tasks()
        .with_exe(UpdateKind::Always)
        .with_cwd(UpdateKind::Always)
        .with_cmd(UpdateKind::Always);
    let system = System::new_with_specifics(RefreshKind::nothing().with_processes(fields))
        .map_err(io::Error::other)?;
    if system.processes().is_empty() {
        return Err(io::Error::other("could not enumerate host processes"));
    }
    let mut ids = Vec::new();
    for (pid, process) in system.processes() {
        let Some(exe) = process.exe() else { continue };
        let native = exe.starts_with(&target)
            && matches!(
                exe.file_name().and_then(|s| s.to_str()),
                Some(
                    "firefly-daemon"
                        | "firefly-rootslam"
                        | "render"
                        | "mimir"
                        | "fc"
                        | "hil"
                        | "localization"
                        | "planner"
                        | "traj"
                        | "landing"
                        | "depth"
                )
            );
        if native
            || (process.cwd().is_some_and(|cwd| cwd.starts_with(&root))
                && simulation_command(process.cmd()))
        {
            ids.push(pid.as_u32());
        }
    }
    ids.sort_unstable();
    Ok(ids)
}
