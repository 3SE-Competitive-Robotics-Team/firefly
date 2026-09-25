//! `apps/render` 与 `apps/quad` 共用的 Bevy 可视化基建。
//!
//! 两个应用都是「在场地里显示一架机体 + 第三人称相机」的 Bevy 程序，场景资产根、
//! 光照、机体可视化与追踪相机同源，抽在这里，避免各写一份。

pub mod camera;
pub mod drone;
pub mod lighting;
pub mod scene;
