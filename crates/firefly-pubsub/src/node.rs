//! 进程级共享 iceoryx2 节点。
//!
//! 对照 iceoryx2 官方示例（`examples/rust/publish_subscribe`）：每个进程创建
//! 主线程共享的 [`IpcNode`]，话题端口由其派生。独立传感器接收线程持有自己的
//! 节点与端口，退出标志与主线程同步；节点不得跨越所属线程的生命周期。
//! 节点以 [`Node::wait`] 驱动（收到 `SIGINT`/`SIGTERM`
//! 返回 Err），退出时所有端口正常 Drop，iceoryx2 释放全部 IPC 资源。
//!
//! ⚠️ 硬杀进程（SIGKILL / pkill -9）会跳过 Drop，在内核里留下孤儿共享内存
//! 对象与幽灵端口注册（后续订阅端会连上死端口的残留连接，收不到任何数据）。
//! 排障时的清理手段见仓库 AGENTS.md。

use iceoryx2::node::Node;
use iceoryx2::prelude::*;
use iceoryx2::service::ipc::Service;

/// 进程共享的 ipc 节点类型。
pub type IpcNode = Node<Service>;

/// 创建进程共享节点。
///
/// # Errors
/// iceoryx2 node 创建失败（IPC 资源不可用等）。
pub fn create_node() -> Result<IpcNode, firefly_error::Error> {
    NodeBuilder::new().create::<Service>().map_err(|e| {
        firefly_error::Error::new(
            firefly_error::ErrorKind::Internal,
            format!("创建 iceoryx2 node 失败: {e:?}"),
        )
    })
}
