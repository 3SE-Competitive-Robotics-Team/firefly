use iceoryx2::node::Node;
use iceoryx2::prelude::*;
use iceoryx2::service::ipc::Service;

pub type IpcNode = Node<Service>;

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
