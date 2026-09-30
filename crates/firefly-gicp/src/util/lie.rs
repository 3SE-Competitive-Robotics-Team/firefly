//! SE(3) 运算使用基础层统一实现；扭量顺序为 `[rotation, translation]`。

pub use firefly_base::se3::{se3_exp, se3_log, skew, so3_exp, so3_log};
