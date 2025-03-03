use postcard_rpc::header::VarHeader;

use crate::app::{AppTx, Context, TaskContext};

/// This is an example of a BLOCKING handler.
pub fn unique_id(context: &mut Context, _header: VarHeader, _arg: ()) -> u64 {
    context.unique_id
}

pub fn meow(context: &mut Context, _header: VarHeader, _arg: ()) -> &'static str {
    "meow!"
}
