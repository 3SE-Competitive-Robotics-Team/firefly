use fastrace::prelude::*;
use firefly_observability::init as init_observability;

#[fastrace::trace]
fn inner() {
    std::thread::sleep(std::time::Duration::from_millis(10));
}

#[fastrace::trace]
fn outer() {
    inner();
    std::thread::sleep(std::time::Duration::from_millis(5));
}

fn main() {
    init_observability();
    for _ in 0..3 {
        let root = Span::root("trace-check", SpanContext::random());
        let _guard = root.set_local_parent();
        outer();
    }
    firefly_observability::flush();
    println!("done");
}
