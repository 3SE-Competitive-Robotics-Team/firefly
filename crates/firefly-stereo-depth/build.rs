fn main() {
    println!("cargo:rerun-if-changed=native/fusion.cpp");
    println!("cargo:rerun-if-changed=native/smoothing.hpp");
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("linux")
        && std::env::var("CARGO_CFG_TARGET_ARCH").as_deref() == Ok("aarch64")
    {
        cc::Build::new()
            .cpp(true)
            .std("c++17")
            .include("/usr/include/opencv4")
            .file("native/fusion.cpp")
            .compile("depth_fusion");
        for library in ["opencv_video", "opencv_imgproc", "opencv_core"] {
            println!("cargo:rustc-link-lib={library}");
        }
    }
}
