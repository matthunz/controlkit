use std::{fs, process::Command};

fn main() {
    if fs::read_dir("drake").is_err() {
        Command::new("curl").arg("-fsSLO")
        .arg("https://github.com/RobotLocomotion/drake/releases/download/v1.26.0/drake-1.26.0-mac-arm64.tar.gz").spawn().unwrap().wait().unwrap();

        Command::new("tar")
            .arg("-xvf")
            .arg("drake-1.26.0-mac-arm64.tar.gz")
            .spawn()
            .unwrap()
            .wait()
            .unwrap();
    }

    cxx_build::bridge("src/lib.rs")
        .include("drake/include")
        .include("eigen/include/eigen3")
        .include("fmt/include")
        .file("ffi/drake_ffi.cc")
        .flag("-std=c++20")
        .compile("drake_ffi");

    println!(
        "cargo:rustc-link-search=native={}/drake/lib",
        env!("CARGO_MANIFEST_DIR")
    );
    println!("cargo:rustc-link-lib=drake");
    println!("cargo:rerun-if-changed=src/main.rs");
}
