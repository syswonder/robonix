// SPDX-License-Identifier: MulanPSL-2.0

use std::process::Command;

/// Run clock fault injection in an isolated test process without changing host time.
pub fn run_case(name: &str, exercise: fn()) -> Option<std::process::Output> {
    if std::env::var("ROBONIX_CLOCK_TEST_CHILD").as_deref() == Ok(name) {
        exercise();
        return None;
    }
    let directory = tempfile::tempdir().unwrap();
    let library = directory.path().join("clock_failure.so");
    let source =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/support/clock_failure.c");
    let output = Command::new("cc")
        .args([
            "-shared", "-fPIC", "-std=c11", "-Wall", "-Wextra", "-Werror",
        ])
        .arg(source)
        .arg("-ldl")
        .arg("-o")
        .arg(&library)
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let output = Command::new(std::env::current_exe().unwrap())
        .args(["--exact", name, "--nocapture"])
        .env("LD_PRELOAD", library)
        .env("ROBONIX_CLOCK_TEST_CHILD", name)
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}\n{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    Some(output)
}

/// Select a fault using the loaded fixture's atomic mode switch.
pub fn mode(value: i32) {
    unsafe extern "C" {
        fn dlsym(
            handle: *mut std::ffi::c_void,
            name: *const std::ffi::c_char,
        ) -> *mut std::ffi::c_void;
    }
    // SAFETY: run_case loads the fixture in the child; the symbol has this C ABI.
    unsafe {
        let symbol = dlsym(std::ptr::null_mut(), c"robonix_test_clock_mode".as_ptr());
        assert!(!symbol.is_null());
        let set_mode: unsafe extern "C" fn(i32) = std::mem::transmute(symbol);
        set_mode(value);
    }
}
