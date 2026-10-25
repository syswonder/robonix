// SPDX-License-Identifier: MulanPSL-2.0

use std::sync::OnceLock;

// Each caller keeps its own origin because published timestamps are elapsed values.
pub(crate) fn monotonic_ns(origin: &OnceLock<u64>) -> std::io::Result<u64> {
    let start = match origin.get() {
        Some(start) => *start,
        None => {
            let now = robonix_chronos_clock::steady_time()?.timestamp_ns;
            *origin.get_or_init(|| now)
        }
    };
    let now = robonix_chronos_clock::steady_time()?.timestamp_ns;
    now.checked_sub(start)
        .ok_or_else(|| std::io::Error::other("Chronos steady time regressed"))
}

#[cfg(test)]
pub(crate) mod test_support {
    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../chronos/clock/tests/support/mod.rs"
    ));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn origins_remain_independent_and_reference_failures_do_not_affect_elapsed_time() {
        test_support::run_case(
            "clock::tests::origins_remain_independent_and_reference_failures_do_not_affect_elapsed_time",
            || {
                let first = OnceLock::new();
                let second = OnceLock::new();
                monotonic_ns(&first).unwrap();
                std::thread::sleep(std::time::Duration::from_millis(10));
                let elapsed = monotonic_ns(&first).unwrap();
                assert!(elapsed >= 10_000_000);
                monotonic_ns(&second).unwrap();
                assert!(second.get().unwrap() > first.get().unwrap());
                test_support::mode(1);
                assert!(monotonic_ns(&first).unwrap() >= elapsed);
                test_support::mode(16);
                assert!(monotonic_ns(&first).is_err());
                assert!(monotonic_ns(&first).unwrap() >= elapsed);
            },
        );
    }
}
