// SPDX-License-Identifier: MulanPSL-2.0

/// Charecters used to encode capability aliases in base36
///
/// 0-9 -> "0"-"9", 10-35 -> "a"-"z"
const BASE36_DIGITS: &[u8; 36] = b"0123456789abcdefghijklmnopqrstuvwxyz";

pub fn alias_for_index(mut index: usize) -> String {
    let mut suffix = Vec::new();

    loop {
        let digit = index % 36;
        suffix.push(BASE36_DIGITS[digit] as char);
        index /= 36;
        if index == 0 {
            break;
        }
    }
    suffix.reverse();
    let suffix: String = suffix.into_iter().collect();
    format!("c{suffix}")
}

#[cfg(test)]
mod tests {
    use std::assert_ne;

    use super::alias_for_index;
    #[test]
    fn aliases_use_base36() {
        assert_eq!(alias_for_index(0), "c0");
        assert_eq!(alias_for_index(1), "c1");
        assert_eq!(alias_for_index(9), "c9");
        assert_eq!(alias_for_index(10), "ca");
        assert_eq!(alias_for_index(35), "cz");
        assert_eq!(alias_for_index(36), "c10");
        assert_eq!(alias_for_index(37), "c11");
    }
    #[test]
    fn one2one_mapping() {
        let aliases: Vec<String> = (0..100).map(alias_for_index).collect();
        for i in 0..aliases.len() {
            for j in (i + 1)..aliases.len() {
                assert_ne!(aliases[i], aliases[j]);
            }
        }
    }
}
