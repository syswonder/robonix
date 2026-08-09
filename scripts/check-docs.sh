#!/usr/bin/env bash
# Docs check — code and in-repo docs must be English.
#
# Flags CJK ideographs / CJK & fullwidth punctuation (i.e. non-English prose
# that slipped into source or docs). Detection uses explicit CJK Unicode ranges,
# NOT \p{Han} — \p{Han} false-positives on the Latin middle dot U+00B7 '·' that
# the TUI / web UI use as a separator.
#
# Exempt:
#   - docs/                                       developer manual (separate submodule)
#   - capabilities/lib/{common,rcl}_interfaces/   vendored upstream ROS 2 IDL
#   - **/LICENSE                                  Mulan PSL is legally bilingual
#   - *-zh.* / *_zh.*                             intentional translations
#   - tools/codegen/src/codegen/docs_gen.rs       the Chinese-doc generator
#                                                 (`rbnx docs` emits a Chinese
#                                                 manual; its templates live here)
#   - binary/rendering assets                    png/gif/svg/jpg/pdf/ico/dae/stl/
#                                                tar.gz/fonts
#   - any line containing 'i18n-ok'              documented encoding/i18n test fixture
set -uo pipefail
cd "$(git rev-parse --show-toplevel)"

mapfile -t files < <(git ls-files \
  | grep -vE '^docs/' \
  | grep -vE '^capabilities/lib/(common_interfaces|rcl_interfaces)/' \
  | grep -vE '^tools/codegen/src/codegen/docs_gen\.rs$' \
  | grep -vEi '(^|/)LICENSE$' \
  | grep -vE '(-zh|_zh)\.[A-Za-z0-9]+$' \
  | grep -vEi '\.(png|gif|svg|jpe?g|pdf|ico|dae|stl|woff2?|ttf|eot)$' \
  | grep -vE '\.tar\.gz$')

python3 - "${files[@]}" <<'PY'
import sys
def is_cjk(c):
    o = ord(c)
    return (0x3000 <= o <= 0x303F or   # CJK symbols & punctuation
            0x3400 <= o <= 0x4DBF or   # CJK ext A
            0x4E00 <= o <= 0x9FFF or   # CJK unified ideographs
            0xF900 <= o <= 0xFAFF or   # CJK compatibility ideographs
            0xFF00 <= o <= 0xFFEF)     # fullwidth / halfwidth forms
viol = 0
for path in sys.argv[1:]:
    try:
        with open(path, encoding='utf-8', errors='ignore') as fh:
            for n, line in enumerate(fh, 1):
                if 'i18n-ok' in line:
                    continue
                if any(is_cjk(ch) for ch in line):
                    print(f"::error file={path},line={n}::non-English text found")
                    print(f"  {path}:{n}: {line.rstrip()}")
                    viol += 1
    except (OSError, UnicodeError):
        pass
if viol:
    print()
    print(f"FAIL: {viol} line(s) contain non-English (CJK) text.")
    print("Code and in-repo docs must be English. The developer manual under")
    print("docs/ is exempt; tag a justified encoding/i18n test fixture with 'i18n-ok'.")
    sys.exit(1)
print("OK: code and in-repo docs are English.")
PY
