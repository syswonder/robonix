#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Print the input for a code review.
#   review_input.sh diff <base>        commits on HEAD since merge-base(<base>, HEAD)
#   review_input.sh files <path>...    current contents of the files, line-numbered
set -euo pipefail

mode="${1:-}"
shift || true
case "$mode" in
  diff)
    base="${1:?usage: review_input.sh diff <base>}"
    from="$(git merge-base "$base" HEAD)"
    echo "# review: diff $base ($(git rev-parse --short "$from")..$(git rev-parse --short HEAD))"
    git log --reverse --format='%n## commit %h%n%n%B' --patch --stat "$from..HEAD"
    ;;
  files)
    [ "$#" -gt 0 ] || { echo "usage: review_input.sh files <path>..." >&2; exit 2; }
    for path in "$@"; do
      echo "## file $path"
      nl -ba "$path"
    done
    ;;
  *)
    echo "usage: review_input.sh diff <base> | files <path>..." >&2
    exit 2
    ;;
esac
