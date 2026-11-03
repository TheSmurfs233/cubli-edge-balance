#!/usr/bin/env bash
# L1 offline test: compile the firmware parser source (unchanged, no HAL) on the PC
# and check it against synthetic FDILink frames.
#
# Run from anywhere:  bash test/l1/run.sh
# Needs: bash + gcc + libm. Runs in WSL (Ubuntu-24.04) or any POSIX shell.
set -u

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
OUT="$HERE/.build"

# Workflow 04: the script must prove its own environment instead of assuming it,
# otherwise a wrong toolchain (msys2 bash vs WSL bash) hides the real error.
echo "== env =="
echo " BASH_VERSION = $BASH_VERSION"
echo " gcc          = $(command -v gcc || echo NOT-FOUND)"
command -v gcc >/dev/null 2>&1 || { echo "FATAL: gcc not found"; exit 2; }

mkdir -p "$OUT" || exit 2

SRC="$ROOT/User/Mid/mid_fdilink.c"
INC="$ROOT/User/Mid"
[ -f "$SRC" ] || { echo "FATAL: $SRC not found"; exit 2; }

echo "== build =="
gcc -std=c99 -Wall -Wextra -Wno-unused-parameter -O2 \
    -I"$INC" -o "$OUT/fdilink_test" \
    "$HERE/fdilink_test.c" "$SRC" -lm || exit 2

echo "== run =="
"$OUT/fdilink_test"
rc=$?
echo "== exit $rc =="
exit $rc
