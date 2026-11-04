#!/usr/bin/env bash
# L1 offline tests: compile the real firmware sources (unchanged, no HAL) on the PC
# and check them against synthetic inputs.
#
#   fdilink_test : User/Mid/mid_fdilink.c  frame parser, 15 cases
#   att_test     : User/Mid/mid_att.c      attitude layer, 12 cases
#   bal_test     : User/Alg/alg_bal.c      balance law, 14 cases
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

INC="$ROOT/User/Mid"
INC_ALG="$ROOT/User/Alg"
for f in "$ROOT/User/Mid/mid_fdilink.c" "$ROOT/User/Mid/mid_att.c" \
         "$ROOT/User/Alg/alg_bal.c"; do
    [ -f "$f" ] || { echo "FATAL: $f not found"; exit 2; }
done

fail=0

echo "== build fdilink_test =="
gcc -std=c99 -Wall -Wextra -Wno-unused-parameter -O2 \
    -I"$INC" -o "$OUT/fdilink_test" \
    "$HERE/fdilink_test.c" "$ROOT/User/Mid/mid_fdilink.c" -lm || exit 2

echo "== run fdilink_test =="
"$OUT/fdilink_test" || fail=1

echo "== build att_test =="
gcc -std=c99 -Wall -Wextra -Wno-unused-parameter -O2 \
    -I"$INC" -o "$OUT/att_test" \
    "$HERE/att_test.c" "$ROOT/User/Mid/mid_att.c" -lm || exit 2

echo "== run att_test =="
"$OUT/att_test" || fail=1

echo "== build bal_test =="
gcc -std=c99 -Wall -Wextra -Wno-unused-parameter -O2 \
    -I"$INC_ALG" -o "$OUT/bal_test" \
    "$HERE/bal_test.c" "$ROOT/User/Alg/alg_bal.c" -lm || exit 2

echo "== run bal_test =="
"$OUT/bal_test" || fail=1

echo "== exit $fail =="
exit $fail