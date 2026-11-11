#!/usr/bin/env bash
# =============================================================================
#  One-shot verification gate (workflow 05).
#
#  Runs every host-side gate and stops reporting at the first failure:
#    1. test/check_binding.py   cross-file consistency (static)
#    2. test/l1/run.sh          offline algorithm test (L1, real gcc)
#    3. test/host/build.sh      firmware logic on a fake HAL (L2)
#    4. test/doc_check.py       documentation consistency (static)
#    5. test/check_build_id.py  FW_BUILD_ID has a row in docs/01 (static)
#
#  The Keil firmware build (0 error / 0 warning) is a Windows-only gate and is
#  NOT part of this script; run it separately:
#    pwsh: Start-Process "D:/installfile/Keil_v5/UV4/UV4.exe" `
#            -ArgumentList '-r','cubli.uvprojx','-j0','-o','build.log' `
#            -Wait -PassThru
#
#  Usage:  bash test/verify.sh
#  Exit:   0 = all gates green, 1 = at least one gate failed
# =============================================================================
set -u

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"

echo "env  : bash $BASH_VERSION"
echo "env  : root $ROOT"
command -v gcc    >/dev/null 2>&1 || { echo "FATAL: gcc not found"; exit 1; }
command -v python3 >/dev/null 2>&1 || { echo "FATAL: python3 not found"; exit 1; }
echo "env  : gcc $(gcc -dumpversion), python $(python3 -V 2>&1 | awk '{print $2}')"

fails=0
step=0

run_step() {
    name="$1"
    shift
    step=$((step + 1))
    printf '\n== [%d] %s ==\n' "$step" "$name"
    if "$@"; then
        eval "RESULT_$step=ok"
        printf '   -> OK\n'
    else
        eval "RESULT_$step=FAILED"
        fails=$((fails + 1))
        printf '   -> FAILED\n'
    fi
}

run_step "cross-file binding"        python3 "$HERE/check_binding.py"
run_step "L1 offline algorithms"     bash   "$HERE/l1/run.sh"
run_step "L2 firmware on fake HAL"   bash   "$HERE/host/build.sh"
run_step "documentation consistency" python3 "$HERE/doc_check.py"
run_step "firmware build id"         python3 "$HERE/check_build_id.py"

printf '\n============================\n'
for i in $(seq 1 "$step"); do
    eval "r=\$RESULT_$i"
    printf 'gate %d: %s\n' "$i" "$r"
done

if [ "$fails" -ne 0 ]; then
    printf 'RESULT: FAIL (%d gate(s) failed)\n' "$fails"
    exit 1
fi

printf 'RESULT: PASS (all %d gates)\n' "$step"
echo "note : the Keil firmware build must be verified separately on Windows"
exit 0
