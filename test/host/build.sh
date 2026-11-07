#!/usr/bin/env bash
# =============================================================================
#  Host unit tests (L2) for the cubli firmware logic modules.
#
#  Compiles the pure-logic part of User/** against a fake HAL on the host GCC
#  and runs the test binary. No board, no Keil, no CubeMX.
#
#  Usage:  bash test/host/build.sh        (from anywhere; WSL or any POSIX gcc)
# =============================================================================
set -u

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"

if ! command -v gcc >/dev/null 2>&1; then
    echo "ERROR: host gcc not found in PATH" >&2
    echo "       hint: run this inside WSL (Ubuntu-24.04) where gcc 13.x is installed" >&2
    exit 1
fi

# Workflow 04: print the environment. A "wrong shell" otherwise shows up as a
# bogus "undefined reference to cosf" instead of the real error.
echo "env  : bash $BASH_VERSION"
echo "env  : gcc  $(command -v gcc)  ($(gcc -dumpversion))"
echo "env  : root $ROOT"

INC="-I$HERE/fake -I$ROOT/User/Cfg -I$ROOT/User/Bsp -I$ROOT/User/Mid -I$ROOT/User/Alg -I$ROOT/User/App"

SRC="$HERE/test_main.c
     $HERE/fake/hal_stub.c
     $ROOT/User/Mid/mid_ringbuf.c
     $ROOT/User/Mid/mid_log.c
     $ROOT/User/Mid/mid_vofa.c
     $ROOT/User/Mid/mid_current.c
     $ROOT/User/Mid/mid_trig.c
     $ROOT/User/Mid/mid_enc.c
     $ROOT/User/Mid/mid_fdilink.c
$ROOT/User/Mid/mid_att.c
     $ROOT/User/Alg/alg_svpwm.c
     $ROOT/User/Alg/alg_vf.c
     $ROOT/User/Alg/alg_pi.c
     $ROOT/User/Alg/alg_foc.c
     $ROOT/User/Alg/alg_spd.c
     $ROOT/User/Alg/alg_pll.c
     $ROOT/User/Alg/alg_obs.c
     $ROOT/User/Alg/alg_if.c
     $ROOT/User/Alg/alg_bal.c
     $ROOT/User/Bsp/bsp_gpio.c
     $ROOT/User/Bsp/bsp_pwm.c
     $ROOT/User/Bsp/bsp_adc.c
     $ROOT/User/Bsp/bsp_prof.c
     $ROOT/User/Bsp/bsp_vbus.c
     $ROOT/User/Bsp/bsp_sys.c
     $HERE/fake/bsp_flash_fake.c
     $HERE/fake/bsp_ssi_fake.c
     $HERE/fake/bsp_cordic_fake.c
$HERE/fake/bsp_crash_fake.c
    $HERE/fake/bsp_stack_fake.c
     $HERE/fake/bsp_wp_fake.c
     $HERE/fake/bsp_img_fake.c
     $ROOT/User/Bsp/bsp_imu.c
     $ROOT/User/Bsp/bsp_uart.c
     $ROOT/User/App/app_main.c
     $ROOT/User/App/app_cmd.c"

OUT="$HERE/run_tests"

echo "== compiling =="
# shellcheck disable=SC2086
gcc -std=gnu99 -O1 -Wall -Wextra -Wno-unused-parameter $INC $SRC -o "$OUT" -lm || exit 1

echo "== running =="
"$OUT"
RC=$?
echo "== exit $RC =="
exit $RC
