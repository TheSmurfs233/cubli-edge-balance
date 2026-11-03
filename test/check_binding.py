#!/usr/bin/env python3
"""Cross-check the constants that User/ assumes about the CubeMX-generated Core/.

Every check here exists because a mismatch compiles cleanly, links cleanly and
then misbehaves only on the board:

  - SPI1_RX moved from DMA1_Channel3 (dsh_foc) to DMA1_Channel5 (this project).
    bsp_ssi.c pokes DMA registers directly, so a stale channel number means it
    polls a TC flag that never gets set -> encoder reads stale bytes, every CRC
    still passes.
  - The debug UART moved from USART1 (CN3) to USART2 (on-board CH340N). A stale
    handle means no logs at all.
  - DBG_UART_BAUD must equal huartX.Init.BaudRate; a mismatch prints garbage.
  - The RX DMA must stay CIRCULAR, otherwise bsp_uart's wrap logic silently
    drops a whole buffer per lap.

Run:  python test/check_binding.py        (exit 0 = PASS, 1 = FAIL)
"""

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FAILS = []
CHECKS = 0


def read(rel):
    path = os.path.join(ROOT, rel)
    if not os.path.exists(path):
        FAILS.append("%s: file not found" % rel)
        return ""
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        return f.read()


def check(name, ok, detail=""):
    global CHECKS
    CHECKS += 1
    print(("  PASS  " if ok else "  FAIL  ") + name + (("  <- " + detail) if (detail and not ok) else ""))
    if not ok:
        FAILS.append(name + ((" (" + detail + ")") if detail else ""))


def grab(pattern, text, default=None):
    m = re.search(pattern, text)
    return m.group(1) if m else default


# ---------------------------------------------------------------- SSI / DMA
spi_c = read("Core/Src/spi.c")
ssi_c = read("User/Bsp/bsp_ssi.c")

dma_from_cubemx = grab(r"hdma_spi1_rx\.Instance\s*=\s*(DMA1_Channel\d+)", spi_c)
dma_in_driver = grab(r"#define\s+SSI_DMA\s+(DMA1_Channel\d+)", ssi_c)
check("spi1_rx: bsp_ssi.c SSI_DMA matches spi.c", dma_from_cubemx == dma_in_driver,
      "cubemx=%s driver=%s" % (dma_from_cubemx, dma_in_driver))

ch = grab(r"DMA1_Channel(\d+)", dma_from_cubemx or "")
if ch:
    check("spi1_rx: IFCR/ISR flag macros use channel %s" % ch,
          ("DMA_IFCR_CGIF%s" % ch) in ssi_c and ("DMA_ISR_TCIF%s" % ch) in ssi_c,
          "expected CGIF%s/TCIF%s" % (ch, ch))

# ------------------------------------------------------------ debug UART
usart_c = read("Core/Src/usart.c")
uart_c = read("User/Bsp/bsp_uart.c")
board_cfg = read("User/Cfg/board_cfg.h")

uart_in_driver = grab(r"#define\s+DBG_UART_NUM\s+(\d)", uart_c)          # absent => USART2 hardcoded
if uart_in_driver is None:
    uart_in_driver = "2" if ("huart2" in uart_c and "huart1" not in uart_c) else "1"
check("uart: bsp_uart.c drives USART2 and never mentions huart1",
      uart_in_driver == "2" and ("huart1" not in uart_c) and ("USART1" not in uart_c),
      "detected USART%s" % uart_in_driver)

baud_hw = grab(r"huart" + uart_in_driver + r"\.Init\.BaudRate\s*=\s*(\d+)", usart_c)
baud_cfg = grab(r"#define\s+DBG_UART_BAUD\s+(\d+)u?", board_cfg)
check("uart: DBG_UART_BAUD == huart%s.Init.BaudRate" % uart_in_driver, baud_hw == baud_cfg,
      "cubemx=%s config=%s" % (baud_hw, baud_cfg))

rx_mode = grab(r"hdma_usart" + uart_in_driver + r"_rx\.Init\.Mode\s*=\s*(DMA_\w+)", usart_c)
check("uart: RX DMA mode is DMA_CIRCULAR", rx_mode == "DMA_CIRCULAR", "cubemx=%s" % rx_mode)

# The 4 DMA handles this project uses must all exist and be distinct.
used = re.findall(r"hdma_(?:usart\d_[rt]x|spi1_rx)\.Instance\s*=\s*(DMA1_Channel\d+)", usart_c + spi_c)
check("dma: 5 handles map to 5 distinct channels", len(set(used)) == 5 and len(used) == 5,
      "found %s" % (used,))

# ---------------------------------------------------------------- encoding
bads = []
for base, _, files in os.walk(os.path.join(ROOT, "User")):
    for fn in files:
        if fn.endswith((".c", ".h")):
            p = os.path.join(base, fn)
            with open(p, "rb") as f:
                if any(b > 127 for b in f.read()):
                    bads.append(os.path.relpath(p, ROOT))
check("source: User/ is pure ASCII (ARMCC 5 parses as GBK)", not bads, ", ".join(bads))

# ----------------------------------------------------------------- summary
print("")
print("checks: %d, failed: %d" % (CHECKS, len(FAILS)))
if FAILS:
    print("RESULT: FAIL")
    sys.exit(1)
print("RESULT: PASS")
sys.exit(0)
