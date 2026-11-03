#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Documentation consistency check for the cubli repository.

Run from anywhere:

    python test/doc_check.py

Hard failures (exit code 1) catch things that are objectively wrong:

  1. mojibake          U+FFFD replacement characters in any document
  2. non-ASCII source  User/**/*.c|h must stay ASCII (Keil's encoding is GBK)
  3. broken tables     a markdown table row split across two lines
  4. table drift       a row whose cell count differs from its header
  5. version drift     FW_BUILD_ID != the "固件 #N" shown in docs/01
  6. missing doc       a mandatory document listed in docs/README.md is absent

Warnings (exit code stays 0) point at text that is probably stale:

  7. stale markers     TBD / 待测 / 待提供 / 未开始 / ⬜ in the current-state
                       documents. Historical entries in 00_开发日志.md and the
                       open list in 18_待确认清单.md are skipped on purpose.

Adapted from dsh_foc's test/doc_check.py (workflow 02 + 05).
"""

import glob
import os
import re
import sys

# Windows consoles default to GBK; documents contain Chinese.
try:
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

DOCS = sorted(glob.glob(os.path.join(ROOT, 'docs', '*.md')))
# workflow/ is the reusable process spec: same formatting rules, but it is NOT a
# project document (no version numbers / TBD policy), so only the generic checks
# (mojibake, table structure) apply to it.
PROCDOCS = sorted(glob.glob(os.path.join(ROOT, 'workflow', '**', '*.md'),
                            recursive=True))
SRC = sorted(glob.glob(os.path.join(ROOT, 'User', '**', '*.c'), recursive=True) +
             glob.glob(os.path.join(ROOT, 'User', '**', '*.h'), recursive=True))
BOARD_CFG = os.path.join(ROOT, 'User', 'Cfg', 'board_cfg.h')
PROGRESS = os.path.join(ROOT, 'docs', '01_项目进度.md')

# documents that describe the *current* state; 00 is history, 18 is an open list
STALE_SCAN_SKIP = ('00_开发日志.md', '18_待确认清单.md')

# mandatory process documents (workflow 02 section 3)
MANDATORY = [
    'docs/00_开发日志.md',
    'docs/01_项目进度.md',
    'docs/10_框架_主框架.md',
    'docs/90_已知问题与限制.md',
]

STALE_PATTERNS = [
    (r'\bTBD\b', 'TBD'),
    (r'待测', '待测'),
    (r'待提供', '待提供'),
    (r'待定', '待定'),
    (r'尚未实施', '尚未实施'),
    (r'一步[：:]\s*S\d', '下一步：S<n>'),
]

errors = []
warnings = []


def err(msg):
    errors.append(msg)


def warn(msg):
    warnings.append(msg)


def read(path):
    with open(path, 'r', encoding='utf-8', errors='replace') as f:
        return f.read()


def rel(path):
    return os.path.relpath(path, ROOT).replace('\\', '/')


# ---------------------------------------------------------------- 1. mojibake
for p in DOCS + PROCDOCS + [os.path.join(ROOT, 'test', 'README.md')]:
    if not os.path.exists(p):
        continue
    for n, line in enumerate(read(p).splitlines(), 1):
        if '\ufffd' in line:
            err('%s:%d mojibake (U+FFFD)' % (rel(p), n))

# ------------------------------------------------------------ 2. ascii source
for p in SRC:
    data = open(p, 'rb').read()
    if any(b > 127 for b in data):
        err('%s contains non-ASCII bytes' % rel(p))

# ---------------------------------------------------------- 3. broken tables
for p in DOCS + PROCDOCS:
    in_fence = False
    for n, line in enumerate(read(p).splitlines(), 1):
        t = line.strip()
        if t.startswith('```'):
            in_fence = not in_fence
            continue
        if in_fence or not t.startswith('|'):
            continue
        if len(t) > 2 and not t.endswith('|'):
            err('%s:%d table row not closed with |' % (rel(p), n))

# ------------------------------------------- 4. table column consistency
CELL_SPLIT = re.compile(r'(?<!\\)\|')


def table_cells(line):
    parts = CELL_SPLIT.split(line)
    # a well formed row starts and ends with '|' -> first/last parts empty
    return len(parts) - 2


for p in DOCS + PROCDOCS:
    in_fence = False
    header = None
    for n, line in enumerate(read(p).splitlines(), 1):
        t = line.strip()
        if t.startswith('```'):
            in_fence = not in_fence
            continue
        if in_fence:
            continue
        if not t.startswith('|'):
            header = None
            continue
        cols = table_cells(t)
        if header is None:
            header = cols
        elif cols != header:
            err('%s:%d table has %d cells, header has %d (unescaped | ?)'
                % (rel(p), n, cols, header))

# ------------------------------------------------------------- 5. build id
build_id = None
if os.path.exists(BOARD_CFG):
    m = re.search(r'#define\s+FW_BUILD_ID\s+(\d+)u', read(BOARD_CFG))
    if m:
        build_id = int(m.group(1))
if build_id is not None and os.path.exists(PROGRESS):
    # Only the highest stage row (the project's current stage) may quote the
    # build number; older rows are history.
    best = None
    for n, line in enumerate(read(PROGRESS).splitlines(), 1):
        m = re.search(r'\|\s*\*{0,2}S(\d+)', line)
        if not m:
            continue
        stage = int(m.group(1))
        if best is None or stage > best[0]:
            best = (stage, n, line)
    if best is not None:
        m = re.search(r'固件\s*#(\d+)', best[2])
        if m and int(m.group(1)) != build_id:
            err('%s:%d S%d row says 固件 #%s, board_cfg.h has #%d'
                % (rel(PROGRESS), best[1], best[0], m.group(1), build_id))

# --------------------------------------------------------- 6. mandatory docs
for doc in MANDATORY:
    if not os.path.exists(os.path.join(ROOT, doc)):
        err('mandatory document missing: %s' % doc)

# --------------------------------------------------------- 7. stale markers
for p in DOCS:
    if os.path.basename(p) in STALE_SCAN_SKIP:
        continue
    in_fence = False
    for n, line in enumerate(read(p).splitlines(), 1):
        t = line.strip()
        if t.startswith('```'):
            in_fence = not in_fence
            continue
        if in_fence:
            continue
        # A heading such as "## 7. 已知限制与 TBD" is a section title, and the
        # progress legend "完成/进行中/未开始" is a legend: neither is a stale
        # statement about the project. Lines that *cite* the open-item list or
        # describe the gate itself are likewise not stale claims.
        if t.startswith('#'):
            continue
        if '图例' in line:
            continue
        if '待确认清单' in line or 'doc_check' in line:
            continue
        for pat, label in STALE_PATTERNS:
            if re.search(pat, line):
                warn('%s:%d [%s] %s' % (rel(p), n, label, t[:70]))
                break

# ----------------------------------------------------------------- report
print('doc_check: %d documents, %d source files, %d workflow docs'
      % (len(DOCS), len(SRC), len(PROCDOCS)))
print('doc_check: build id %s' % build_id)

if warnings:
    print()
    print('--- warnings (stale text, review manually) ---')
    for w in warnings:
        print('  ' + w)

if errors:
    print()
    print('--- errors ---')
    for e in errors:
        print('  ' + e)
    print()
    print('RESULT: FAIL (%d errors, %d warnings)' % (len(errors), len(warnings)))
    sys.exit(1)

print()
print('RESULT: PASS (%d warnings)' % len(warnings))
sys.exit(0)
