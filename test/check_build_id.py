#!/usr/bin/env python3
"""Gate 5: FW_BUILD_ID has a row in the docs/01 build-id table.

Why this exists: the console banner prints ``build = __DATE__ __TIME__ #<id>``.
__DATE__/__TIME__ are frozen by incremental Keil builds, so the id is the only
honest answer to "which firmware is on the board". The id lives in
``User/Cfg/board_cfg.h`` and must be bumped on every firmware change; the
mapping id -> commit is kept in ``docs/01`` section 2.6. Without this gate the
id silently rots at its first value (it did: stayed 1 for two weeks).

Checks:
  1. exactly one ``#define FW_BUILD_ID <n>u`` in board_cfg.h
  2. docs/01 has a ``| <n> |`` row in the "FW_BUILD_ID" table
  3. no row in that table has an id greater than the current one
     (means the source was reverted without fixing the table)

Exit 0 = pass, 1 = fail.
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CFG = ROOT / "User" / "Cfg" / "board_cfg.h"
DOC = ROOT / "docs" / "01_项目进度.md"

SECTION = "2.6"


def fail(msg):
    print("FAIL: " + msg)
    return 1


def main():
    if not CFG.is_file():
        return fail("missing " + str(CFG))
    if not DOC.is_file():
        return fail("missing " + str(DOC))

    cfg = CFG.read_text(encoding="utf-8", errors="replace")
    hits = re.findall(r"^#define\s+FW_BUILD_ID\s+(\d+)u\b", cfg, re.M)
    if len(hits) != 1:
        return fail("board_cfg.h must define FW_BUILD_ID exactly once (found %d)"
                    % len(hits))
    cur = int(hits[0])

    doc = DOC.read_text(encoding="utf-8", errors="replace")
    lines = doc.splitlines()

    # locate the build-id table: the heading that names the section and the
    # macro, then the rows that follow until the next heading.
    start = None
    for i, line in enumerate(lines):
        if line.startswith("#") and SECTION in line and "FW_BUILD_ID" in line:
            start = i
            break
    if start is None:
        return fail("docs/01 has no '%s ... FW_BUILD_ID' section (the mapping "
                    "table lives there)" % SECTION)

    ids = []
    for line in lines[start:]:
        if line.startswith("#") and "FW_BUILD_ID" not in line:
            break
        m = re.match(r"^\|\s*(\d+)\s*\|", line)
        if m:
            ids.append(int(m.group(1)))

    print("fw   : FW_BUILD_ID = %d" % cur)
    print("docs : table rows   = %s" % (ids if ids else "(none)"))

    if not ids:
        return fail("docs/01 section %s has no '| <id> |' rows" % SECTION)
    if cur not in ids:
        return fail("FW_BUILD_ID %d has no row in docs/01 section %s; add "
                    "'| %d | date | commit | Code=... | note |'"
                    % (cur, SECTION, cur))
    if max(ids) > cur:
        return fail("docs/01 lists id %d but the source says %d (source "
                    "reverted without fixing the table?)" % (max(ids), cur))

    print("RESULT: PASS (FW_BUILD_ID %d is recorded in docs/01 %s)"
          % (cur, SECTION))
    return 0


if __name__ == "__main__":
    sys.exit(main())
