#!/usr/bin/env python3
"""Gate 6: the "code must arrive with its documents" pre-commit hook is live.

Why this exists: AGENTS.md section 4 ("改代码必须同步对应文档") used to be a rule
that only lived in prose. On 2026-11-11 two commits (7aad7ce, d221ab9) changed
BAL_* constants and left the documentation to a later commit, so for a while the
repo described firmware that did not exist. The enforcement is now a real
pre-commit hook (``test/hooks/pre-commit``), but git does not version hooks, so a
fresh clone silently has none. This gate refuses to pass until the hook is
installed, and warns when the installed copy differs from the tracked one.

Checks:
  1. ``<root>/test/hooks/pre-commit`` exists (it is tracked, so it must)
  2. ``.git/hooks/pre-commit`` exists and is executable
  3. its content equals the tracked copy (installed copy is not stale)

Usage:  python3 test/check_hooks.py
Exit 0 = pass, 1 = fail.
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "test" / "hooks" / "pre-commit"
DST = ROOT / ".git" / "hooks" / "pre-commit"
INSTALLER = "bash test/install_hooks.sh"


def fail(msg):
    print("FAIL: " + msg)
    return 1


def norm(b):
    # the installer strips CRLF; compare line endings the same way
    return b.replace(b"\r\n", b"\n")


def main():
    if not SRC.is_file():
        return fail("missing tracked hook " + str(SRC))
    if not DST.is_file():
        return fail(".git/hooks/pre-commit is not installed; run: " + INSTALLER)

    want = norm(SRC.read_bytes())
    have = norm(DST.read_bytes())
    if want != have:
        return fail(".git/hooks/pre-commit is stale (differs from %s); run: %s"
                    % (SRC, INSTALLER))

    print("hook : .git/hooks/pre-commit installed, matches test/hooks/pre-commit")
    print("RESULT: PASS (a User/ or Core/ change must come with a docs/ change)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
