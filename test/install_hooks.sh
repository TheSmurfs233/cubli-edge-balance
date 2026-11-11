#!/bin/sh
# Install the repository's commit hooks into .git/hooks.
#
# Hooks are not versioned by git itself, so the authoritative copies live in
# test/hooks/ and this script links them into place. Run it once per clone:
#
#   bash test/install_hooks.sh

set -e

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
DEST="$ROOT/.git/hooks"

if [ ! -d "$DEST" ]; then
    echo "not a git repository: $ROOT" >&2
    exit 1
fi

# LF endings are mandatory: a CRLF shebang breaks sh on Windows.
for f in "$HERE"/hooks/*; do
    name=$(basename "$f")
    case "$name" in
        *.md) continue ;;
    esac
    sed 's/\r$//' "$f" > "$DEST/$name"
    chmod +x "$DEST/$name"
    echo "installed: .git/hooks/$name"
done

echo "done"
