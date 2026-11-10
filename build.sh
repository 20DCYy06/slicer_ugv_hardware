#!/usr/bin/env bash
set -euo pipefail
HARDWARE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
VERSION="${1:-v1.0}"
[[ "$VERSION" =~ ^v[0-9]+\.[0-9]+(\.[0-9]+)?$ ]] || { echo 'Invalid version'; exit 2; }
export PYTHONDONTWRITEBYTECODE=1
export TMPDIR="$HARDWARE/.tmp"
mkdir -p "$TMPDIR"
python3 -s "$HARDWARE/Firmware/$VERSION/src/build.py"
python3 -s "$HARDWARE/archive_build.py" "$VERSION"
