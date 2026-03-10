#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec python3 "$WS_ROOT/src/lrs_halmstad/lrs_halmstad/run_dataset_make_obb.py" "$@"
