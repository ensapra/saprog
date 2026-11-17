#!/usr/bin/env bash
set -Eeuo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
exec python3 "${workspace_dir}/src/saprog/scripts/home.py"
