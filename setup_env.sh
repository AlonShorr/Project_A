#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_IDF_PATH="$ROOT_DIR/tools/esp-idf"

usage() {
  cat <<'EOF'
Usage:
  source ./setup_env.sh
  source ./setup_env.sh --install

Behavior:
  - prefers an existing $IDF_PATH if already exported
  - otherwise falls back to a repo-local ESP-IDF checkout at tools/esp-idf
  - with --install, runs install.sh esp32 before exporting the environment
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
    return 0
  fi
  exit 0
fi

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "ERROR: setup_env.sh must be sourced so it can modify your shell environment." >&2
  echo >&2
  usage >&2
  exit 1
fi

if [[ -n "${IDF_PATH:-}" && -f "${IDF_PATH}/export.sh" ]]; then
  export IDF_PATH
elif [[ -f "$REPO_IDF_PATH/export.sh" ]]; then
  export IDF_PATH="$REPO_IDF_PATH"
else
  cat >&2 <<EOF
ERROR: Could not locate ESP-IDF.

Expected one of:
  - an existing \$IDF_PATH with export.sh
  - $REPO_IDF_PATH

Recommended setup:
  source /path/to/your/esp-idf/export.sh
Optional repo-local setup:
  git clone --branch v5.1.6 --recursive https://github.com/espressif/esp-idf.git tools/esp-idf
  source ./setup_env.sh --install
EOF
  return 1
fi

if [[ "${1:-}" == "--install" ]]; then
  "$IDF_PATH/install.sh" esp32
fi

# shellcheck disable=SC1091
source "$IDF_PATH/export.sh" >/dev/null
echo "ESP-IDF ready: $IDF_PATH"
