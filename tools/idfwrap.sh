#!/usr/bin/env bash
set -euo pipefail

ACTION="${1:-build}"

# Find Project_A root by locating tools/esp-idf in parent directories
find_project_root() {
  local d="$PWD"
  while [[ "$d" != "/" ]]; do
    if [[ -d "$d/tools/esp-idf" ]]; then
      echo "$d"; return 0
    fi
    d="$(dirname "$d")"
  done
  echo "ERROR: Could not find Project_A root (tools/esp-idf not found in parents)." >&2
  exit 2
}

# Find nearest ESP-IDF app root by locating CMakeLists.txt + main/
find_idf_app_root() {
  local d="$PWD"
  while [[ "$d" != "/" ]]; do
    if [[ -f "$d/CMakeLists.txt" && -d "$d/main" ]]; then
      echo "$d"; return 0
    fi
    d="$(dirname "$d")"
  done
  echo "ERROR: Could not find an ESP-IDF app in parents (need CMakeLists.txt + main/)." >&2
  exit 3
}

ROOT="$(find_project_root)"
export IDF_PATH="$ROOT/tools/esp-idf"

# Load ESP-IDF environment (puts idf.py and toolchains on PATH)
# shellcheck disable=SC1091
source "$IDF_PATH/export.sh" >/dev/null

APP="$(find_idf_app_root)"

echo "ESP-IDF action: $ACTION"
echo "ESP-IDF app:    $APP"

exec idf.py -C "$APP" "$ACTION"
