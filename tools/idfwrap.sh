#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

if (($#)); then
  IDF_ARGS=("$@")
else
  IDF_ARGS=(build)
fi

resolve_idf_path() {
  local repo_idf_path="$ROOT/tools/esp-idf"

  if [[ -n "${IDF_PATH:-}" && -f "${IDF_PATH}/export.sh" ]]; then
    printf '%s\n' "$IDF_PATH"
    return 0
  fi

  if [[ -f "$repo_idf_path/export.sh" ]]; then
    printf '%s\n' "$repo_idf_path"
    return 0
  fi

  cat >&2 <<EOF
ERROR: Could not locate ESP-IDF.

Checked:
  1. \$IDF_PATH
  2. $repo_idf_path

Fix one of these:
  - export IDF_PATH to an existing ESP-IDF checkout
  - or clone ESP-IDF into tools/esp-idf
  - or source ./setup_env.sh from the repo root
EOF
  exit 2
}

is_idf_app_root() {
  local d="$1"
  [[ -f "$d/CMakeLists.txt" ]] && grep -Eq 'tools/cmake/project\.cmake' "$d/CMakeLists.txt"
}

find_idf_apps_under() {
  local base_dir="$1"

  find "$base_dir" \
    \( -path "$ROOT/.git" -o -path "$ROOT/tools/esp-idf" -o -path '*/build' -o -path '*/managed_components' \) -prune -o \
    -name CMakeLists.txt -print |
    while IFS= read -r cmake_file; do
      local app_dir
      app_dir="$(dirname "$cmake_file")"
      if is_idf_app_root "$app_dir"; then
        printf '%s\n' "$app_dir"
      fi
    done
}

find_repo_app_roots() {
  find_idf_apps_under "$ROOT"
}

resolve_idf_app_path() {
  local requested_path="$1"

  if is_idf_app_root "$requested_path"; then
    printf '%s\n' "$requested_path"
    return 0
  fi

  if [[ ! -d "$requested_path" ]]; then
    echo "ERROR: '$requested_path' does not exist." >&2
    exit 4
  fi

  local candidate_apps=()
  mapfile -t candidate_apps < <(find_idf_apps_under "$requested_path")
  if ((${#candidate_apps[@]} == 1)); then
    printf '%s\n' "${candidate_apps[0]}"
    return 0
  fi

  if ((${#candidate_apps[@]} > 1)); then
    echo "ERROR: '$requested_path' contains multiple ESP-IDF apps. Point IDFWRAP_APP at one app root." >&2
    printf '  %s\n' "${candidate_apps[@]}" >&2
    exit 5
  fi

  echo "ERROR: '$requested_path' is not an ESP-IDF app root and does not contain one." >&2
  exit 4
}

find_idf_app_root() {
  if [[ -n "${IDFWRAP_APP:-}" ]]; then
    resolve_idf_app_path "$IDFWRAP_APP"
    return 0
  fi

  local d="$PWD"
  while [[ "$d" != "/" ]]; do
    if is_idf_app_root "$d"; then
      echo "$d"; return 0
    fi
    d="$(dirname "$d")"
  done

  local repo_apps=()
  mapfile -t repo_apps < <(find_repo_app_roots)
  if ((${#repo_apps[@]} == 1)); then
    printf '%s\n' "${repo_apps[0]}"
    return 0
  fi

  if ((${#repo_apps[@]} > 1)); then
    echo "ERROR: Found multiple ESP-IDF apps in the repo. Set IDFWRAP_APP or run from a specific app directory." >&2
    printf '  %s\n' "${repo_apps[@]}" >&2
    exit 5
  fi

  echo "ERROR: Could not find an ESP-IDF app in parents or anywhere else in the repo." >&2
  exit 3
}

export IDF_PATH="$(resolve_idf_path)"

# Load ESP-IDF environment (puts idf.py and toolchains on PATH)
# shellcheck disable=SC1091
source "$IDF_PATH/export.sh" >/dev/null

APP="$(find_idf_app_root)"

echo "ESP-IDF args:   ${IDF_ARGS[*]}"
echo "ESP-IDF app:    $APP"

exec idf.py -C "$APP" "${IDF_ARGS[@]}"
