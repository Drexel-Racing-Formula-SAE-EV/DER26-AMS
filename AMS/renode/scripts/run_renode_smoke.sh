#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
ELF="$ROOT_DIR/AMS/build/DER26-AMS.elf"
SCRIPT="$ROOT_DIR/AMS/renode/scripts/ams_f767_cli_smoke.resc"

msys_to_windows_path() {
  local path="$1"

  case "$path" in
    /[a-zA-Z]/*)
      printf "%s:/%s\n" "${path:1:1}" "${path:3}"
      ;;
    *)
      printf "%s\n" "$path"
      ;;
  esac
}

if ! command -v renode >/dev/null 2>&1; then
  echo "ERROR: renode is not installed or not on PATH." >&2
  exit 127
fi

if [[ ! -f "$ELF" ]]; then
  echo "ERROR: missing $ELF" >&2
  echo "Run ./ci/stm32/build_ams_headless_gcc.sh from the repo root first." >&2
  exit 2
fi

cd "$ROOT_DIR"

RENODE_SCRIPT="$SCRIPT"
if [[ "${OSTYPE:-}" == msys* || "${OSTYPE:-}" == cygwin* ]]; then
  RENODE_SCRIPT="$(msys_to_windows_path "$SCRIPT")"
fi

echo "Launching Renode with: $RENODE_SCRIPT"
renode --console -e "include @$RENODE_SCRIPT"
