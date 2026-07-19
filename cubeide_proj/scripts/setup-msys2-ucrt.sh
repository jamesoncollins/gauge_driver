#!/usr/bin/env sh
set -eu

usage() {
  cat <<'EOF'
Usage: ./scripts/setup-msys2-ucrt.sh [--no-system-update]

Installs the MSYS2 UCRT64 packages listed in env/msys2-ucrt-packages.txt.
Run from an MSYS2 shell.
EOF
}

no_system_update=0
while [ "$#" -gt 0 ]; do
  case "$1" in
    --no-system-update)
      no_system_update=1
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repo_root=$(CDPATH= cd -- "$script_dir/.." && pwd)
package_file="$repo_root/env/msys2-ucrt-packages.txt"

. "$script_dir/env-msys2-ucrt.sh"

if ! command -v pacman >/dev/null 2>&1; then
  echo "pacman was not found. Run this from an MSYS2 shell." >&2
  exit 1
fi

if [ ! -f "$package_file" ]; then
  echo "Package list not found: $package_file" >&2
  exit 1
fi

packages=$(
  sed -e 's/[[:space:]]*#.*$//' -e '/^[[:space:]]*$/d' "$package_file"
)

if [ -z "$packages" ]; then
  echo "No packages listed in $package_file" >&2
  exit 1
fi

if [ "$no_system_update" -eq 0 ]; then
  pacman --noconfirm -Syu
fi

# Package names do not contain spaces; word splitting is intentional here.
pacman --noconfirm --needed -S $packages

missing=0
for tool in gcc cmake ninja arm-none-eabi-gcc emcc; do
  if command -v "$tool" >/dev/null 2>&1; then
    printf '%-18s %s\n' "$tool:" "$(command -v "$tool")"
  else
    echo "Missing required tool: $tool" >&2
    missing=1
  fi
done

exit "$missing"
