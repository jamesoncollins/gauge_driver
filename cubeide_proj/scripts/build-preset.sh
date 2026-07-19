#!/usr/bin/env sh
set -eu

usage() {
  cat <<'EOF'
Usage: ./scripts/build-preset.sh [--fresh] [--no-build] <preset>

Configures and builds a CMake preset using the MSYS2 UCRT64 tool environment.
EOF
}

fresh=0
no_build=0
preset=

while [ "$#" -gt 0 ]; do
  case "$1" in
    --fresh)
      fresh=1
      ;;
    --no-build)
      no_build=1
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    -*)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
    *)
      if [ -n "$preset" ]; then
        echo "Only one preset may be specified." >&2
        usage >&2
        exit 2
      fi
      preset=$1
      ;;
  esac
  shift
done

if [ -z "$preset" ]; then
  usage >&2
  exit 2
fi

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
. "$script_dir/env-msys2-ucrt.sh"

if [ "$fresh" -eq 1 ]; then
  cmake --fresh --preset "$preset"
else
  cmake --preset "$preset"
fi

if [ "$no_build" -eq 0 ]; then
  cmake --build --preset "$preset"
fi
