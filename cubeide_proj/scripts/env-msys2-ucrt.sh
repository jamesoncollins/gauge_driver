#!/usr/bin/env sh

# Source this file from an MSYS2 shell:
#   . ./scripts/env-msys2-ucrt.sh

case ":$PATH:" in
  *:/ucrt64/bin:*) ;;
  *) PATH="/ucrt64/bin:$PATH" ;;
esac

case ":$PATH:" in
  *:/usr/bin:*) ;;
  *) PATH="/usr/bin:$PATH" ;;
esac

export MSYSTEM=UCRT64
export CHERE_INVOKING=1
export PATH

if [ "${0##*/}" = "env-msys2-ucrt.sh" ]; then
  echo "MSYS2 UCRT64 environment exported for this process."
  echo "Source this script to update your current shell:"
  echo "  . ./scripts/env-msys2-ucrt.sh"
fi
