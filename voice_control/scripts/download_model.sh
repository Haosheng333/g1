#!/usr/bin/env bash
# Explicit, manual model download -- never run automatically by the node.
# Usage: ./download_model.sh [cn|en]  (default: cn)
set -euo pipefail

LANG_CHOICE="${1:-cn}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODELS_DIR="$SCRIPT_DIR/../models"
mkdir -p "$MODELS_DIR"

case "$LANG_CHOICE" in
  cn)
    URL="https://alphacephei.com/vosk/models/vosk-model-small-cn-0.22.zip"
    NAME="vosk-model-small-cn-0.22"
    ;;
  en)
    URL="https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip"
    NAME="vosk-model-small-en-us-0.15"
    ;;
  *)
    echo "usage: $0 [cn|en]" >&2
    exit 1
    ;;
esac

DEST="$MODELS_DIR/$NAME"
if [ -d "$DEST" ]; then
  echo "already present: $DEST"
  exit 0
fi

echo "downloading $URL ..."
ZIP_PATH="$MODELS_DIR/$NAME.zip"
curl -L -o "$ZIP_PATH" "$URL"
unzip -q "$ZIP_PATH" -d "$MODELS_DIR"
rm -f "$ZIP_PATH"

echo "installed: $DEST"
echo "set ~vosk_model_path or config/vosk.yaml's model_path to this directory"
