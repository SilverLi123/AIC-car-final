#!/usr/bin/env bash
# Restore the three ring-map presets into parameter_presets/, keeping timestamped copies
# of any file that is about to be overwritten.
set -Eeuo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../../.." && pwd)"
DEST="$ROOT/src/line_follow_ws/src/line_follower/config/parameter_presets"
FILES=(
  RB_FORK85_085_R040_P045_K255_Y170_Y020_HOLD045.yaml
  RB_FORK85_085_R040_P045_K255_Y170_Y020_HOLD05.yaml
  RB_FORK85_085_R040_P045_K255_Y170_Y020_HOLD04.yaml
)
STAMP="$(date +%Y%m%d_%H%M%S)"

[[ -d "$DEST" ]] || { echo "找不到参数目录：$DEST" >&2; exit 1; }
[[ -f "$HERE/SHA256SUMS" ]] || { echo "找不到 SHA256SUMS" >&2; exit 1; }

echo "校验备份文件..."
if command -v sha256sum >/dev/null 2>&1; then
  (cd "$HERE" && sha256sum -c SHA256SUMS)
else
  (cd "$HERE" && shasum -a 256 -c SHA256SUMS)
fi

for f in "${FILES[@]}"; do
  [[ -f "$HERE/$f" ]] || { echo "备份缺少文件：$f" >&2; exit 1; }
  if [[ -f "$DEST/$f" ]]; then
    cp -p "$DEST/$f" "$DEST/$f.bak.$STAMP"
    echo "已保留原文件：$f.bak.$STAMP"
  fi
  cp -p "$HERE/$f" "$DEST/$f"
  echo "已恢复：$f"
done

echo "校验恢复后的文件..."
if command -v sha256sum >/dev/null 2>&1; then
  (cd "$DEST" && sha256sum -c "$HERE/SHA256SUMS")
else
  (cd "$DEST" && shasum -a 256 -c "$HERE/SHA256SUMS")
fi

echo "完成。重启 line_follower 后生效。"
