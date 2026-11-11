#!/bin/bash
# 사용법: ~/ros2_ws/backup.sh "메모"
NAME="bungeoppang_$(date +%Y%m%d_%H%M)"
[ -n "$1" ] && NAME="${NAME}_$(echo "$1" | tr ' ' '_')"
DEST=~/bungeoppang_backups/$NAME
mkdir -p "$DEST"
cp -r ~/ros2_ws/src/bungeoppang/. "$DEST"/
cp ~/ros2_ws/*.sh "$DEST"/ 2>/dev/null
rm -rf "$DEST"/__pycache__ "$DEST"/bungeoppang/__pycache__
echo "✅ 백업 완료: $DEST"
