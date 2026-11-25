#!/bin/bash
# 지도(maps/pcd/<이름>)를 GitHub Releases에 올리고 받는 스크립트.
# 지도는 용량이 커서 git에 넣지 않고, 지도 하나당 릴리스 하나(태그 map-<이름>)에 tar.gz로 붙인다.
#
#   map_release.sh upload <이름> [설명]   maps/pcd/<이름> -> 릴리스 map-<이름>
#   map_release.sh download <이름>        릴리스 map-<이름> -> maps/pcd/<이름>
#   map_release.sh list                   올라간 지도 목록
#
# 올리기: gh (GitHub CLI) 로그인 필요 (gh auth login).
# 받기/목록: 공개 레포라 gh 없이 curl로도 됨 (gh가 있으면 gh 사용).
# 워크스페이스 위치: 이 스크립트 기준 ../../../.. (ws/src/luco3dnav/luco3dnav_bringup/scripts)
# 다른 곳이면 LUCO_WS=/경로/ws 로 지정.
set -euo pipefail

REPO=${LUCO_MAP_REPO:-lkh0320/Robit_luco_3DNav}
HERE=$(cd "$(dirname "$(realpath "$0")")" && pwd)
WS=${LUCO_WS:-$(cd "$HERE/../../../.." && pwd)}
PCD_DIR=$WS/maps/pcd

usage() { sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'; exit 1; }
need_gh() { command -v gh > /dev/null || { echo "gh(GitHub CLI)가 없음: sudo apt install gh"; exit 1; }; }

has_gh() { command -v gh > /dev/null; }

cmd=${1:-}; [ -n "$cmd" ] || usage

case "$cmd" in
upload)
  need_gh
  name=${2:-}; [ -n "$name" ] || usage
  note=${3:-}
  src=$PCD_DIR/$name
  [ -f "$src/map.pcd" ] || { echo "$src/map.pcd 가 없음"; exit 1; }
  tag=map-$name
  if gh release view "$tag" -R "$REPO" > /dev/null 2>&1; then
    echo "이미 있는 릴리스: $tag (덮어쓰려면 먼저 gh release delete $tag -R $REPO --cleanup-tag)"; exit 1
  fi
  tmp=$(mktemp -d); trap 'rm -rf "$tmp"' EXIT
  tar -C "$PCD_DIR" -czf "$tmp/$name.tar.gz" "$name"
  kf=$(wc -l < "$src/poses.txt" 2>/dev/null || echo 0)
  commit=$(git -C "$HERE" rev-parse --short HEAD 2>/dev/null || echo unknown)
  cat > "$tmp/notes.md" <<EOF
3D 지도 \`$name\` (FAST-LIO2 + PGO)

${note}

| 항목 | 값 |
|---|---|
| map.pcd | $(du -h "$src/map.pcd" | cut -f1) |
| 키프레임 수 (poses.txt) | $kf |
| patches | $([ -d "$src/patches" ] && echo 포함 || echo 없음) |
| 저장 시각 | $(date -r "$src/map.pcd" '+%Y-%m-%d %H:%M') |
| luco3dnav 커밋 | $commit |

받기: \`luco3dnav_bringup/scripts/map_release.sh download $name\` -> \`maps/pcd/$name/\`
EOF
  gh release create "$tag" "$tmp/$name.tar.gz" -R "$REPO" --title "map: $name" --notes-file "$tmp/notes.md" --latest=false
  ;;
download)
  name=${2:-}; [ -n "$name" ] || usage
  dst=$PCD_DIR/$name
  [ -e "$dst" ] && { echo "$dst 가 이미 있음 (지우거나 옮긴 뒤 다시)"; exit 1; }
  mkdir -p "$PCD_DIR"
  tmp=$(mktemp -d); trap 'rm -rf "$tmp"' EXIT
  if has_gh; then
    gh release download "map-$name" -R "$REPO" -p "$name.tar.gz" -D "$tmp"
  else
    curl -fL --progress-bar -o "$tmp/$name.tar.gz" \
      "https://github.com/$REPO/releases/download/map-$name/$name.tar.gz" \
      || { echo "받기 실패: map-$name 릴리스가 있는지 확인 (map_release.sh list)"; exit 1; }
  fi
  tar -C "$PCD_DIR" -xzf "$tmp/$name.tar.gz"
  echo "받음: $dst"; ls "$dst"
  ;;
list)
  if has_gh; then
    gh release list -R "$REPO" --limit 100 | awk -F'\t' '$3 ~ /^map-/ {print $3 "\t" $4}' | sed 's/^map-//'
  else
    curl -fsSL "https://api.github.com/repos/$REPO/releases?per_page=100" | python3 -c '
import json, sys
for r in json.load(sys.stdin):
    if r["tag_name"].startswith("map-"):
        print(r["tag_name"][4:] + "\t" + r["published_at"])'
  fi
  ;;
*) usage ;;
esac
