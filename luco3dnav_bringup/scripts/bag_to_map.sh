#!/bin/bash
# bag 하나로 지도 폴더 만들기 (RViz 없이, 끝나면 자동 종료)
#
#   bag_to_map.sh <bag 폴더> [지도 이름] [재생 길이(초)]   이름 기본값 = bag 폴더 이름
#   재생 길이: bag 앞에서부터 이만큼만 씀 (끝에 턱에 걸리는 등 망가진 부분 빼기)
#
# 순서: SLAM(FAST-LIO2 + PGO) + 레이어를 bag으로 돌림 -> save_maps (PGO 지도 + 사람 등 제거)
#       -> rebuild_layers (octomap/2D/elevation을 PGO 최종 자세 + 정적 점으로 다시 만듦)
# 결과: maps/pcd/<이름>/ (개발 PC는 워크스페이스, 로봇 PC는 clone 안). 같은 이름이 있으면 덮어씀.
# 자체 ROS 도메인(기본 88, localhost)에서 돌려서 실행 중인 로봇 SLAM과 섞이지 않음 (LUCO_BAG_DOMAIN으로 변경).
# bag 길이만큼 걸림 (실시간 재생).
set -eo pipefail

[ $# -ge 1 ] || { sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'; exit 1; }
BAG=$(realpath "$1")
NAME=${2:-$(basename "$BAG")}
DUR=${3:-}
[ -f "$BAG/metadata.yaml" ] || { echo "bag 폴더가 아님: $BAG"; exit 1; }

HERE=$(cd "$(dirname "$(realpath "$0")")" && pwd)
WS=${LUCO_WS:-$(cd "$HERE/../../../.." && pwd)}
set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=${LUCO_BAG_DOMAIN:-88} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI ROS_LOCALHOST_ONLY

LOG=$(mktemp -d /tmp/bag_to_map_XXXX)
echo "[bag_to_map] $BAG -> $NAME (로그 $LOG)"
# 새 프로세스 그룹으로 띄워서 끝날 때 이 그룹(launch + 노드)만 정리 (로봇에서 도는 SLAM은 안 건드림)
setsid ros2 launch luco3dnav_mapping layers.launch.py use_sensors:=false use_rviz:=false > "$LOG/launch.log" 2>&1 &
PGID=$!
cleanup() {
  kill -INT -- -$PGID 2>/dev/null || true; sleep 4; kill -9 -- -$PGID 2>/dev/null || true
}
trap cleanup EXIT
sleep 6
echo "[bag_to_map] 재생 중 ${DUR:+(앞 $DUR s 만) }($(grep -m1 -A1 'duration' "$BAG/metadata.yaml" | grep -o '[0-9]*' | tail -1 | awk '{printf "%.0f", $1/1e9}') s)..."
ros2 bag play "$BAG" --clock --topics /imu/data /velodyne_points ${DUR:+--playback-duration $DUR} > "$LOG/play.log" 2>&1
sleep 3
ros2 run luco3dnav_mapping save_maps "$NAME" --rebuild-layers
