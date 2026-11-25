#!/bin/bash
# 로봇 PC <-> 개발 PC 원격 ROS 통신 환경. luco 관련 터미널에서 "source" 해서 씀 (실행 X).
#
#   source ~/luco_ws/src/luco3dnav/luco3dnav_bringup/scripts/luco_net.sh          (로봇 PC)
#   source ~/<ws>/src/luco3dnav/luco3dnav_bringup/scripts/luco_net.sh             (개발 PC)
#
# 같은 터미널의 모든 노드(bringup, mapping, rviz ...)가 같은 설정이어야 서로 보임.
# IP가 바뀌면: LUCO_ROBOT_IP=... LUCO_PC_IP=... 를 먼저 export 하고 source.
# 끄려면 새 터미널을 열면 됨 (~/.bashrc 는 건드리지 않음).

_luco_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

export LUCO_ROBOT_IP=${LUCO_ROBOT_IP:-192.168.0.68}
export LUCO_PC_IP=${LUCO_PC_IP:-192.168.0.111}
export ROS_DOMAIN_ID=${LUCO_DOMAIN_ID:-46}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE

# 이 PC가 둘 중 어느 쪽인지 자동 판별
if ip -4 addr | grep -q "inet ${LUCO_ROBOT_IP}/"; then
  export LUCO_SELF_IP=$LUCO_ROBOT_IP; _luco_role=로봇
elif ip -4 addr | grep -q "inet ${LUCO_PC_IP}/"; then
  export LUCO_SELF_IP=$LUCO_PC_IP; _luco_role=개발PC
else
  echo "[luco_net] 이 PC에 ${LUCO_ROBOT_IP} / ${LUCO_PC_IP} 주소가 없음. IP 확인 후 LUCO_ROBOT_IP/LUCO_PC_IP 지정"
  ip -4 -br addr | grep -v "^lo"
  return 1 2>/dev/null || exit 1
fi

# share 설치본이 있으면 그걸, 없으면 소스의 config 사용
_luco_xml=$_luco_dir/../config/dds/cyclonedds_luco.xml
export CYCLONEDDS_URI=file://$(realpath "$_luco_xml")

echo "[luco_net] ${_luco_role} ${LUCO_SELF_IP} | 도메인 ${ROS_DOMAIN_ID} | 피어 로봇 ${LUCO_ROBOT_IP}, PC ${LUCO_PC_IP}"
unset _luco_dir _luco_xml _luco_role
