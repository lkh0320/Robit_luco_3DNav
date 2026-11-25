#!/bin/bash
# 로봇 PC <-> 운영(개발) PC 시간 동기화 (팀 문서 "Luco 베이스 제어" 9장 chrony 설정)
#   로봇 PC:  sudo bash setup_chrony.sh robot     (NTP 서버: allow + local stratum 10)
#   운영 PC:  sudo bash setup_chrony.sh pc        (로봇 PC를 따름: server ... iburst prefer)
#
# /etc/chrony/chrony.conf 맨 아래에 "# >>> luco chrony" ~ "# <<< luco chrony" 블록을 넣음.
# 다시 실행하면 블록을 새로 씀(중복 안 됨). 원본은 chrony.conf.luco-bak 으로 한 번만 백업.
# 망: 현장망 192.168.1.0/24 (로봇 192.168.1.5) + 지금 쓰는 망 192.168.0.0/24 (로봇 192.168.0.68)
#     바꾸려면 LUCO_NETS="192.168.1.0/24 ..." LUCO_ROBOT_IPS="192.168.1.5 ..." 로 지정
set -euo pipefail

ROLE=${1:-}
NETS=${LUCO_NETS:-"192.168.1.0/24 192.168.0.0/24"}
ROBOT_IPS=${LUCO_ROBOT_IPS:-"192.168.1.5 192.168.0.68"}
CONF=/etc/chrony/chrony.conf

[ "$(id -u)" = 0 ] || { echo "sudo 로 실행: sudo bash $0 $ROLE"; exit 1; }
case "$ROLE" in robot|pc) ;; *) sed -n '2,4p' "$0"; exit 1 ;; esac
command -v chronyd > /dev/null || { echo "chrony 가 설치되지 않음: sudo apt install chrony"; exit 1; }

[ -f "$CONF.luco-bak" ] || cp "$CONF" "$CONF.luco-bak"

# 이전 블록 제거
sed -i '/^# >>> luco chrony/,/^# <<< luco chrony/d' "$CONF"

if [ "$ROLE" = robot ]; then
  # 로봇 PC에 잘못 들어간 운영 PC용 줄(자기 자신을 서버로 지정) 주석 처리
  for ip in $ROBOT_IPS; do
    sed -i "s|^server $ip |# (luco: 로봇 PC 자신이라 비활성) server $ip |" "$CONF"
  done
  {
    echo "# >>> luco chrony (robot: NTP 서버)"
    echo "# 운영 PC들이 이 PC를 NTP 서버로 쓰도록 허용 (/24 = 서브넷 마스크 255.255.255.0)"
    for n in $NETS; do echo "allow $n"; done
    echo "# 인터넷이 될 때는 실제 시각을 따름 (이게 없으면 로봇 시계가 혼자 틀어져서"
    echo "# 인터넷 NTP도 같이 쓰는 운영 PC가 로봇을 '틀린 시계(x)'로 판정하고 무시함)"
    echo "pool ntp.ubuntu.com iburst maxsources 4"
    echo "# 인터넷(상위 NTP 서버)이 끊겨도 이 PC의 시계를 기준으로 시간을 제공"
    echo "local stratum 10"
    echo "# <<< luco chrony"
  } >> "$CONF"
  # 방화벽이 켜져 있으면 NTP(UDP 123) 허용
  if command -v ufw > /dev/null && ufw status | grep -q "Status: active"; then
    for n in $NETS; do ufw allow from "$n" to any port 123 proto udp; done
  fi
else
  {
    echo "# >>> luco chrony (pc: 로봇 PC를 따름)"
    echo "# 로봇 PC IP. iburst: 시작 직후 빠르게 여러 번 질의, prefer: 여러 서버 중 우선 선택"
    for ip in $ROBOT_IPS; do echo "server $ip iburst prefer"; done
    echo "# <<< luco chrony"
  } >> "$CONF"
fi

systemctl restart chrony
sleep 3
echo "== 적용된 블록"; sed -n '/^# >>> luco chrony/,/^# <<< luco chrony/p' "$CONF"
if [ "$ROLE" = pc ]; then
  # 시차가 크면 바로 맞춤 (문서: sudo chronyc makestep)
  chronyc -a makestep > /dev/null || true
  echo "== chronyc sources"; chronyc sources
  echo "== chronyc tracking"; chronyc tracking | grep -E "Reference ID|Stratum|System time|Leap"
else
  echo "== chronyc tracking"; chronyc tracking | grep -E "Reference ID|Stratum|Leap"
fi
