# luco3dnav

Luco 로봇용 FAST-LIO2 기반 3D 맵핑/내비게이션 패키지 모음 (ROS 2 Jazzy, Ubuntu 24.04).

| 패키지 | 역할 |
|---|---|
| `third_party/FASTLIO2_ROS2` | [beomsuchoi/FASTLIO2_ROS2](https://github.com/beomsuchoi/FASTLIO2_ROS2) 복사본. fastlio2 CMake에 `Sophus::Sophus` 한 줄 추가, `hba`는 빌드 제외 |
| `luco3dnav_sensors` | EBIMU 시리얼 → `/imu/data`(sensor_msgs/Imu), Velodyne VLP-16 → `/velodyne_points` |
| `luco3dnav_bringup` | 우리 `lio.yaml`/`pgo.yaml`, `mapping.launch.py` |
| `luco3dnav_mapping` | 맵 레이어: 3D voxel / 2D occupancy (octomap), 2.5D elevation (grid_map). `layers.launch.py` |
| `luco3dnav_navigation` | (작업 예정) localizer, Nav2 설정 |

## 설치

```bash
cd ~/luco_ws/src && git clone <this repo> luco3dnav
cd ~/luco_ws && rosdep install --from-paths src --ignore-src -y
sudo usermod -aG dialout $USER     # 이후 로그아웃/로그인
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

선택: IMU에 고정 이름(`/dev/ebimu`) 주기

```bash
sudo cp src/luco3dnav/luco3dnav_sensors/udev/99-ebimu.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger
```

로봇 모델/TF: description은 별도 레포(`luco2_description`)를 같은 `src/`에 둔다. `mapping.launch.py`가 기본으로 불러오며(`use_description:=true`), 없으면 `use_description:=false`로 실행하면 정적 TF(base_link→velodyne/imu_link)를 대신 발행한다.

라이다: PC 유선 랜 `192.168.1.100/24`, 라이다 `192.168.1.201`.

## 실행

```bash
source install/setup.bash
ros2 launch luco3dnav_bringup mapping.launch.py                    # imu_port:=/dev/ebimu, use_rviz:=false 등
```

맵 저장 (워크스페이스 루트의 `maps/pcd`):

```bash
ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: '$PWD/maps/pcd', save_patches: true}"
```

원격 보기 (로봇 PC에서 SLAM 계산, 개발 PC에서 RViz만). 두 PC 모두 luco 터미널마다 먼저:

```bash
source src/luco3dnav/luco3dnav_bringup/scripts/luco_net.sh   # 도메인 46, CycloneDDS 유니캐스트 (로봇 192.168.0.68 <-> PC 192.168.0.111)
```

```bash
# 로봇 PC
ros2 launch luco_bringup luco.launch.py
ros2 launch luco3dnav_bringup mapping.launch.py wheel_odom:=true use_description:=false use_static_tf:=false use_rviz:=false
# 개발 PC
ros2 launch luco3dnav_bringup view.launch.py
```

IP가 바뀌면 `LUCO_ROBOT_IP=... LUCO_PC_IP=...`를 export한 뒤 source. `~/.bashrc`는 건드리지 않으므로 새 터미널은 원래 설정.

지도 공유 (GitHub Releases, 지도 하나당 릴리스 `map-<이름>`):

```bash
src/luco3dnav/luco3dnav_bringup/scripts/map_release.sh upload luco_1125_1954 "설명"   # maps/pcd/<이름> 올리기
src/luco3dnav/luco3dnav_bringup/scripts/map_release.sh list                            # 올라간 지도 목록
src/luco3dnav/luco3dnav_bringup/scripts/map_release.sh download luco_1125_1954         # maps/pcd/<이름> 로 받기
```

올리기는 `gh auth login` 필요, 받기는 gh 없이도 됨(curl). 레포가 공개라 올린 지도도 공개됨.
지도 위치: 개발 PC는 워크스페이스 `maps/pcd/`, 로봇 PC는 레포 안 `maps/pcd/`(git 제외). bag도 로봇 PC에서는 레포 안 `bags/`에 녹화.

## 주의

- **TF 구조**: `map -(PGO)-> lio_world -(FAST-LIO)-> odom -(휠 오도메트리)-> base_footprint`. luco_bringup과 같이 쓸 때는 `wheel_odom:=true`로 FAST-LIO가 `lio_world -> odom`을 발행해서 휠 odom TF와 겹치지 않음. 휠 odom이 없으면(bag/핸드헬드) `lio_world -> base_footprint` 직접 발행.
- **시간 동기화**: 로봇 PC와 운영 PC 시계가 다르면 원격 RViz에서 TF 에러. `scripts/setup_chrony.sh` (로봇 PC `robot` 먼저, 운영 PC `pc`), sudo 필요.

- `lio.yaml`의 `r_bi`/`t_bi`(base_link→IMU), `r_il`/`t_il`(IMU→라이다)은 luco2 실측값. IMU는 로봇 대비 z축 -90도 장착. 바꿀 때는 description 레포(`luco2_description`) URDF와 `mapping.launch.py` 정적 TF도 같이 맞출 것.
- **스캔 왜곡 보정**: 원본 포크는 점별 시간을 안 써서 보정이 없었음 → velodyne `time` 필드로 IMU 디스큐하도록 수정 (`lio.yaml` `lidar_deskew: true`). 회전 중 벽/기둥이 두 겹으로 찍히던 문제 해결 (bags/luco_1126_0201 방문 간 어긋남 71→48 mm). 튜닝 도구: 워크스페이스 `tune/dyn/` (bag 재생 + 지표).
- **움직이는 물체(사람) 제거**: `save_maps`가 PGO 지도 저장 후 자동으로 `remove_dynamic`을 돌려 `map.pcd`를 정적 지도로 바꿈 (원본 `map_raw.pcd`, 지운 점 `map_dynamic.pcd`). 끄려면 `--keep-dynamic`. 이미 저장한 지도에도 `ros2 run luco3dnav_mapping remove_dynamic maps/pcd/<이름> --replace` (patches 필요). 같은 곳을 나중에 다시 보지 못하면 지울 근거가 없어 남음. octomap/2D/elevation 레이어는 실시간으로 쌓여서 이 처리 대상이 아님.
