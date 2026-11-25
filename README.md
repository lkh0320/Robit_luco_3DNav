# luco3dnav

Luco 로봇(4륜 스키드 + VLP-16 + EBIMU)용 FAST-LIO2 3D SLAM / 지도 패키지 모음. ROS 2 Jazzy, Ubuntu 24.04.

| 패키지 | 역할 |
|---|---|
| `luco3dnav_sensors` | EBIMU 시리얼 → `/imu/data`, Velodyne VLP-16 → `/velodyne_points` |
| `luco3dnav_bringup` | SLAM 설정(`lio.yaml`, `pgo.yaml`), `mapping.launch.py` / `view.launch.py`, 스크립트(`scripts/`) |
| `luco3dnav_mapping` | 지도 레이어(3D voxel / 2D occupancy / 2.5D elevation), 지도 저장·후처리 도구 ([README](luco3dnav_mapping/README.md)) |
| `luco3dnav_navigation` | (작업 예정, 아직 ROS 패키지 아님) localizer, Nav2 |
| `third_party/FASTLIO2_ROS2` | [beomsuchoi/FASTLIO2_ROS2](https://github.com/beomsuchoi/FASTLIO2_ROS2) 수정본 (아래 [변경점](#fastlio2_ros2-변경점)). `hba`는 빌드 제외 |

## 설치

```bash
cd ~/luco_ws/src && git clone https://github.com/lkh0320/Robit_luco_3DNav.git luco3dnav
cd ~/luco_ws && rosdep install --from-paths src --ignore-src -y
sudo usermod -aG dialout $USER                  # IMU 시리얼 권한, 로그아웃 후 적용
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

- 로봇 모델/TF는 별도 레포 `luco2_description`을 같은 `src/`에 둔다. 없으면 `use_description:=false`(정적 TF로 대신).
- 라이다: PC 유선 랜 `192.168.1.100/24`, 라이다 `192.168.1.201`.
- (선택) IMU 고정 이름 `/dev/ebimu`: `sudo cp luco3dnav_sensors/udev/99-ebimu.rules /etc/udev/rules.d/ && sudo udevadm control --reload && sudo udevadm trigger`

## 명령어 요약

워크스페이스 루트에서 실행. `S=src/luco3dnav/luco3dnav_bringup/scripts`

| 하고 싶은 것 | 명령 |
|---|---|
| SLAM (센서 직접 연결, RViz 포함) | `ros2 launch luco3dnav_bringup mapping.launch.py` |
| SLAM + 지도 레이어 | `ros2 launch luco3dnav_mapping layers.launch.py` |
| 로봇에서 SLAM (luco_bringup 실행 중) | `ros2 launch luco3dnav_mapping layers.launch.py wheel_odom:=true use_description:=false use_static_tf:=false use_rviz:=false` |
| 개발 PC에서 원격 보기 | `ros2 launch luco3dnav_bringup view.launch.py` |
| 지도 저장 (SLAM 도는 PC에서) | `ros2 run luco3dnav_mapping save_maps [이름]` |
| bag → 지도 한 번에 | `$S/bag_to_map.sh bags/<bag> [이름]` |
| bag 재생해서 보기 | `ros2 launch luco3dnav_mapping layers.launch.py use_sensors:=false` + `ros2 bag play bags/<bag> --clock --topics /imu/data /velodyne_points` |
| 저장된 지도에서 사람 등 지우기 | `ros2 run luco3dnav_mapping remove_dynamic maps/pcd/<이름> --replace` |
| 저장된 지도의 레이어 다시 만들기 | `ros2 run luco3dnav_mapping rebuild_layers maps/pcd/<이름>` |
| 지도 공유 | `$S/map_release.sh upload <이름> "설명"` / `list` / `download <이름>` |
| 지도 보기 | `pcl_viewer maps/pcd/<이름>/map.pcd` (뷰어에서 `5` = 높이 색) |
| 두 PC 원격 통신 설정 | `source $S/luco_net.sh` (개발 PC는 alias `luco_net`) |
| 두 PC 시계 맞추기 (한 번) | `sudo bash $S/setup_chrony.sh robot` (로봇 먼저) / `... pc` |

두 PC 경로, alias, 순서까지 포함한 전체 명령은 워크스페이스 루트 `Luco3DNav_명령어_모음.pdf`.

## 지도 만들기 흐름

1. **SLAM 실행**: 로봇이면 luco_bringup + `layers.launch.py wheel_odom:=true ...`, 개발 PC면 `layers.launch.py`.
2. **저장**: `save_maps` → `maps/pcd/<이름>/`
   - `map.pcd`: PGO 3D 점군. 움직인 물체(사람 등)는 자동으로 지운 정적 지도 (원본 `map_raw.pcd`, 지운 점 `map_dynamic.pcd`).
   - `octomap.bt`, `grid.pgm/yaml`(Nav2 형식), `elevation.npz/pgm/yaml`: 실시간으로 쌓인 레이어.
   - `--rebuild-layers`: 레이어를 실시간 결과 대신 **PGO 최종 자세 + 정적 점**으로 다시 만든다(약 1분). 루프 보정이 반영되고 사람 자국이 빠진다.
3. **bag으로 다시 만들기**: `bag_to_map.sh`가 1~2를 자동으로 함 (`--rebuild-layers` 포함).

지도/bag 위치: 개발 PC는 워크스페이스 `maps/pcd/`, `bags/`, 로봇 PC는 레포 안 `maps/pcd/`, `bags/` (git 제외, 공유는 `map_release.sh`).

## 원격 (로봇 PC 계산, 개발 PC는 RViz)

두 PC 모두 luco 터미널마다 `luco_net.sh`를 source (도메인 46, CycloneDDS 유니캐스트, 로봇 `192.168.0.68` ↔ PC `192.168.0.111`).
IP가 바뀌면 `export LUCO_ROBOT_IP=... LUCO_PC_IP=...` 후 source. `~/.bashrc`는 바꾸지 않는다.

```bash
# 로봇 PC
ros2 launch luco_bringup luco.launch.py
ros2 launch luco3dnav_mapping layers.launch.py wheel_odom:=true use_description:=false use_static_tf:=false use_rviz:=false
# 개발 PC
ros2 launch luco3dnav_bringup view.launch.py
```

두 PC 시계가 다르면 원격 RViz에서 TF 에러 → `setup_chrony.sh`.

## 구조와 주의

- **TF**: `map -(PGO)-> lio_world -(FAST-LIO)-> odom -(휠 오도메트리)-> base_footprint -> base_link -> velodyne / imu_link`.
  `wheel_odom:=true`면 FAST-LIO가 `lio_world -> odom`을 발행해서 휠 odom TF와 안 겹침. 휠 odom이 없으면(bag/핸드헬드) `lio_world -> base_footprint` 직접.
- **센서 외부 파라미터**(base_link 기준 실측 mm): velodyne (125.45, 0, 306.4), IMU (101.0, 2.25, 254.8), IMU는 z축 -90° 장착.
  `lio.yaml`의 `r_bi/t_bi/r_il/t_il`, `luco2_description` URDF, `mapping.launch.py` 정적 TF를 항상 같이 맞출 것.
- **bag**: 센서 토픽(`/imu/data /velodyne_points`, 로봇이면 `+ /diff_drive_controller/odom`)만 녹화. TF는 재생 때 다시 계산되므로 설정을 바꿔도 재사용 가능. 재생은 `--clock` 필수.

## FASTLIO2_ROS2 변경점

| 위치 | 내용 |
|---|---|
| `fastlio2/src/utils.cpp` | **스캔 왜곡 보정**: velodyne 점별 `time` 필드를 읽어 IMU 디스큐 (`lio.yaml` `lidar_deskew`). 원본은 보정이 없어 회전 중 벽/기둥이 두 겹으로 찍힘. 쓰지 않던 법선 계산 제거 |
| `fastlio2/src/lio_node.cpp` | `body_frame` 기준 출력(`r_bi/t_bi`), 휠 odom 연동(`odom_frame`), `lidar_time_offset` |
| `fastlio2/src/map_builder/lidar_processor.cpp` | 퇴화 방향(복도 진행 방향 등) 라이다 가중치 감소 (`degen_enable`) |
| `pgo/src` | `keyframes.txt`(시각 + LIO/보정 자세), `loops.txt` 저장, `loop_icp_max_corr` 설정 |
| `fastlio2/CMakeLists.txt` | `Sophus::Sophus` 링크 |

튜닝 기록과 근거는 `lio.yaml` 주석에. bag 반복 재생 튜닝 도구는 개발 PC 워크스페이스 `tune/dyn/`(레포 밖).
