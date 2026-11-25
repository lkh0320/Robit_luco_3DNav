# luco3dnav_mapping

FAST-LIO2 + PGO 위에 올리는 맵 레이어. 입력은 모두 `/fastlio2/body_cloud` + TF(`map -> lio_world -> base_footprint`).

| 레이어 | 노드 | 토픽 | RViz |
|---|---|---|---|
| 3D voxel | `octomap_server` | `/octomap_full`, `/occupied_cells_vis_array` | 3D voxel (octomap), MarkerArray로 표시 |
| 2D occupancy | `octomap_server` 투영 | `/projected_map` (nav_msgs/OccupancyGrid) | 2D occupancy |
| 2.5D elevation | `elevation_mapper` (이 패키지) | `/elevation_mapper/elevation_map` (grid_map_msgs/GridMap: elevation/min/count) | 2.5D elevation (기본 꺼짐) |

```bash
sudo apt install ros-jazzy-octomap-server ros-jazzy-octomap-rviz-plugins ros-jazzy-grid-map-ros ros-jazzy-grid-map-rviz-plugin
```

## bag으로 확인 (개발 PC)

```bash
ros2 launch luco3dnav_mapping layers.launch.py use_sensors:=false              # SLAM + 레이어 + RViz
ros2 bag play bags/<bag> --clock --topics /imu/data /velodyne_points  # 다른 터미널
```

- 레이어 하나만: `octomap:=false` 또는 `elevation:=false`
- SLAM은 이미 떠 있으면: `slam:=false use_sim_time:=true`
- 다시 쌓기: `ros2 service call /octomap_server/reset std_srvs/srv/Empty`, `ros2 service call /elevation_mapper/reset std_srvs/srv/Empty`

## 로봇 실시간 (계산은 로봇 PC, 개발 PC는 RViz만)

두 PC 모두 luco 터미널에서 `source .../luco3dnav_bringup/scripts/luco_net.sh` 먼저 (로봇 PC는 `sb_luco`도).

```bash
# 로봇 PC: luco_bringup 실행 중일 때 (mapping.launch.py 대신 이걸로. SLAM + 레이어)
ros2 launch luco3dnav_mapping layers.launch.py wheel_odom:=true use_description:=false use_static_tf:=false use_rviz:=false
# 로봇 PC: bringup 없이 센서만
ros2 launch luco3dnav_mapping layers.launch.py use_rviz:=false
# 개발 PC: 보기만
ros2 launch luco3dnav_mapping layers.launch.py slam:=false octomap:=false elevation:=false
```

로봇 PC에도 octomap_server / grid_map 패키지 설치 필요 (위 apt 명령).

## 지도 저장

SLAM(PGO)이 돌고 있는 PC에서 실행 (로봇에서 SLAM 중이면 로봇 PC에서, luco 환경 source 후).

```bash
ros2 run luco3dnav_mapping save_maps                   # 이름 luco_<월일_시분>
ros2 run luco3dnav_mapping save_maps my_map --tar      # my_map.tar.gz 한 파일로도
ros2 run luco3dnav_mapping save_maps my_map --rebuild-layers   # 레이어를 PGO 최종 자세 + 정적 점으로 다시 만듦
ros2 run luco3dnav_mapping save_maps my_map --keep-dynamic     # 사람 등 제거 안 함
```

`maps/pcd/<이름>/` (로봇 PC는 clone 안 `maps/pcd/`)에 저장. 같은 이름이면 덮어씀.

| 파일 | 내용 |
|---|---|
| `map.pcd` | PGO 3D 점군 (localizer용). 움직인 물체 지운 정적 지도 |
| `map_raw.pcd`, `map_dynamic.pcd` | 지우기 전 원본 / 지운 점 |
| `poses.txt`, `patches/` | 키프레임 자세 + 점군 (후처리에 필요, `--no-patches`면 생략되고 후처리도 안 됨) |
| `keyframes.txt`, `loops.txt` | 키프레임 시각 + LIO/보정 자세, 루프 쌍 (분석용) |
| `octomap.bt` | 3D voxel |
| `grid.pgm`, `grid.yaml` | 2D occupancy (Nav2 map_server 형식) |
| `elevation.npz` | 2.5D elevation 원본 (elevation/min/count, NaN = 없음) |
| `elevation.pgm`, `elevation.yaml` | elevation 미리보기 (밝을수록 높음) |
| `layers_old/` | `rebuild_layers` 전 레이어 |
| `info.yaml` | 저장 시각, 항목별 성공 여부 |

`map_release.sh upload <이름>`은 이 폴더를 tar.gz 하나로 올리므로 위 파일이 모두 같이 공유됨.

## 후처리 도구 (저장된 지도 폴더 대상, SLAM 없이 실행)

```bash
ros2 run luco3dnav_mapping remove_dynamic maps/pcd/<이름> --replace   # 사람 등 지우기 (save_maps가 자동으로 함)
ros2 run luco3dnav_mapping rebuild_layers maps/pcd/<이름>             # octomap/2D/elevation 다시 만들기
luco3dnav_bringup/scripts/bag_to_map.sh bags/<bag> [이름]            # bag -> SLAM -> 저장 -> 위 두 개까지 한 번에
```

- **remove_dynamic**: 지도 점 자리를 나중 스캔의 광선이 뚫고 지나갔으면 그때 비어 있었던 것 → 움직인 물체로 판정.
  바닥(+0.15 m 아래)과 머리 위(+2.2 m 위)는 판정 안 함. 그 자리를 나중에 다시 보지 못했으면 근거가 없어 남는다.
- **rebuild_layers**: 키프레임을 PGO 최종(루프 보정 후) 자세로, 정적 점만, 라이다 위치에서 raycast해 다시 쌓는다.
  자체 ROS 도메인(87)에서 octomap_server / elevation_mapper를 띄우므로 로봇 SLAM이 돌고 있어도 섞이지 않음.
  입력이 키프레임(0.5 m / 10° 간격)뿐이라 elevation 빈 칸은 실시간보다 조금 많다.

## 설정

- `config/octomap.yaml`: `resolution`, `occupancy_min_z/max_z`(2D에 장애물로 넣을 높이, map z 기준. z=0 = 시작 위치 바닥)
- `config/elevation.yaml`: `resolution`, `length_x/y`(map 원점 중심 고정 크기), `max_height_above_robot`(천장 제외)

## 한계

- 실시간 레이어는 점을 넣는 순간의 PGO 보정만 반영 (루프가 닫혀도 이미 쌓인 voxel은 안 고쳐짐), 사람도 그대로 쌓임
  → 저장할 때 `--rebuild-layers` 또는 나중에 `rebuild_layers`
- 실시간 octomap raycast 원점은 base_footprint(바닥). 라이다보다 0.37 m 아래라 free 공간이 약간 다름 (rebuild는 라이다 위치)
- `incremental_2D_projection: true`면 jazzy octomap_server에서 projected_map이 전부 unknown으로 나와서 false로 둠
- RViz voxel은 octomap_rviz_plugins 대신 기본 MarkerArray(`/occupied_cells_vis_array`)로 표시.
  apt의 octomap_rviz_plugins는 ros-jazzy-octomap(1.10) 기준 빌드라 시스템 liboctomap 1.9.7만 있으면
  `undefined symbol: typeinfo for octomap::OcTreeStamped`로 로드 실패 (`sudo apt install ros-jazzy-octomap`로 해결 가능)
