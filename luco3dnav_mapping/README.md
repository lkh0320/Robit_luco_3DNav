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
ros2 bag play bags/luco_1120_0133 --clock --topics /imu/data /velodyne_points  # 다른 터미널
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

## 설정

- `config/octomap.yaml`: `resolution`, `occupancy_min_z/max_z`(2D에 장애물로 넣을 높이, map z 기준. z=0 = 시작 위치 바닥)
- `config/elevation.yaml`: `resolution`, `length_x/y`(map 원점 중심 고정 크기), `max_height_above_robot`(천장 제외)

## 한계

- 점을 넣는 순간의 PGO 보정만 반영. 루프가 닫혀 map->lio_world가 바뀌어도 이미 쌓인 voxel/셀은 다시 정렬되지 않음
  (PGO 튜닝 효과는 "루프 닫힌 뒤 새로 쌓이는 부분이 기존과 겹치는지"로 봐야 함)
- octomap raycast 원점은 base_footprint(바닥). 라이다보다 0.37 m 아래라 free 공간이 약간 다름
- `incremental_2D_projection: true`면 jazzy octomap_server에서 projected_map이 전부 unknown으로 나와서 false로 둠
- RViz voxel은 octomap_rviz_plugins 대신 기본 MarkerArray(`/occupied_cells_vis_array`)로 표시.
  apt의 octomap_rviz_plugins는 ros-jazzy-octomap(1.10) 기준 빌드라 시스템 liboctomap 1.9.7만 있으면
  `undefined symbol: typeinfo for octomap::OcTreeStamped`로 로드 실패 (`sudo apt install ros-jazzy-octomap`로 해결 가능)
