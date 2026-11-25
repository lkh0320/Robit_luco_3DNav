# luco3dnav

Luco 로봇용 FAST-LIO2 기반 3D 맵핑/내비게이션 패키지 모음 (ROS 2 Jazzy, Ubuntu 24.04).

| 패키지 | 역할 |
|---|---|
| `third_party/FASTLIO2_ROS2` | [beomsuchoi/FASTLIO2_ROS2](https://github.com/beomsuchoi/FASTLIO2_ROS2) 복사본. fastlio2 CMake에 `Sophus::Sophus` 한 줄 추가, `hba`는 빌드 제외 |
| `luco3dnav_sensors` | EBIMU 시리얼 → `/imu/data`(sensor_msgs/Imu), Velodyne VLP-16 → `/velodyne_points` |
| `luco3dnav_bringup` | 우리 `lio.yaml`/`pgo.yaml`, `mapping.launch.py` |
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

지도 공유 (GitHub Releases, 지도 하나당 릴리스 `map-<이름>`):

```bash
src/luco3dnav/luco3dnav_bringup/scripts/map_release.sh upload luco_1125_1954 "설명"   # maps/pcd/<이름> 올리기
src/luco3dnav/luco3dnav_bringup/scripts/map_release.sh list                            # 올라간 지도 목록
src/luco3dnav/luco3dnav_bringup/scripts/map_release.sh download luco_1125_1954         # maps/pcd/<이름> 로 받기
```

`gh auth login` 필요. 레포가 공개라 올린 지도도 공개됨.

## 주의

- `lio.yaml`의 `r_bi`/`t_bi`(base_link→IMU), `r_il`/`t_il`(IMU→라이다)은 luco2 실측값. IMU는 로봇 대비 z축 -90도 장착. 바꿀 때는 description 레포(`luco2_description`) URDF와 `mapping.launch.py` 정적 TF도 같이 맞출 것.
- 이 FAST-LIO2 포크는 점별 시간 정보를 쓰지 않아 움직임 왜곡 보정이 없음.
