# luco3dnav_navigation (skeleton)
아직 ROS 패키지 아님 (package.xml 없음 → colcon이 무시함). 3D Nav 구성용 자리.
- config/  : localizer / 센서 관련 설정
- params/  : nav2 파라미터 (costmap, planner, controller)
- launch/  : bringup (localizer + nav2 + map_server)
- rviz/    : rviz 설정
- scripts/ : pcd → 2D grid 변환 등 유틸
