"""2.5D elevation map: FAST-LIO body_cloud + TF(map <- base_footprint, PGO 보정 포함) -> grid_map_msgs/GridMap.

셀마다 들어온 점의 최고 높이(elevation), 최저 높이(min), 점 개수(count)를 누적한다.
로봇 바닥 기준 max_height_above_robot 보다 높은 점(천장 등)은 버린다.
넣는 순간의 TF만 쓰므로 PGO 루프 보정 후에도 이미 쌓인 셀은 다시 정렬되지 않는다.

grid_map 저장 규칙: index (i, j), i = x 방향(큰 x가 0), j = y 방향(큰 y가 0),
  데이터는 Eigen column-major (flat = i + j * rows).
"""
from array import array
from collections import deque

import numpy as np
import rclpy
from grid_map_msgs.msg import GridMap
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener


def _transform_to_matrix(tf):
    q = tf.transform.rotation
    t = tf.transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    r = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    return r, np.array([t.x, t.y, t.z])


class ElevationMapper(Node):
    def __init__(self):
        super().__init__('elevation_mapper')
        p = self.declare_parameter
        self.cloud_topic = p('cloud_topic', '/fastlio2/body_cloud').value
        self.map_frame = p('map_frame', 'map').value
        self.robot_frame = p('robot_frame', 'base_footprint').value
        self.res = float(p('resolution', 0.1).value)
        self.len_x = float(p('length_x', 40.0).value)
        self.len_y = float(p('length_y', 40.0).value)
        self.max_range = float(p('max_range', 15.0).value)
        self.max_h = float(p('max_height_above_robot', 1.2).value)
        self.min_h = float(p('min_height_above_robot', -1.5).value)
        self.min_points = int(p('min_points', 2).value)
        rate = float(p('publish_rate', 2.0).value)
        self.tf_wait = float(p('tf_wait', 1.0).value)  # 이 시간 안에 TF가 안 오면 클라우드 버림

        self.rows = int(round(self.len_x / self.res))
        self.cols = int(round(self.len_y / self.res))
        # 실제 길이는 셀 개수에 맞춤
        self.len_x = self.rows * self.res
        self.len_y = self.cols * self.res
        self._reset()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.sub = self.create_subscription(PointCloud2, self.cloud_topic, self._on_cloud, qos_profile_sensor_data)
        # latched(transient_local): bag이 끝나거나 SLAM이 멈춘 뒤에도 마지막 맵을 save_maps / RViz가 받을 수 있게
        self.pub = self.create_publisher(GridMap, '~/elevation_map', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Empty, '~/reset', self._on_reset)
        # 벽시계 타이머: use_sim_time에서 /clock이 멈춰도(bag 끝) 마지막 변경분을 발행
        wall = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / rate, self._publish, clock=wall)
        self.published_clouds = -1
        self.queue = deque(maxlen=50)
        self.create_timer(0.02, self._process_queue, clock=wall)
        self.n_dropped = 0
        self.last_stamp = None
        self.n_clouds = 0
        self.get_logger().info(
            f'{self.cloud_topic} -> ~/elevation_map  {self.rows}x{self.cols} cells @ {self.res} m')

    def _reset(self):
        n = (self.rows, self.cols)
        self.elev = np.full(n, -np.inf, dtype=np.float32)
        self.zmin = np.full(n, np.inf, dtype=np.float32)
        self.count = np.zeros(n, dtype=np.float32)

    def _on_reset(self, req, res):
        self._reset()
        self.get_logger().info('elevation map reset')
        return res

    def _on_cloud(self, msg):
        # TF(map <- cloud)는 PGO가 같은 시각으로 조금 뒤에 발행하므로 여기서 기다리지 않고 큐에 넣어둠
        self.queue.append(msg)

    def _process_queue(self):
        while self.queue:
            msg = self.queue[0]
            stamp = Time.from_msg(msg.header.stamp)
            if self.tf_buffer.can_transform(self.map_frame, msg.header.frame_id, stamp) and \
                    self.tf_buffer.can_transform(self.map_frame, self.robot_frame, stamp):
                self.queue.popleft()
                self._integrate(msg, stamp)
                continue
            age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
            if age > self.tf_wait:
                self.queue.popleft()
                self.n_dropped += 1
                self.get_logger().warn(f'TF 없음 {age:.2f}s, 클라우드 버림 (누적 {self.n_dropped})',
                                       throttle_duration_sec=2.0)
                continue
            break

    def _integrate(self, msg, stamp):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, msg.header.frame_id, stamp)
            tf_robot = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, stamp)
        except TransformException as e:
            self.get_logger().warn(f'TF 없음: {e}', throttle_duration_sec=2.0)
            return
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.size == 0:
            return
        r, t = _transform_to_matrix(tf)
        w = pts.astype(np.float64) @ r.T + t
        robot = np.array([tf_robot.transform.translation.x, tf_robot.transform.translation.y,
                          tf_robot.transform.translation.z])

        dz = w[:, 2] - robot[2]
        d2 = (w[:, 0] - robot[0]) ** 2 + (w[:, 1] - robot[1]) ** 2
        keep = (dz <= self.max_h) & (dz >= self.min_h) & (d2 <= self.max_range ** 2)
        w = w[keep]
        # map 원점 중심 그리드. i: x 큰 쪽이 0, j: y 큰 쪽이 0
        i = np.floor((self.len_x / 2 - w[:, 0]) / self.res).astype(np.int64)
        j = np.floor((self.len_y / 2 - w[:, 1]) / self.res).astype(np.int64)
        inside = (i >= 0) & (i < self.rows) & (j >= 0) & (j < self.cols)
        i, j, z = i[inside], j[inside], w[inside, 2].astype(np.float32)
        np.maximum.at(self.elev, (i, j), z)
        np.minimum.at(self.zmin, (i, j), z)
        np.add.at(self.count, (i, j), 1.0)
        self.last_stamp = msg.header.stamp
        self.n_clouds += 1

    def _layer(self, arr):
        m = Float32MultiArray()
        m.layout.dim = [
            MultiArrayDimension(label='column_index', size=self.cols, stride=self.rows * self.cols),
            MultiArrayDimension(label='row_index', size=self.rows, stride=self.rows),
        ]
        m.data = array('f', np.asarray(arr, dtype=np.float32).ravel(order='F').tobytes())
        return m

    def _publish(self):
        if self.last_stamp is None:
            return
        valid = self.count >= self.min_points
        self.get_logger().info(f'클라우드 {self.n_clouds}개 통합, 유효 셀 {int(valid.sum())}, '
                               f'대기 {len(self.queue)}, 버림 {self.n_dropped}', throttle_duration_sec=10.0)
        if self.n_clouds == self.published_clouds:   # 바뀐 게 없으면 다시 안 보냄 (latched라 마지막 것 유지)
            return
        self.published_clouds = self.n_clouds
        nan = np.float32(np.nan)
        msg = GridMap()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.last_stamp
        msg.info.resolution = self.res
        msg.info.length_x = self.len_x
        msg.info.length_y = self.len_y
        msg.info.pose.orientation.w = 1.0
        msg.layers = ['elevation', 'min', 'count']
        msg.basic_layers = ['elevation']
        msg.data = [
            self._layer(np.where(valid, self.elev, nan)),
            self._layer(np.where(valid, self.zmin, nan)),
            self._layer(np.where(valid, self.count, nan)),
        ]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ElevationMapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
