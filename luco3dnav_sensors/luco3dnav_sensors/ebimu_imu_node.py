"""EBIMU serial -> sensor_msgs/Imu (for FAST-LIO2).

Reads the EBIMU ASCII stream directly from the serial port and publishes
sensor_msgs/Imu on /imu/data.

EBIMU line format: "*f0,f1,f2,..." with fields in this order
(only the ones enabled on the sensor are present):
    orientation (euler: roll,pitch,yaw [deg]  |  quaternion: z,y,x,w)
    gyro  (x,y,z)   [deg/s by default]
    accel (x,y,z)   [g by default]
    ...anything after that (mag, temp, battery...) is ignored

FAST-LIO2 needs gyro + accel, and accel MUST include gravity
(local/sensor-frame acceleration, not gravity-removed).
Set the matching output mode on the sensor (via `init_commands` or the
E2BOX tool) and the parameters below to match what it actually sends.
"""
import math
import threading

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import Imu

G = 9.80665


def euler_deg_to_quat(roll, pitch, yaw):
    r, p, y = (math.radians(v) / 2.0 for v in (roll, pitch, yaw))
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return (
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )


class EbimuImuNode(Node):
    def __init__(self):
        super().__init__('ebimu_imu_node')
        self.port = self.declare_parameter('port', '/dev/ttyUSB0').value
        self.baud = int(self.declare_parameter('baudrate', 115200).value)
        self.frame_id = self.declare_parameter('frame_id', 'imu_link').value
        topic = self.declare_parameter('topic', '/imu/data').value
        # 'euler' | 'quaternion' | 'none'
        self.ori_fmt = self.declare_parameter('orientation_format', 'euler').value
        self.gyro_unit = self.declare_parameter('gyro_unit', 'deg').value      # 'deg' | 'rad'
        self.accel_unit = self.declare_parameter('accel_unit', 'g').value      # 'g' | 'mps2'
        # Raw commands sent once at startup, e.g. ['<sof1>', '<sog1>', '<soa1>']
        # Empty by default: the sensor's current settings are left untouched.
        self.init_commands = self.declare_parameter(
            'init_commands', rclpy.Parameter.Type.STRING_ARRAY).value or []
        self.print_raw = int(self.declare_parameter('print_raw_lines', 5).value)

        self.n_ori = {'euler': 3, 'quaternion': 4, 'none': 0}[self.ori_fmt]
        self.pub = self.create_publisher(Imu, topic, 200)
        self.bad_lines = 0

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1.0)
        except serial.SerialException as e:
            hint = ''
            if 'Permission denied' in str(e):
                hint = (' -> 포트 권한 없음. udev 규칙(luco3dnav_sensors/udev/99-ebimu.rules)을 설치하거나, '
                        'dialout 그룹 추가 후 재부팅하세요.')
            elif 'No such file' in str(e):
                hint = ' -> 포트가 없음. IMU USB 연결과 port 파라미터를 확인하세요.'
            self.get_logger().fatal(f'cannot open {self.port}: {e}{hint}')
            raise SystemExit(1)
        for cmd in (c for c in self.init_commands if c.strip()):
            self.ser.write(cmd.encode())
            self.get_logger().info(f'sent init command {cmd}')
        self.ser.reset_input_buffer()

        self.get_logger().info(
            f'{self.port}@{self.baud} -> {topic} (orientation={self.ori_fmt}, '
            f'gyro={self.gyro_unit}, accel={self.accel_unit})')
        self.running = True
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def loop(self):
        while self.running and rclpy.ok():
            raw = self.ser.readline()
            stamp = self.get_clock().now().to_msg()
            if not raw:
                continue
            line = raw.decode('ascii', errors='ignore').strip()
            if self.print_raw > 0:
                self.print_raw -= 1
                self.get_logger().info(f'raw: {line}')
            msg = self.parse(line)
            if msg is None:
                continue
            msg.header.stamp = stamp
            self.pub.publish(msg)

    def parse(self, line):
        if not line.startswith('*'):
            return None
        try:
            f = [float(v) for v in line[1:].split(',')]
        except ValueError:
            return None
        need = self.n_ori + 6
        if len(f) < need:
            self.bad_lines += 1
            if self.bad_lines % 200 == 1:
                self.get_logger().warn(
                    f'line has {len(f)} fields, need >= {need} '
                    f'(orientation {self.n_ori} + gyro 3 + accel 3). '
                    f'Enable gyro/accel output on the sensor. line="{line}"')
            return None

        msg = Imu()
        msg.header.frame_id = self.frame_id

        if self.ori_fmt == 'euler':
            x, y, z, w = euler_deg_to_quat(*f[0:3])
        elif self.ori_fmt == 'quaternion':
            z, y, x, w = f[0:4]  # EBIMU quaternion order is z,y,x,w
        if self.n_ori:
            msg.orientation.x, msg.orientation.y = x, y
            msg.orientation.z, msg.orientation.w = z, w
        else:
            msg.orientation_covariance[0] = -1.0

        gs = math.pi / 180.0 if self.gyro_unit == 'deg' else 1.0
        acs = G if self.accel_unit == 'g' else 1.0
        g = f[self.n_ori:self.n_ori + 3]
        a = f[self.n_ori + 3:self.n_ori + 6]
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (v * gs for v in g)
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = (v * acs for v in a)
        return msg

    def destroy_node(self):
        self.running = False
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EbimuImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
