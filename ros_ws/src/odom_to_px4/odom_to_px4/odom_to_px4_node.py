"""
odom_to_px4_node.py

ROS 2 节点：订阅 nav_msgs/Odometry (ENU/FLU)，
通过 MAVLink 串口发送视觉里程计给 PX4 v1.15.4。

发送消息：
  - VISION_POSITION_ESTIMATE (#102) — 位置 + 姿态 (NED 欧拉角)
  - VISION_SPEED_ESTIMATE    (#103) — 速度 (NED)

这两个是 PX4 最成熟、兼容性最好的外部视觉定位输入方式。
PX4 收到后会映射到 vehicle_visual_odometry uORB 话题。

通信方式：pymavlink → 串口 → PX4 飞控 TELEM 端口
无需 uXRCE-DDS Agent。

坐标系转换说明：
  ROS 2:  位置 ENU (East-North-Up),  机体 FLU (Forward-Left-Up)
  PX4:    位置 NED (North-East-Down), 机体 FRD (Forward-Right-Down)

  位置:      x_ned =  y_enu,  y_ned =  x_enu,  z_ned = -z_enu
  欧拉角:    roll_ned = roll_enu, pitch_ned = -pitch_enu, yaw_ned = pi/2 - yaw_enu

PX4 EKF2 参数配置：
  EKF2_EV_CTRL  = 15    (位置+速度+偏航融合)
  EKF2_HGT_REF  = 3     (视觉高度)
  EKF2_EV_DELAY = 0~50  (视觉延迟 ms)
"""

import os
os.environ['MAVLINK20'] = '1'  # 必须在 import pymavlink 之前设置

import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

# pymavlink (MAVLink 2.0)
from pymavlink import mavutil


# ---------------------------------------------------------------------------
#  常量：坐标系旋转四元数 [w, x, y, z]
# ---------------------------------------------------------------------------
_SQRT2_2 = math.sqrt(2.0) / 2.0

# NED → ENU  (共轭)
Q_NED_TO_ENU = (0.0, -_SQRT2_2, -_SQRT2_2, 0.0)
# FLU → FRD  (Rx(π), 即绕 X 转 180°)
Q_FLU_TO_FRD = (0.0, 1.0, 0.0, 0.0)


# ---------------------------------------------------------------------------
#  四元数工具
# ---------------------------------------------------------------------------
def _quat_multiply(q1, q2):
    """Hamilton 乘法  [w, x, y, z]"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def ros_to_px4_orientation(qx, qy, qz, qw):
    """
    将 ROS 四元数 (ENU→FLU, geometry_msgs 格式 x,y,z,w)
    转换为 PX4 四元数 (NED→FRD, [w,x,y,z])
    """
    q_enu_flu = (qw, qx, qy, qz)
    tmp = _quat_multiply(Q_NED_TO_ENU, q_enu_flu)
    q_ned_frd = _quat_multiply(tmp, Q_FLU_TO_FRD)
    return q_ned_frd  # (w, x, y, z)


def _quat_to_euler(w, x, y, z):
    """四元数 → 欧拉角 (roll, pitch, yaw) 弧度"""
    # roll (x-axis)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def _rotate_velocity_body_to_ned(qw, qx, qy, qz, vx, vy, vz):
    """用四元数将机体速度旋转到 NED 全局系"""
    # q ⊗ v ⊗ q*
    # 简化版：直接用旋转矩阵
    q0, q1, q2, q3 = qw, qx, qy, qz
    r00 = 1 - 2*(q2*q2 + q3*q3)
    r01 = 2*(q1*q2 - q0*q3)
    r02 = 2*(q1*q3 + q0*q2)
    r10 = 2*(q1*q2 + q0*q3)
    r11 = 1 - 2*(q1*q1 + q3*q3)
    r12 = 2*(q2*q3 - q0*q1)
    r20 = 2*(q1*q3 - q0*q2)
    r21 = 2*(q2*q3 + q0*q1)
    r22 = 1 - 2*(q1*q1 + q2*q2)

    vx_ned = r00*vx + r01*vy + r02*vz
    vy_ned = r10*vx + r11*vy + r12*vz
    vz_ned = r20*vx + r21*vy + r22*vz
    return vx_ned, vy_ned, vz_ned


# ---------------------------------------------------------------------------
#  节点
# ---------------------------------------------------------------------------
class OdomToPx4Node(Node):
    """订阅 nav_msgs/Odometry → MAVLink 串口发送视觉位置/速度到 PX4"""

    def __init__(self):
        super().__init__('odom_to_px4_node')

        # ---- 参数 ----
        self.declare_parameter('odom_topic', 'odometry')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 921600)
        self.declare_parameter('convert_enu_to_ned', True)
        self.declare_parameter('debug_print_rate', 1.0)
        # MAVLink system/component ID
        # sys_id: 不能与飞控相同 (飞控默认 1)
        #   2   = 机载电脑 (推荐)
        #   255 = GCS 地面站
        # comp_id: 197 = MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
        # tgt_system: VISION_POSITION_ESTIMATE 是广播消息, 无需设置目标系统
        self.declare_parameter('mav_sys_id', 2)
        self.declare_parameter('mav_comp_id', 197)  # MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY

        odom_topic = self.get_parameter('odom_topic').value
        serial_port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        self._convert = self.get_parameter('convert_enu_to_ned').value
        self._debug_rate = self.get_parameter('debug_print_rate').value
        sys_id = self.get_parameter('mav_sys_id').value
        comp_id = self.get_parameter('mav_comp_id').value

        # ---- 调试统计 ----
        self._odom_recv_count = 0
        self._mav_send_count = 0
        self._mav_error_count = 0
        self._last_debug_time = time.monotonic()
        self._last_sent_data = None

        # ---- 打开 MAVLink 串口连接 ----
        self.get_logger().info(f'正在打开串口 {serial_port} @ {baudrate} baud ...')
        try:
            self._mav = mavutil.mavlink_connection(
                serial_port,
                baud=baudrate,
                source_system=sys_id,
                source_component=comp_id,
            )
            self.get_logger().info(f'串口已打开: {serial_port} @ {baudrate}')
        except Exception as e:
            self.get_logger().fatal(f'无法打开串口 {serial_port}: {e}')
            raise

        # ---- 订阅里程计 ----
        self._sub_odom = self.create_subscription(
            Odometry,
            odom_topic,
            self._odom_cb,
            10,
        )

        # ---- 调试定时器 ----
        if self._debug_rate > 0.0:
            self._debug_timer = self.create_timer(
                1.0 / self._debug_rate, self._debug_print_cb
            )

        self.get_logger().info(
            f'OdomToPx4 (MAVLink) 已启动:\n'
            f'  订阅话题:   {odom_topic}\n'
            f'  串口设备:   {serial_port} @ {baudrate}\n'
            f'  ENU→NED:    {self._convert}\n'
            f'  MAVLink:    sys={sys_id} comp={comp_id}\n'
            f'  调试频率:   {self._debug_rate} Hz\n'
            f'  发送消息:   VISION_POSITION_ESTIMATE (#102)\n'
            f'              VISION_SPEED_ESTIMATE (#103)\n'
            f'  ─────────────────────────────────────\n'
            f'  QGC 验证方法:\n'
            f'    MAVLink Inspector → 查看 #102 #103\n'
            f'    MAVLink Console   → listener vehicle_visual_odometry'
        )

    # ------------------------------------------------------------------
    #  调试打印
    # ------------------------------------------------------------------
    def _debug_print_cb(self):
        """定时输出调试信息"""
        now = time.monotonic()
        dt = now - self._last_debug_time
        if dt < 0.01:
            return

        odom_hz = self._odom_recv_count / dt if dt > 0 else 0.0
        send_hz = self._mav_send_count / dt if dt > 0 else 0.0

        self.get_logger().info(
            f'[调试] 收到odom: {self._odom_recv_count}条 ({odom_hz:.1f}Hz) | '
            f'MAVLink发送: {self._mav_send_count}条 ({send_hz:.1f}Hz) | '
            f'发送错误: {self._mav_error_count}'
        )

        if self._last_sent_data is not None:
            d = self._last_sent_data
            self.get_logger().info(
                f'[调试] VISION_POSITION_ESTIMATE (#102):\n'
                f'  time_usec = {d["time_usec"]} μs\n'
                f'  position  = [{d["x"]:.4f}, {d["y"]:.4f}, {d["z"]:.4f}] (NED, m)\n'
                f'  euler     = [R:{math.degrees(d["roll"]):.2f}° P:{math.degrees(d["pitch"]):.2f}° Y:{math.degrees(d["yaw"]):.2f}°]\n'
                f'  velocity  = [{d["vx"]:.4f}, {d["vy"]:.4f}, {d["vz"]:.4f}] (NED, m/s)\n'
                f'  串口字节  = {d.get("bytes_sent", "?")} bytes'
            )
        elif self._odom_recv_count == 0:
            self.get_logger().warn('[调试] 尚未收到任何 odom 数据！请检查话题是否正确')

        self._odom_recv_count = 0
        self._mav_send_count = 0
        self._last_debug_time = now

    # ------------------------------------------------------------------
    #  里程计回调
    # ------------------------------------------------------------------
    def _odom_cb(self, odom: Odometry):
        """接收 ROS Odometry，转换坐标系后通过 MAVLink 串口发送"""
        self._odom_recv_count += 1

        # ---------- 时间戳 ----------
        time_usec = int(odom.header.stamp.sec * 1_000_000) + \
                    int(odom.header.stamp.nanosec / 1000)

        # ---------- 位置 ----------
        px = odom.pose.pose.position.x
        py = odom.pose.pose.position.y
        pz = odom.pose.pose.position.z

        if self._convert:
            # ENU → NED
            x_ned = float(py)       # North
            y_ned = float(px)       # East
            z_ned = float(-pz)      # Down
        else:
            x_ned = float(px)
            y_ned = float(py)
            z_ned = float(pz)

        # ---------- 四元数 → 欧拉角 ----------
        qx = odom.pose.pose.orientation.x
        qy = odom.pose.pose.orientation.y
        qz = odom.pose.pose.orientation.z
        qw = odom.pose.pose.orientation.w

        if self._convert:
            w, x, y, z = ros_to_px4_orientation(qx, qy, qz, qw)
        else:
            w, x, y, z = qw, qx, qy, qz

        roll, pitch, yaw = _quat_to_euler(w, x, y, z)

        # ---------- 速度 → NED 全局系 ----------
        vx_body = odom.twist.twist.linear.x
        vy_body = odom.twist.twist.linear.y
        vz_body = odom.twist.twist.linear.z

        if self._convert:
            # 先转 FLU → FRD
            vx_frd = float(vx_body)
            vy_frd = float(-vy_body)
            vz_frd = float(-vz_body)
            # 再用 NED→FRD 四元数旋转到 NED
            vx_ned, vy_ned, vz_ned = _rotate_velocity_body_to_ned(
                w, x, y, z, vx_frd, vy_frd, vz_frd
            )
        else:
            vx_ned = float(vx_body)
            vy_ned = float(vy_body)
            vz_ned = float(vz_body)

        # ---------- 协方差 ----------
        # VISION_POSITION_ESTIMATE 支持 covariance[21] (上三角 6x6)
        pose_cov = [float('nan')] + [0.0] * 20  # 默认: NaN 表示未知
        pcov = odom.pose.covariance
        if pcov[0] > 0.0:
            if self._convert:
                pose_cov[0] = float(pcov[7])    # y_enu → x_ned
                pose_cov[6] = float(pcov[0])    # x_enu → y_ned
                pose_cov[11] = float(pcov[14])  # z
                pose_cov[15] = float(pcov[28])  # pitch → roll
                pose_cov[18] = float(pcov[21])  # roll → pitch
                pose_cov[20] = float(pcov[35])  # yaw
            else:
                pose_cov[0] = float(pcov[0])
                pose_cov[6] = float(pcov[7])
                pose_cov[11] = float(pcov[14])
                pose_cov[15] = float(pcov[21])
                pose_cov[18] = float(pcov[28])
                pose_cov[20] = float(pcov[35])

        vel_cov = [float('nan')] + [0.0] * 20
        tcov = odom.twist.covariance
        if tcov[0] > 0.0:
            vel_cov[0] = float(tcov[0])
            vel_cov[6] = float(tcov[7])
            vel_cov[11] = float(tcov[14])

        # ---------- 发送 VISION_POSITION_ESTIMATE (#102) ----------
        try:
            self._mav.mav.vision_position_estimate_send(
                time_usec,              # usec
                x_ned,                  # x (NED, m)
                y_ned,                  # y (NED, m)
                z_ned,                  # z (NED, m)
                roll,                   # roll (rad)
                pitch,                  # pitch (rad)
                yaw,                    # yaw (rad)
                pose_cov,               # covariance [21] (MAVLink 2.0 ext)
                0,                      # reset_counter (MAVLink 2.0 ext)
            )

            # ---------- 发送 VISION_SPEED_ESTIMATE (#103) ----------
            self._mav.mav.vision_speed_estimate_send(
                time_usec,              # usec
                float(vx_ned),          # x speed (NED, m/s)
                float(vy_ned),          # y speed (NED, m/s)
                float(vz_ned),          # z speed (NED, m/s)
                vel_cov,                # covariance [9] (MAVLink 2.0 ext)
                0,                      # reset_counter (MAVLink 2.0 ext)
            )

            self._mav_send_count += 1

            # 保存调试数据
            self._last_sent_data = {
                'time_usec': time_usec,
                'x': x_ned, 'y': y_ned, 'z': z_ned,
                'roll': roll, 'pitch': pitch, 'yaw': yaw,
                'vx': vx_ned, 'vy': vy_ned, 'vz': vz_ned,
                'bytes_sent': self._mav.mav.total_bytes_sent,
            }

        except Exception as e:
            self._mav_error_count += 1
            if self._mav_error_count <= 10:
                self.get_logger().error(f'MAVLink 发送失败: {e}')

    def destroy_node(self):
        """关闭串口"""
        if hasattr(self, '_mav') and self._mav:
            try:
                self._mav.close()
                self.get_logger().info('MAVLink 串口已关闭')
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdomToPx4Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
