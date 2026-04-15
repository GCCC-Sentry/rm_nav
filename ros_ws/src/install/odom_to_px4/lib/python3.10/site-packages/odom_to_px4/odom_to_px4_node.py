"""
odom_to_px4_node.py

ROS 2 节点：订阅 nav_msgs/Odometry (ENU/FLU)，
转换坐标系后发布 px4_msgs/VehicleOdometry (NED/FRD) 到 PX4 v1.15.4。

通信方式：uXRCE-DDS（PX4 官方 ROS 2 接口）
发布话题：/fmu/in/vehicle_visual_odometry

坐标系转换说明：
  ROS 2:  位置 ENU (East-North-Up),  机体 FLU (Forward-Left-Up)
  PX4:    位置 NED (North-East-Down), 机体 FRD (Forward-Right-Down)

  位置:      x_ned =  y_enu,  y_ned =  x_enu,  z_ned = -z_enu
  四元数:    q_ned_frd = q_ned_enu ⊗ q_enu_flu ⊗ q_flu_frd
  线速度(体): vx_frd =  vx_flu,  vy_frd = -vy_flu,  vz_frd = -vz_flu
  角速度(体): wx_frd =  wx_flu,  wy_frd = -wy_flu,  wz_frd = -wz_flu
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry, TimesyncStatus


# ---------------------------------------------------------------------------
#  常量：坐标系旋转四元数 [w, x, y, z]
# ---------------------------------------------------------------------------
_SQRT2_2 = math.sqrt(2.0) / 2.0

# ENU → NED  (Rz(π/2) · Rx(π))
Q_ENU_TO_NED = (0.0, _SQRT2_2, _SQRT2_2, 0.0)
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

    q_ned_frd = q_ned_enu ⊗ q_enu_flu ⊗ q_flu_frd
    """
    q_enu_flu = (qw, qx, qy, qz)   # 转换为 [w,x,y,z]
    tmp = _quat_multiply(Q_NED_TO_ENU, q_enu_flu)
    q_ned_frd = _quat_multiply(tmp, Q_FLU_TO_FRD)
    return q_ned_frd  # (w, x, y, z)


# ---------------------------------------------------------------------------
#  节点
# ---------------------------------------------------------------------------
class OdomToPx4Node(Node):
    """订阅 nav_msgs/Odometry → 发布 px4_msgs/VehicleOdometry"""

    def __init__(self):
        super().__init__('odom_to_px4_node')

        # ---- 参数 ----
        self.declare_parameter('odom_topic', 'odometry')
        self.declare_parameter('px4_odom_topic', '/fmu/in/vehicle_visual_odometry')
        # 是否使用 PX4 时间同步（推荐开启）
        self.declare_parameter('use_timesync', True)
        # 是否转换坐标系（如果里程计已经是 NED/FRD 则关闭）
        self.declare_parameter('convert_enu_to_ned', True)

        odom_topic = self.get_parameter('odom_topic').value
        px4_topic = self.get_parameter('px4_odom_topic').value
        self._use_timesync = self.get_parameter('use_timesync').value
        self._convert = self.get_parameter('convert_enu_to_ned').value

        # ---- PX4 QoS (uXRCE-DDS 推荐) ----
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---- 时间同步 ----
        self._timestamp_offset_us = 0  # companion - px4 (微秒)
        self._timesync_ok = False
        if self._use_timesync:
            self.create_subscription(
                TimesyncStatus,
                '/fmu/out/timesync_status',
                self._timesync_cb,
                px4_qos,
            )
            self.get_logger().info('等待 PX4 时间同步 (/fmu/out/timesync_status)...')

        # ---- 订阅里程计 ----
        self._sub_odom = self.create_subscription(
            Odometry,
            odom_topic,
            self._odom_cb,
            10,
        )

        # ---- 发布 PX4 里程计 ----
        self._pub_px4_odom = self.create_publisher(
            VehicleOdometry,
            px4_topic,
            px4_qos,
        )

        self.get_logger().info(
            f'OdomToPx4 已启动: {odom_topic} -> {px4_topic}  '
            f'ENU→NED={self._convert}  timesync={self._use_timesync}'
        )

    # ------------------------------------------------------------------
    #  回调
    # ------------------------------------------------------------------
    def _timesync_cb(self, msg: TimesyncStatus):
        """更新与 PX4 的时间偏移"""
        self._timestamp_offset_us = msg.estimated_offset
        if not self._timesync_ok:
            self._timesync_ok = True
            self.get_logger().info(
                f'PX4 时间同步已建立, offset={self._timestamp_offset_us} μs'
            )

    def _get_px4_timestamp_us(self, ros_stamp):
        """
        将 ROS 时间戳转换为 PX4 微秒时间戳。
        如果未启用 timesync 或尚未同步，返回 0（PX4 将使用接收时间）。
        """
        if not self._use_timesync or not self._timesync_ok:
            return 0
        ros_us = int(ros_stamp.sec * 1_000_000) + int(ros_stamp.nanosec / 1000)
        return ros_us + self._timestamp_offset_us

    def _odom_cb(self, odom: Odometry):
        """接收 ROS Odometry，转换并发布 PX4 VehicleOdometry"""
        msg = VehicleOdometry()

        # ---------- 时间戳 ----------
        stamp_us = self._get_px4_timestamp_us(odom.header.stamp)
        msg.timestamp = stamp_us
        msg.timestamp_sample = stamp_us

        # ---------- 位置 ----------
        px = odom.pose.pose.position.x
        py = odom.pose.pose.position.y
        pz = odom.pose.pose.position.z

        if self._convert:
            # ENU → NED
            msg.position[0] = float(py)      # North
            msg.position[1] = float(px)      # East
            msg.position[2] = float(-pz)     # Down
        else:
            msg.position[0] = float(px)
            msg.position[1] = float(py)
            msg.position[2] = float(pz)

        msg.pose_frame = VehicleOdometry.POSE_FRAME_NED

        # ---------- 四元数 ----------
        qx = odom.pose.pose.orientation.x
        qy = odom.pose.pose.orientation.y
        qz = odom.pose.pose.orientation.z
        qw = odom.pose.pose.orientation.w

        if self._convert:
            w, x, y, z = ros_to_px4_orientation(qx, qy, qz, qw)
        else:
            w, x, y, z = qw, qx, qy, qz

        msg.q[0] = float(w)
        msg.q[1] = float(x)
        msg.q[2] = float(y)
        msg.q[3] = float(z)

        # ---------- 线速度 (body frame) ----------
        vx = odom.twist.twist.linear.x
        vy = odom.twist.twist.linear.y
        vz = odom.twist.twist.linear.z

        if self._convert:
            # FLU → FRD
            msg.velocity[0] = float(vx)
            msg.velocity[1] = float(-vy)
            msg.velocity[2] = float(-vz)
        else:
            msg.velocity[0] = float(vx)
            msg.velocity[1] = float(vy)
            msg.velocity[2] = float(vz)

        msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_BODY_FRD

        # ---------- 角速度 (body frame) ----------
        wx = odom.twist.twist.angular.x
        wy = odom.twist.twist.angular.y
        wz = odom.twist.twist.angular.z

        if self._convert:
            # FLU → FRD
            msg.angular_velocity[0] = float(wx)
            msg.angular_velocity[1] = float(-wy)
            msg.angular_velocity[2] = float(-wz)
        else:
            msg.angular_velocity[0] = float(wx)
            msg.angular_velocity[1] = float(wy)
            msg.angular_velocity[2] = float(wz)

        # ---------- 协方差 → 方差 ----------
        # nav_msgs/Odometry 的 covariance 是 6x6 行主序
        # 取对角线元素 [0,7,14] 为 x,y,z 方差
        pcov = odom.pose.covariance
        if pcov[0] > 0.0:
            if self._convert:
                msg.position_variance[0] = float(pcov[7])   # y_enu → x_ned
                msg.position_variance[1] = float(pcov[0])   # x_enu → y_ned
                msg.position_variance[2] = float(pcov[14])  # z_enu → z_ned
            else:
                msg.position_variance[0] = float(pcov[0])
                msg.position_variance[1] = float(pcov[7])
                msg.position_variance[2] = float(pcov[14])

            # 姿态方差 (roll, pitch, yaw → 对应 [21, 28, 35])
            if self._convert:
                msg.orientation_variance[0] = float(pcov[28])
                msg.orientation_variance[1] = float(pcov[21])
                msg.orientation_variance[2] = float(pcov[35])
            else:
                msg.orientation_variance[0] = float(pcov[21])
                msg.orientation_variance[1] = float(pcov[28])
                msg.orientation_variance[2] = float(pcov[35])
        else:
            # 协方差未知 → 填 NaN
            nan = float('nan')
            msg.position_variance[0] = nan
            msg.position_variance[1] = nan
            msg.position_variance[2] = nan
            msg.orientation_variance[0] = nan
            msg.orientation_variance[1] = nan
            msg.orientation_variance[2] = nan

        # 速度协方差
        tcov = odom.twist.covariance
        if tcov[0] > 0.0:
            if self._convert:
                msg.velocity_variance[0] = float(tcov[0])
                msg.velocity_variance[1] = float(tcov[7])
                msg.velocity_variance[2] = float(tcov[14])
            else:
                msg.velocity_variance[0] = float(tcov[0])
                msg.velocity_variance[1] = float(tcov[7])
                msg.velocity_variance[2] = float(tcov[14])
        else:
            nan = float('nan')
            msg.velocity_variance[0] = nan
            msg.velocity_variance[1] = nan
            msg.velocity_variance[2] = nan

        msg.reset_counter = 0
        msg.quality = 0

        # ---------- 发布 ----------
        self._pub_px4_odom.publish(msg)


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
