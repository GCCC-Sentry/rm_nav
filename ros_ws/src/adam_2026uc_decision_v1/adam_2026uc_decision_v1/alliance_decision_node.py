#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ADAM 2026UC 哨兵决策节点 v1
============================
替代行为树的轻量级状态机决策。

状态:
    WAIT_FOR_GAME  - 等待比赛开始 (game_progress == 4)
    PATROL         - 在巡逻点1和巡逻点2之间往返, 间隔5秒
    SUPPLY         - 前往补给点回血 (HP < 120 触发)
    DEAD           - 阵亡, 停车等复活

话题 (与行为树完全兼容):
    订阅: referee/game_status, referee/robot_status
    发布: goal_pose, cmd_spin, cmd_vel, /cmd_stance
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped
from example_interfaces.msg import Float32
from std_msgs.msg import Float32 as StdFloat32

try:
    from pb_rm_interfaces.msg import GameStatus, RobotStatus
except ImportError:
    print("[FATAL] 无法导入 pb_rm_interfaces, 请确保已 source install/setup.bash")
    raise


# ═══════════════════════════════════════
#  状态枚举
# ═══════════════════════════════════════
class State:
    WAIT_FOR_GAME = "WAIT_FOR_GAME"
    PATROL = "PATROL"
    SUPPLY = "SUPPLY"
    DEAD = "DEAD"


class Stance:
    MOVE = 0
    ATTACK = 1
    DEFENSE = 2


STANCE_NAMES = {
    Stance.MOVE: "移动姿态",
    Stance.ATTACK: "进攻姿态",
    Stance.DEFENSE: "防御姿态",
}


# ═══════════════════════════════════════
#  坐标点 (来自 rmul_2026_patrol.xml)
# ═══════════════════════════════════════
PATROL_POINTS = [
    {"name": "巡逻点1", "x": -3.37, "y": -4.41, "yaw": 0.00},
    {"name": "巡逻点2", "x": 0.47, "y": 0.00, "yaw": 1.57},
]

SUPPLY_POINT = {"name": "补给点", "x": -0.83, "y": 0.05, "yaw": 0.00}


# ═══════════════════════════════════════
#  可视化工具
# ═══════════════════════════════════════
BOX_WIDTH = 60

def box_top():
    return "╔" + "═" * BOX_WIDTH + "╗"

def box_bot():
    return "╚" + "═" * BOX_WIDTH + "╝"

def box_line(text: str):
    """居中 / 左对齐的 box 行"""
    s = f"║  {text}"
    pad = BOX_WIDTH + 1 - _display_width(s)
    if pad < 1:
        pad = 1
    return s + " " * pad + "║"

def _display_width(s: str) -> int:
    """粗略计算含中文字符串的显示宽度"""
    w = 0
    for ch in s:
        if '\u4e00' <= ch <= '\u9fff' or '\u3000' <= ch <= '\u303f' or '\uff00' <= ch <= '\uffef':
            w += 2
        else:
            w += 1
    return w

def state_icon(state: str) -> str:
    icons = {
        State.WAIT_FOR_GAME: "⏳",
        State.PATROL:        "🔄",
        State.SUPPLY:        "💊",
        State.DEAD:          "💀",
    }
    return icons.get(state, "❓")


# ═══════════════════════════════════════
#  主节点
# ═══════════════════════════════════════
class AllianceDecisionNode(Node):

    def __init__(self):
        super().__init__('alliance_decision_node')

        # ---- 参数 ----
        self.declare_parameter('hp_supply_threshold', 120)
        self.declare_parameter('hp_full', 400)
        self.declare_parameter('supply_timeout_sec', 5.0)
        self.declare_parameter('patrol_interval_sec', 5.0)
        self.declare_parameter('spin_speed', 7.0)
        self.declare_parameter('loop_rate_hz', 10.0)
        self.declare_parameter('stance_hp_threshold', 201)
        self.declare_parameter('attack_stance_duration_sec', 170.0)
        self.declare_parameter('defense_stance_duration_sec', 8.0)

        self.HP_SUPPLY_THRESH = self.get_parameter('hp_supply_threshold').value
        self.HP_FULL = self.get_parameter('hp_full').value
        self.SUPPLY_TIMEOUT = self.get_parameter('supply_timeout_sec').value
        self.PATROL_INTERVAL = self.get_parameter('patrol_interval_sec').value
        self.SPIN_SPEED = self.get_parameter('spin_speed').value
        loop_hz = self.get_parameter('loop_rate_hz').value
        self.STANCE_HP_THRESH = self.get_parameter('stance_hp_threshold').value
        self.ATTACK_STANCE_DURATION = self.get_parameter('attack_stance_duration_sec').value
        self.DEFENSE_STANCE_DURATION = self.get_parameter('defense_stance_duration_sec').value

        # ---- 状态 ----
        self.state = State.WAIT_FOR_GAME
        self.prev_state = None               # 用于检测状态切换
        self.patrol_index = 0                # 当前巡逻点索引
        self.last_patrol_switch_time = None  # 上次切换巡逻点的时间
        self.supply_enter_time = None        # 进入补给状态的时间
        self.supply_goal_sent = False        # 是否已发送补给目标点
        self.spin_started = False            # 是否已开启小陀螺
        self.last_goal_sent = None           # 上一次发送的目标描述
        self.current_stance = None           # 当前已发布的姿态 ID
        self.attack_stance_start_time = None # 本轮进攻姿态开始计时
        self.defense_stance_start_time = None# 临时防御姿态开始计时

        # ---- 数据缓存 ----
        self.game_status: GameStatus = None
        self.robot_status: RobotStatus = None

        # ---- 通信 ----
        qos = QoSProfile(depth=10)

        self.sub_game = self.create_subscription(
            GameStatus, 'referee/game_status', self._game_status_cb, qos)
        self.sub_robot = self.create_subscription(
            RobotStatus, 'referee/robot_status', self._robot_status_cb, qos)

        self.pub_goal = self.create_publisher(PoseStamped, 'goal_pose', qos)
        self.pub_spin = self.create_publisher(Float32, 'cmd_spin', qos)
        self.pub_vel = self.create_publisher(Twist, 'cmd_vel', qos)
        self.pub_stance = self.create_publisher(StdFloat32, '/cmd_stance', qos)

        # ---- 主循环 ----
        self.timer = self.create_timer(1.0 / loop_hz, self._loop)

        # ---- 启动提示 ----
        self._print_banner()

    # ─────────────── 回调 ───────────────
    def _game_status_cb(self, msg: GameStatus):
        self.game_status = msg

    def _robot_status_cb(self, msg: RobotStatus):
        self.robot_status = msg

    # ─────────────── 主循环 ─────────────
    def _loop(self):
        prev = self.state
        self._update_stance()

        if self.state == State.WAIT_FOR_GAME:
            self._handle_wait_for_game()
        elif self.state == State.DEAD:
            self._handle_dead()
        elif self.state == State.SUPPLY:
            self._handle_supply()
        elif self.state == State.PATROL:
            self._handle_patrol()

        # 状态切换提示
        if self.state != prev:
            self._print_state_transition(prev, self.state)
            self.prev_state = prev

    # ═══════════════════════════════════════
    #  各状态处理
    # ═══════════════════════════════════════

    def _handle_wait_for_game(self):
        """等待 game_progress == 4 (RUNNING)"""
        if self.game_status is not None and self.game_status.game_progress == 0:
            self._log_info("比赛开始! game_progress=0")
            self.state = State.PATROL
            self.last_patrol_switch_time = self.get_clock().now()
            self.patrol_index = 0
            # 启动小陀螺
            self._publish_spin(self.SPIN_SPEED)
            self.spin_started = True
            # 立即发送第一个巡逻点
            self._send_goal(PATROL_POINTS[0])
            return

        # 每 2 秒打印一次等待
        now = self.get_clock().now()
        if not hasattr(self, '_last_wait_log') or \
                (now - self._last_wait_log).nanoseconds > 2_000_000_000:
            remain = "未知"
            if self.game_status is not None:
                remain = f"{self.game_status.stage_remain_time}s"
            self._log_info(f"⏳ 等待比赛开始... (剩余时间: {remain})")
            self._last_wait_log = now

    def _handle_dead(self):
        """阵亡: 停车, 停陀螺, 等HP>0"""
        # 只在刚进入时执行一次停车
        if self.prev_state != State.DEAD or not hasattr(self, '_dead_stopped'):
            self._publish_spin(0.0)
            self._publish_stop()
            self.spin_started = False
            self._dead_stopped = True
            self._log_warn("💀 哨兵阵亡! 停止一切运动, 等待复活...")

        # 检查是否复活
        if self.robot_status is not None and self.robot_status.current_hp > 0:
            self._log_info("🎉 复活! HP=%d, 重新开始巡逻" % self.robot_status.current_hp)
            self._dead_stopped = False
            self.state = State.PATROL
            self.patrol_index = 0
            self.last_patrol_switch_time = self.get_clock().now()
            # 重新启动小陀螺
            self._publish_spin(self.SPIN_SPEED)
            self.spin_started = True
            self._send_goal(PATROL_POINTS[0])
            return

        # 每 1 秒打印等待复活
        now = self.get_clock().now()
        if not hasattr(self, '_last_dead_log') or \
                (now - self._last_dead_log).nanoseconds > 1_000_000_000:
            self._log_warn("💀 阵亡中... 等待复活 (HP=0)")
            self._last_dead_log = now

    def _handle_supply(self):
        """回血: 前往补给点, 待到血满为止"""
        now = self.get_clock().now()

        # ① 检查是否阵亡
        if self._is_dead():
            self.state = State.DEAD
            return

        # ② 第一次进入: 记录时间
        if not self.supply_goal_sent:
            self.supply_enter_time = now
            self.supply_goal_sent = True
            self._last_supply_goal_time = None  # 强制立即发送
            self._log_info(f"💊 前往补给点 ({SUPPLY_POINT['x']}, {SUPPLY_POINT['y']})")

        # ③ 每秒重发补给目标 (防止单次发布被丢弃)
        if not hasattr(self, '_last_supply_goal_time') or \
                self._last_supply_goal_time is None or \
                (now - self._last_supply_goal_time).nanoseconds > 1_000_000_000:
            self._send_goal(SUPPLY_POINT)
            self._last_supply_goal_time = now

        # ④ 检查退出条件: 血量回满
        hp = self.robot_status.current_hp if self.robot_status else 0
        max_hp = self.robot_status.maximum_hp if self.robot_status else self.HP_FULL
        time_in_supply = (now - self.supply_enter_time).nanoseconds / 1e9

        if hp >= max_hp:
            self._log_info(f"✅ 血量回满 HP={hp}/{max_hp}, 恢复巡逻!")
            self._exit_supply()
            return

        # ⑤ 每秒打印回血进度
        if not hasattr(self, '_last_supply_log') or \
                (now - self._last_supply_log).nanoseconds > 1_000_000_000:
            bar = self._hp_bar(hp, max_hp)
            self._log_info(
                f"💊 补给中 [{bar}] HP={hp}/{max_hp} "
                f"| 已执行补给任务 {time_in_supply:.1f}s"
            )
            self._last_supply_log = now

    def _handle_patrol(self):
        """巡逻: 在巡逻点之间每隔5秒切换"""
        now = self.get_clock().now()

        # ① 检查是否阵亡
        if self._is_dead():
            self.state = State.DEAD
            return

        # ② 检查HP是否需要回血
        if self._need_supply():
            hp = self.robot_status.current_hp if self.robot_status else 0
            self._log_warn(f"⚠ HP={hp} < {self.HP_SUPPLY_THRESH}, 需要回血!")
            self.state = State.SUPPLY
            self.supply_goal_sent = False
            return

        # ③ 检查是否该切换巡逻点
        elapsed = (now - self.last_patrol_switch_time).nanoseconds / 1e9
        if elapsed >= self.PATROL_INTERVAL:
            self.patrol_index = (self.patrol_index + 1) % len(PATROL_POINTS)
            self.last_patrol_switch_time = now
            pt = PATROL_POINTS[self.patrol_index]
            self._send_goal(pt)
            self._log_info(
                f"🔄 切换 → {pt['name']} ({pt['x']}, {pt['y']}) "
                f"[{self.patrol_index + 1}/{len(PATROL_POINTS)}]"
            )

        # ④ 每 2 秒打印巡逻状态
        if not hasattr(self, '_last_patrol_log') or \
                (now - self._last_patrol_log).nanoseconds > 2_000_000_000:
            pt = PATROL_POINTS[self.patrol_index]
            hp = self.robot_status.current_hp if self.robot_status else "?"
            remain = self.PATROL_INTERVAL - elapsed
            self._log_info(
                f"🔄 巡逻中 → {pt['name']} | HP={hp} | "
                f"下次切换: {remain:.1f}s"
            )
            self._last_patrol_log = now

    # ═══════════════════════════════════════
    #  辅助方法
    # ═══════════════════════════════════════

    def _is_dead(self) -> bool:
        if self.robot_status is None:
            return False
        return self.robot_status.current_hp <= 0

    def _need_supply(self) -> bool:
        if self.robot_status is None:
            return False
        return self.robot_status.current_hp < self.HP_SUPPLY_THRESH

    def _exit_supply(self):
        """退出补给, 恢复巡逻"""
        self.state = State.PATROL
        self.patrol_index = 0
        self.last_patrol_switch_time = self.get_clock().now()
        self.supply_goal_sent = False
        self._send_goal(PATROL_POINTS[0])

    def _send_goal(self, point: dict):
        """发布导航目标 (PoseStamped), 效果等同于 rviz2 点击"""
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(point["x"])
        msg.pose.position.y = float(point["y"])
        msg.pose.position.z = 0.0

        yaw = float(point["yaw"])
        # yaw → quaternion
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.pub_goal.publish(msg)
        self.last_goal_sent = point["name"]

    def _publish_spin(self, speed: float):
        msg = Float32()
        msg.data = speed
        self.pub_spin.publish(msg)

    def _update_stance(self):
        """根据 HP 和进攻姿态累计时间发布 /cmd_stance。"""
        if self.robot_status is None or self._is_dead():
            self._set_stance(Stance.MOVE, "无有效血量或阵亡")
            return

        hp = self.robot_status.current_hp
        now = self.get_clock().now()

        if hp <= self.STANCE_HP_THRESH:
            self.attack_stance_start_time = None
            self.defense_stance_start_time = None
            self._set_stance(Stance.MOVE, f"HP={hp} <= {self.STANCE_HP_THRESH}")
            return

        if self.defense_stance_start_time is not None:
            defense_elapsed = (now - self.defense_stance_start_time).nanoseconds / 1e9
            if defense_elapsed < self.DEFENSE_STANCE_DURATION:
                self._set_stance(Stance.DEFENSE, f"进攻姿态计时保护 {defense_elapsed:.1f}s")
                return

            self.defense_stance_start_time = None
            self.attack_stance_start_time = now
            self._set_stance(Stance.ATTACK, "临时防御结束")
            return

        if self.attack_stance_start_time is None or self.current_stance != Stance.ATTACK:
            self.attack_stance_start_time = now
            self._set_stance(Stance.ATTACK, f"HP={hp} > {self.STANCE_HP_THRESH}")
            return

        attack_elapsed = (now - self.attack_stance_start_time).nanoseconds / 1e9
        if attack_elapsed >= self.ATTACK_STANCE_DURATION:
            self.defense_stance_start_time = now
            self._set_stance(Stance.DEFENSE, f"进攻姿态已持续 {attack_elapsed:.1f}s")
            return

        self._set_stance(Stance.ATTACK, f"HP={hp} > {self.STANCE_HP_THRESH}", log_change=False)

    def _set_stance(self, stance: int, reason: str, log_change: bool = True):
        msg = StdFloat32()
        msg.data = float(stance)
        self.pub_stance.publish(msg)

        if stance == self.current_stance:
            return

        old_name = STANCE_NAMES.get(self.current_stance, "未知姿态")
        new_name = STANCE_NAMES.get(stance, "未知姿态")
        self.current_stance = stance
        if log_change:
            self._log_info(f"姿态切换: {old_name} -> {new_name} ({reason})")

    def _publish_stop(self):
        """停止底盘运动"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        self.pub_vel.publish(msg)

    def _hp_bar(self, hp: int, max_hp: int, length: int = 20) -> str:
        """生成血量进度条"""
        if max_hp <= 0:
            max_hp = self.HP_FULL
        ratio = max(0.0, min(1.0, hp / max_hp))
        filled = int(ratio * length)
        return "█" * filled + "░" * (length - filled)

    # ═══════════════════════════════════════
    #  终端可视化打印
    # ═══════════════════════════════════════

    def _print_banner(self):
        lines = [
            "",
            box_top(),
            box_line("ADAM 2026UC 哨兵决策系统 v1"),
            box_line(""),
            box_line(f"巡逻点1: ({PATROL_POINTS[0]['x']}, {PATROL_POINTS[0]['y']})"),
            box_line(f"巡逻点2: ({PATROL_POINTS[1]['x']}, {PATROL_POINTS[1]['y']})"),
            box_line(f"补给点 : ({SUPPLY_POINT['x']}, {SUPPLY_POINT['y']})"),
            box_line(""),
            box_line(f"回血阈值: HP < {self.HP_SUPPLY_THRESH}"),
            box_line(f"巡逻间隔: {self.PATROL_INTERVAL}s"),
            box_line(f"补给超时: {self.SUPPLY_TIMEOUT}s"),
            box_line(f"陀螺转速: {self.SPIN_SPEED} rad/s"),
            box_line(f"姿态阈值: HP > {self.STANCE_HP_THRESH} 进攻, 否则移动"),
            box_line(f"进攻姿态保护: {self.ATTACK_STANCE_DURATION}s 后防御 {self.DEFENSE_STANCE_DURATION}s"),
            box_line(""),
            box_line("话题订阅: referee/game_status, referee/robot_status"),
            box_line("话题发布: goal_pose, cmd_spin, cmd_vel, /cmd_stance"),
            box_bot(),
            "",
        ]
        for l in lines:
            self.get_logger().info(l)

    def _print_state_transition(self, old_state: str, new_state: str):
        hp = self.robot_status.current_hp if self.robot_status else "?"
        max_hp = self.robot_status.maximum_hp if self.robot_status else "?"
        lines = [
            "",
            "┌─────────────────────────────────────────────┐",
            f"│ {state_icon(old_state)} {old_state}  ══▶  {state_icon(new_state)} {new_state}",
            f"│ HP: {hp}/{max_hp}",
            "└─────────────────────────────────────────────┘",
            "",
        ]
        for l in lines:
            self.get_logger().info(l)

    def _log_info(self, text: str):
        hp_str = ""
        if self.robot_status:
            hp_str = f" [HP:{self.robot_status.current_hp}/{self.robot_status.maximum_hp}]"
        self.get_logger().info(f"[{state_icon(self.state)} {self.state}]{hp_str} {text}")

    def _log_warn(self, text: str):
        self.get_logger().warn(f"[{state_icon(self.state)} {self.state}] {text}")


# ═══════════════════════════════════════
#  入口
# ═══════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = AllianceDecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到 Ctrl+C, 正在退出...")
    finally:
        # 退出前停车
        node._publish_spin(0.0)
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
