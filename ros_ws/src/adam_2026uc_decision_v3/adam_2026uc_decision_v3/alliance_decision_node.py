#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ADAM 2026UC 哨兵决策节点 v3
============================
在 v2 的基础上增加固定时机打符任务。

状态:
    WAIT_FOR_GAME  - 等待比赛开始 (game_progress == 4)
    PATROL         - 在巡逻点1和巡逻点2之间往返
    SUPPLY         - 前往补给点回血 / 补弹 / 复活后强制回补
    ENERGY_TASK    - 固定时机抢占巡逻, 前往打符点并等待 26 秒
    DEAD           - 阵亡, 停车等复活

话题:
    订阅: referee/game_status, referee/robot_status, /active_region
    发布: goal_pose, cmd_spin, cmd_vel, /cmd_stance
"""

import math

import rclpy
from example_interfaces.msg import Float32
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import UInt8, UInt32

try:
    from pb_rm_interfaces.msg import GameStatus, RobotStatus
except ImportError:
    print("[FATAL] 无法导入 pb_rm_interfaces, 请确保已 source install/setup.bash")
    raise


class State:
    WAIT_FOR_GAME = "WAIT_FOR_GAME"
    PATROL = "PATROL"
    SUPPLY = "SUPPLY"
    ENERGY_TASK = "ENERGY_TASK"
    DEAD = "DEAD"


class Stance:
    MOVE = 3
    DEFEND = 2


STANCE_NAMES = {
    Stance.MOVE: "移动姿态",
    Stance.DEFEND: "防御姿态",
}

PATROL_POINTS = [
    {"name": "巡逻点1", "x": 4.27, "y": -2.89, "yaw": 0.00},
    {"name": "巡逻点2", "x": 4.39, "y": -1.54, "yaw": 1.57},
]
SUPPLY_POINT = {"name": "补给点", "x": -1.07, "y": -5.22, "yaw": 0.00}

MATCH_DURATION_SEC = 420
ENERGY_REGION_CODE = 10
BOX_WIDTH = 60


def box_top():
    return "╔" + "═" * BOX_WIDTH + "╗"


def box_bot():
    return "╚" + "═" * BOX_WIDTH + "╝"


def box_line(text: str):
    s = f"║  {text}"
    pad = BOX_WIDTH + 1 - _display_width(s)
    if pad < 1:
        pad = 1
    return s + " " * pad + "║"


def _display_width(s: str) -> int:
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
        State.PATROL: "🔄",
        State.SUPPLY: "💊",
        State.ENERGY_TASK: "🎯",
        State.DEAD: "💀",
    }
    return icons.get(state, "❓")


class AllianceDecisionNode(Node):

    def __init__(self):
        super().__init__('alliance_decision_node')

        self.declare_parameter('hp_supply_threshold', 201)
        self.declare_parameter('ammo_supply_threshold', 20)
        self.declare_parameter('hp_full', 400)
        self.declare_parameter('supply_timeout_sec', 5.0)
        self.declare_parameter('patrol_interval_sec', 5.0)
        self.declare_parameter('spin_speed', 7.0)
        self.declare_parameter('loop_rate_hz', 10.0)
        self.declare_parameter('defend_stance_duration_sec', 60.0)
        self.declare_parameter('move_stance_duration_sec', 60.0)
        self.declare_parameter('energy_task_duration_sec', 26.0)
        self.declare_parameter('energy_goal_x', 0.0)
        self.declare_parameter('energy_goal_y', 0.0)
        self.declare_parameter('energy_goal_yaw', 0.0)

        self.HP_SUPPLY_THRESH = self.get_parameter('hp_supply_threshold').value
        self.AMMO_SUPPLY_THRESH = self.get_parameter('ammo_supply_threshold').value
        self.HP_FULL = self.get_parameter('hp_full').value
        self.SUPPLY_TIMEOUT = self.get_parameter('supply_timeout_sec').value
        self.PATROL_INTERVAL = self.get_parameter('patrol_interval_sec').value
        self.SPIN_SPEED = self.get_parameter('spin_speed').value
        loop_hz = self.get_parameter('loop_rate_hz').value
        self.DEFEND_STANCE_DURATION = self.get_parameter('defend_stance_duration_sec').value
        self.MOVE_STANCE_DURATION = self.get_parameter('move_stance_duration_sec').value
        self.ENERGY_TASK_DURATION = self.get_parameter('energy_task_duration_sec').value
        self.ENERGY_POINT = {
            "name": "打符点",
            "x": float(self.get_parameter('energy_goal_x').value),
            "y": float(self.get_parameter('energy_goal_y').value),
            "yaw": float(self.get_parameter('energy_goal_yaw').value),
        }

        self.state = State.WAIT_FOR_GAME
        self.prev_state = None
        self.patrol_index = 0
        self.last_patrol_switch_time = None
        self.supply_enter_time = None
        self.supply_goal_sent = False
        self.supply_need_hp = False
        self.supply_need_ammo = False
        self.force_supply_after_respawn = False
        self.spin_started = False
        self.last_goal_sent = None
        self.current_stance = None
        self.defend_stance_start_time = None
        self.move_stance_start_time = None
        self.last_stance_reason = "未初始化"

        self.energy_task_started = False
        self.energy_task_goal_sent = False
        self.energy_task_started_at = None
        self.energy_task_region_enter_at = None
        self.energy_task_0_done = False
        self.energy_task_90_done = False
        self.active_energy_task_id = None
        self.saved_patrol_index = 0
        self.active_region_code = 0

        self.game_status = None
        self.robot_status = None

        qos = QoSProfile(depth=10)
        self.sub_game = self.create_subscription(
            GameStatus, 'referee/game_status', self._game_status_cb, qos)
        self.sub_robot = self.create_subscription(
            RobotStatus, 'referee/robot_status', self._robot_status_cb, qos)
        self.sub_active_region = self.create_subscription(
            UInt8, '/active_region', self._active_region_cb, qos)

        self.pub_goal = self.create_publisher(PoseStamped, 'goal_pose', qos)
        self.pub_spin = self.create_publisher(Float32, 'cmd_spin', qos)
        self.pub_vel = self.create_publisher(Twist, 'cmd_vel', qos)
        self.pub_stance = self.create_publisher(UInt32, '/cmd_stance', qos)

        self.timer = self.create_timer(1.0 / loop_hz, self._loop)
        self._print_banner()

    def _game_status_cb(self, msg: GameStatus):
        self.game_status = msg

    def _robot_status_cb(self, msg: RobotStatus):
        self.robot_status = msg

    def _active_region_cb(self, msg: UInt8):
        self.active_region_code = int(msg.data)

    def _loop(self):
        prev = self.state
        self._update_stance()
        self._check_energy_task_trigger()

        if self.state == State.WAIT_FOR_GAME:
            self._handle_wait_for_game()
        elif self.state == State.DEAD:
            self._handle_dead()
        elif self.state == State.SUPPLY:
            self._handle_supply()
        elif self.state == State.ENERGY_TASK:
            self._handle_energy_task()
        elif self.state == State.PATROL:
            self._handle_patrol()

        if self.state != prev:
            self._print_state_transition(prev, self.state)
            self.prev_state = prev

    def _handle_wait_for_game(self):
        if self.game_status is not None and self.game_status.game_progress == 4:
            self._log_info("比赛开始! game_progress=4")
            self.last_patrol_switch_time = self.get_clock().now()
            self.patrol_index = 0
            self._publish_spin(self.SPIN_SPEED)
            self.spin_started = True
            if self._need_supply() or self._need_ammo_supply():
                self.state = State.SUPPLY
                self.supply_goal_sent = False
                self._send_goal(SUPPLY_POINT)
            else:
                self.state = State.PATROL
                self._send_goal(PATROL_POINTS[0])
            return

        now = self.get_clock().now()
        if not hasattr(self, '_last_wait_log') or \
                (now - self._last_wait_log).nanoseconds > 2_000_000_000:
            remain = "未知"
            if self.game_status is not None:
                remain = f"{self.game_status.stage_remain_time}s"
            self._log_info(f"⏳ 等待比赛开始... (剩余时间: {remain})")
            self._last_wait_log = now

    def _handle_dead(self):
        if self.prev_state != State.DEAD or not hasattr(self, '_dead_stopped'):
            self._publish_spin(0.0)
            self._publish_stop()
            self.spin_started = False
            self._dead_stopped = True
            self._log_warn("💀 哨兵阵亡! 停止一切运动, 等待复活...")

        if self.robot_status is not None and self.robot_status.current_hp > 0:
            self._log_info("🎉 复活! HP=%d, 立即返回补给点" % self.robot_status.current_hp)
            self._dead_stopped = False
            self.state = State.SUPPLY
            self.force_supply_after_respawn = True
            self.supply_goal_sent = False
            self._publish_spin(self.SPIN_SPEED)
            self.spin_started = True
            return

        now = self.get_clock().now()
        if not hasattr(self, '_last_dead_log') or \
                (now - self._last_dead_log).nanoseconds > 1_000_000_000:
            self._log_warn("💀 阵亡中... 等待复活 (HP=0)")
            self._last_dead_log = now

    def _handle_supply(self):
        now = self.get_clock().now()

        if self._is_dead():
            self.state = State.DEAD
            return

        if not self.supply_goal_sent:
            self.supply_enter_time = now
            self.supply_goal_sent = True
            self.supply_need_hp = self.force_supply_after_respawn or self._need_supply()
            self.supply_need_ammo = self.force_supply_after_respawn or self._need_ammo_supply()
            self._last_supply_goal_time = None
            reason = self._supply_reason_text()
            self._log_info(
                f"💊 前往补给点 ({SUPPLY_POINT['x']}, {SUPPLY_POINT['y']}) | 原因: {reason}"
            )

        if not hasattr(self, '_last_supply_goal_time') or \
                self._last_supply_goal_time is None or \
                (now - self._last_supply_goal_time).nanoseconds > 1_000_000_000:
            self._send_goal(SUPPLY_POINT)
            self._last_supply_goal_time = now

        hp = self.robot_status.current_hp if self.robot_status else 0
        max_hp = self.robot_status.maximum_hp if self.robot_status else self.HP_FULL
        ammo = self._get_ammo_count()
        time_in_supply = (now - self.supply_enter_time).nanoseconds / 1e9

        if self._is_supply_complete():
            ammo_text = "未知" if ammo is None else str(ammo)
            self._log_info(f"✅ 补给完成 HP={hp}/{max_hp} | Ammo={ammo_text}, 恢复巡逻!")
            self._exit_supply()
            return

        if not hasattr(self, '_last_supply_log') or \
                (now - self._last_supply_log).nanoseconds > 1_000_000_000:
            bar = self._hp_bar(hp, max_hp)
            ammo_text = "未知" if ammo is None else str(ammo)
            self._log_info(
                f"💊 补给中 [{bar}] HP={hp}/{max_hp} | Ammo={ammo_text} | 已执行补给任务 {time_in_supply:.1f}s"
            )
            self._last_supply_log = now

    def _handle_patrol(self):
        now = self.get_clock().now()

        if self._is_dead():
            self.state = State.DEAD
            return

        if self._need_supply() or self._need_ammo_supply():
            hp = self.robot_status.current_hp if self.robot_status else 0
            ammo = self._get_ammo_count()
            ammo_text = "未知" if ammo is None else str(ammo)
            self._log_warn(
                f"⚠ 进入补给: HP={hp}, Ammo={ammo_text} | 阈值 HP<={self.HP_SUPPLY_THRESH} 或 Ammo<{self.AMMO_SUPPLY_THRESH}"
            )
            self.state = State.SUPPLY
            self.supply_goal_sent = False
            self._send_goal(SUPPLY_POINT)
            return

        elapsed = (now - self.last_patrol_switch_time).nanoseconds / 1e9
        if elapsed >= self.PATROL_INTERVAL:
            self.patrol_index = (self.patrol_index + 1) % len(PATROL_POINTS)
            self.last_patrol_switch_time = now
            pt = PATROL_POINTS[self.patrol_index]
            self._send_goal(pt)
            self._log_info(
                f"🔄 切换 → {pt['name']} ({pt['x']}, {pt['y']}) [{self.patrol_index + 1}/{len(PATROL_POINTS)}]"
            )

        if not hasattr(self, '_last_patrol_log') or \
                (now - self._last_patrol_log).nanoseconds > 2_000_000_000:
            pt = PATROL_POINTS[self.patrol_index]
            hp = self.robot_status.current_hp if self.robot_status else "?"
            remain = self.PATROL_INTERVAL - elapsed
            self._log_info(f"🔄 巡逻中 → {pt['name']} | HP={hp} | 下次切换: {remain:.1f}s")
            self._last_patrol_log = now

    def _handle_energy_task(self):
        now = self.get_clock().now()

        if self._is_dead():
            self.state = State.DEAD
            return

        if not self.energy_task_goal_sent:
            self.energy_task_goal_sent = True
            self.energy_task_started = True
            self.energy_task_started_at = now
            self.energy_task_region_enter_at = None
            self._send_goal(self.ENERGY_POINT)
            self._log_warn(
                f"🎯 开始第 {self.active_energy_task_id} 次打符任务, 前往打符点 ({self.ENERGY_POINT['x']}, {self.ENERGY_POINT['y']})"
            )

        if self.active_region_code == ENERGY_REGION_CODE and self.energy_task_region_enter_at is None:
            self.energy_task_region_enter_at = now
            self._log_warn(
                f"🎯 已进入打符区域, 开始计时 {self.ENERGY_TASK_DURATION:.1f}s"
            )

        if self.energy_task_region_enter_at is not None:
            active_elapsed = (now - self.energy_task_region_enter_at).nanoseconds / 1e9
            remain = max(0.0, self.ENERGY_TASK_DURATION - active_elapsed)
            if active_elapsed >= self.ENERGY_TASK_DURATION:
                self._log_info(
                    f"✅ 第 {self.active_energy_task_id} 次打符任务完成, 持续 {self.ENERGY_TASK_DURATION:.1f}s"
                )
                self._finish_energy_task()
                return
            if not hasattr(self, '_last_energy_active_log') or \
                    (now - self._last_energy_active_log).nanoseconds > 1_000_000_000:
                self._log_info(
                    f"🎯 打符中 | 区域码={self.active_region_code} | 剩余 {remain:.1f}s"
                )
                self._last_energy_active_log = now
        else:
            if not hasattr(self, '_last_energy_nav_log') or \
                    (now - self._last_energy_nav_log).nanoseconds > 1_000_000_000:
                self._log_info(
                    f"🎯 前往打符点中 | 当前区域码={self.active_region_code} | 等待进入打符区"
                )
                self._last_energy_nav_log = now

    def _check_energy_task_trigger(self):
        if self.game_status is None or self.game_status.game_progress != 4:
            return
        if self.state in (State.WAIT_FOR_GAME, State.DEAD, State.SUPPLY, State.ENERGY_TASK):
            return

        elapsed = self._match_elapsed_sec()
        if elapsed is None:
            return

        if (not self.energy_task_0_done) and elapsed >= 0.0:
            self._start_energy_task(task_id=1)
            return
        if (not self.energy_task_90_done) and elapsed >= 90.0:
            self._start_energy_task(task_id=2)

    def _start_energy_task(self, task_id: int):
        if self.state != State.PATROL:
            return
        self.saved_patrol_index = self.patrol_index
        self.active_energy_task_id = task_id
        self.state = State.ENERGY_TASK
        self.energy_task_goal_sent = False
        self.energy_task_region_enter_at = None
        self.energy_task_started_at = None
        self._log_warn(f"🎯 抢占巡逻, 执行第 {task_id} 次固定时机打符")

    def _finish_energy_task(self):
        if self.active_energy_task_id == 1:
            self.energy_task_0_done = True
        elif self.active_energy_task_id == 2:
            self.energy_task_90_done = True

        self.state = State.PATROL
        self.energy_task_goal_sent = False
        self.energy_task_started = False
        self.energy_task_started_at = None
        self.energy_task_region_enter_at = None
        self.active_energy_task_id = None
        self.patrol_index = self.saved_patrol_index
        self.last_patrol_switch_time = self.get_clock().now()
        self._send_goal(PATROL_POINTS[self.patrol_index])

    def _match_elapsed_sec(self):
        if self.game_status is None:
            return None
        remain = int(self.game_status.stage_remain_time)
        return float(max(0, MATCH_DURATION_SEC - remain))

    def _is_dead(self) -> bool:
        if self.robot_status is None:
            return False
        return self.robot_status.current_hp <= 0

    def _need_supply(self) -> bool:
        if self.robot_status is None:
            return False
        return self.robot_status.current_hp <= self.HP_SUPPLY_THRESH

    def _get_ammo_count(self):
        if self.robot_status is None:
            return None
        for field in ('projectile_allowance_17mm', 'bullet_remaining_num_17mm'):
            value = getattr(self.robot_status, field, None)
            if value is not None:
                return int(value)
        return None

    def _need_ammo_supply(self) -> bool:
        ammo = self._get_ammo_count()
        if ammo is None:
            return False
        return ammo < self.AMMO_SUPPLY_THRESH

    def _is_supply_complete(self) -> bool:
        hp = self.robot_status.current_hp if self.robot_status else 0
        max_hp = self.robot_status.maximum_hp if self.robot_status else self.HP_FULL
        hp_ok = (not self.supply_need_hp) or (hp >= max_hp)
        ammo = self._get_ammo_count()
        ammo_ok = (not self.supply_need_ammo) or (ammo is not None and ammo >= self.AMMO_SUPPLY_THRESH)
        if self.supply_need_ammo and ammo is None:
            ammo_ok = False
        return hp_ok and ammo_ok

    def _supply_reason_text(self) -> str:
        reasons = []
        if self.force_supply_after_respawn:
            reasons.append("复活后强制补给")
        if self.supply_need_hp:
            reasons.append("回血")
        if self.supply_need_ammo:
            reasons.append("补弹")
        if not reasons:
            reasons.append("补给确认")
        return " + ".join(reasons)

    def _exit_supply(self):
        self.state = State.PATROL
        self.patrol_index = 0
        self.last_patrol_switch_time = self.get_clock().now()
        self.supply_goal_sent = False
        self.supply_need_hp = False
        self.supply_need_ammo = False
        self.force_supply_after_respawn = False
        self._send_goal(PATROL_POINTS[0])

    def _send_goal(self, point: dict):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(point["x"])
        msg.pose.position.y = float(point["y"])
        msg.pose.position.z = 0.0

        yaw = float(point["yaw"])
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
        if self.robot_status is None or self._is_dead():
            self.defend_stance_start_time = None
            self.move_stance_start_time = None
            self._set_stance(Stance.MOVE, "无有效血量或阵亡")
            self._log_current_stance()
            return

        now = self.get_clock().now()

        if self.current_stance is None:
            self.defend_stance_start_time = now
            self.move_stance_start_time = None
            self._set_stance(Stance.DEFEND, "初始化进入防御姿态")
            self._log_current_stance()
            return

        if self.current_stance == Stance.DEFEND:
            if self.defend_stance_start_time is None:
                self.defend_stance_start_time = now
            defend_elapsed = (now - self.defend_stance_start_time).nanoseconds / 1e9
            if defend_elapsed >= self.DEFEND_STANCE_DURATION:
                self.defend_stance_start_time = None
                self.move_stance_start_time = now
                self._set_stance(Stance.MOVE, f"防御姿态已持续 {defend_elapsed:.1f}s")
            else:
                self._set_stance(Stance.DEFEND, f"防御姿态计时 {defend_elapsed:.1f}s", log_change=False)
            self._log_current_stance()
            return

        if self.current_stance == Stance.MOVE:
            if self.move_stance_start_time is None:
                self.move_stance_start_time = now
            move_elapsed = (now - self.move_stance_start_time).nanoseconds / 1e9
            if move_elapsed >= self.MOVE_STANCE_DURATION:
                self.move_stance_start_time = None
                self.defend_stance_start_time = now
                self._set_stance(Stance.DEFEND, f"移动姿态已持续 {move_elapsed:.1f}s")
            else:
                self._set_stance(Stance.MOVE, f"移动姿态计时 {move_elapsed:.1f}s", log_change=False)
            self._log_current_stance()
            return

        self.defend_stance_start_time = now
        self.move_stance_start_time = None
        self._set_stance(Stance.DEFEND, "未知姿态恢复为防御姿态")
        self._log_current_stance()

    def _set_stance(self, stance: int, reason: str, log_change: bool = True):
        msg = UInt32()
        msg.data = int(stance)
        self.pub_stance.publish(msg)
        self.last_stance_reason = reason

        if stance == self.current_stance:
            return

        old_name = STANCE_NAMES.get(self.current_stance, "未知姿态")
        new_name = STANCE_NAMES.get(stance, "未知姿态")
        self.current_stance = stance
        if log_change:
            self._log_info(f"姿态切换: {old_name} -> {new_name} ({reason})")

    def _log_current_stance(self):
        if self.current_stance is None:
            return

        now = self.get_clock().now()
        if hasattr(self, '_last_stance_log') and \
                (now - self._last_stance_log).nanoseconds <= 1_000_000_000:
            return

        stance_name = STANCE_NAMES.get(self.current_stance, f"未知姿态({self.current_stance})")
        self._log_info(f"当前姿态: {stance_name} ({self.last_stance_reason})")
        self._last_stance_log = now

    def _publish_stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        self.pub_vel.publish(msg)

    def _hp_bar(self, hp: int, max_hp: int, length: int = 20) -> str:
        if max_hp <= 0:
            max_hp = self.HP_FULL
        ratio = max(0.0, min(1.0, hp / max_hp))
        filled = int(ratio * length)
        return "█" * filled + "░" * (length - filled)

    def _print_banner(self):
        lines = [
            "",
            box_top(),
            box_line("ADAM 2026UC 哨兵决策系统 v3"),
            box_line(""),
            box_line(f"巡逻点1: ({PATROL_POINTS[0]['x']}, {PATROL_POINTS[0]['y']})"),
            box_line(f"巡逻点2: ({PATROL_POINTS[1]['x']}, {PATROL_POINTS[1]['y']})"),
            box_line(f"补给点 : ({SUPPLY_POINT['x']}, {SUPPLY_POINT['y']})"),
            box_line(f"打符点 : ({self.ENERGY_POINT['x']}, {self.ENERGY_POINT['y']})"),
            box_line(""),
            box_line("打符触发: 比赛开始, 比赛开始 1分30秒"),
            box_line(f"打符区域持续时间: {self.ENERGY_TASK_DURATION}s"),
            box_line(f"回血阈值: HP <= {self.HP_SUPPLY_THRESH}"),
            box_line(f"补弹阈值: Ammo < {self.AMMO_SUPPLY_THRESH}"),
            box_line(f"巡逻间隔: {self.PATROL_INTERVAL}s"),
            box_line(f"陀螺转速: {self.SPIN_SPEED} rad/s"),
            box_line(f"姿态切换周期: 防御 {self.DEFEND_STANCE_DURATION}s, 移动 {self.MOVE_STANCE_DURATION}s"),
            box_line(""),
            box_line("话题订阅: referee/game_status, referee/robot_status, /active_region"),
            box_line("话题发布: goal_pose, cmd_spin, cmd_vel, /cmd_stance"),
            box_bot(),
            "",
        ]
        for line in lines:
            self.get_logger().info(line)

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
        for line in lines:
            self.get_logger().info(line)

    def _log_info(self, text: str):
        hp_str = ""
        if self.robot_status:
            hp_str = f" [HP:{self.robot_status.current_hp}/{self.robot_status.maximum_hp}]"
        self.get_logger().info(f"[{state_icon(self.state)} {self.state}]{hp_str} {text}")

    def _log_warn(self, text: str):
        self.get_logger().warn(f"[{state_icon(self.state)} {self.state}] {text}")


def main(args=None):
    rclpy.init(args=args)
    node = AllianceDecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到 Ctrl+C, 正在退出...")
    finally:
        node._publish_spin(0.0)
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
