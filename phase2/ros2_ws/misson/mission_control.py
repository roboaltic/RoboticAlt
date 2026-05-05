#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, Int32


class MissionAuto(Node):
    SEARCH = 0
    TRACK = 1
    AVOID = 2
    ARRIVED = 3

    def __init__(self):
        super().__init__('mission_auto')

        # ---------------- Topics ----------------
        self.declare_parameter('aruco_topic', '/aruco/target')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('buzzer_topic', '/buzzer_signal')

        # ---------------- Mission ----------------
        self.declare_parameter('target_ids', '')  # приклад: "1,2,3"
        self.declare_parameter('remember_passed_ids', True)

        # ---------------- Motion ----------------
        self.declare_parameter('rate_hz', 20.0)

        self.declare_parameter('search_w', 0.22)

        self.declare_parameter('track_v_fast', 0.10)
        self.declare_parameter('track_v_slow', 0.045)

        self.declare_parameter('kp_w', 0.003)
        self.declare_parameter('max_w', 0.50)

        self.declare_parameter('avoid_turn_w', 0.40)
        self.declare_parameter('avoid_v', 0.06)
        self.declare_parameter('avoid_curve_w', 0.16)

        # ---------------- Thresholds ----------------
        self.declare_parameter('stop_dist', 0.24)
        self.declare_parameter('slow_dist', 0.55)
        self.declare_parameter('avoid_dist', 0.42)
        self.declare_parameter('clear_dist', 0.55)

        self.declare_parameter('center_px', 28.0)
        self.declare_parameter('hard_px', 125.0)
        self.declare_parameter('marker_stop_px', 220.0)
        self.declare_parameter('marker_arrived_min_px', 145.0)

        self.declare_parameter('marker_timeout', 0.45)
        self.declare_parameter('scan_timeout', 0.60)
        self.declare_parameter('arrived_pause', 1.0)

        # ---------------- Lidar sectors ----------------
        self.declare_parameter('front_deg', 25.0)
        self.declare_parameter('left_min_deg', 35.0)
        self.declare_parameter('left_max_deg', 105.0)
        self.declare_parameter('right_min_deg', -105.0)
        self.declare_parameter('right_max_deg', -35.0)

        # ---------------- Load parameters ----------------
        self.aruco_topic = self.get_parameter('aruco_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.buzzer_topic = self.get_parameter('buzzer_topic').value

        self.target_ids = self.parse_ids(self.get_parameter('target_ids').value)
        self.remember_passed = bool(self.get_parameter('remember_passed_ids').value)

        self.search_w = float(self.get_parameter('search_w').value)

        self.track_v_fast = float(self.get_parameter('track_v_fast').value)
        self.track_v_slow = float(self.get_parameter('track_v_slow').value)

        self.kp_w = float(self.get_parameter('kp_w').value)
        self.max_w = float(self.get_parameter('max_w').value)

        self.avoid_turn_w = float(self.get_parameter('avoid_turn_w').value)
        self.avoid_v = float(self.get_parameter('avoid_v').value)
        self.avoid_curve_w = float(self.get_parameter('avoid_curve_w').value)

        self.stop_dist = float(self.get_parameter('stop_dist').value)
        self.slow_dist = float(self.get_parameter('slow_dist').value)
        self.avoid_dist = float(self.get_parameter('avoid_dist').value)
        self.clear_dist = float(self.get_parameter('clear_dist').value)

        self.center_px = float(self.get_parameter('center_px').value)
        self.hard_px = float(self.get_parameter('hard_px').value)

        self.marker_stop_px = float(self.get_parameter('marker_stop_px').value)
        self.marker_arrived_min_px = float(
            self.get_parameter('marker_arrived_min_px').value
        )

        self.marker_timeout = float(self.get_parameter('marker_timeout').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.arrived_pause = float(self.get_parameter('arrived_pause').value)

        self.front_deg = float(self.get_parameter('front_deg').value)

        self.left_min_deg = float(self.get_parameter('left_min_deg').value)
        self.left_max_deg = float(self.get_parameter('left_max_deg').value)

        self.right_min_deg = float(self.get_parameter('right_min_deg').value)
        self.right_max_deg = float(self.get_parameter('right_max_deg').value)

        # ---------------- State ----------------
        self.mode = self.SEARCH
        self.mode_time = time.monotonic()

        self.marker_seen = False
        self.marker_id = -1
        self.dx = 0.0
        self.dy = 0.0
        self.marker_w = 0.0
        self.last_marker_time = 0.0

        self.ranges = []
        self.angle_min = 0.0
        self.angle_inc = 0.0
        self.range_min = 0.05
        self.range_max = 12.0
        self.last_scan_time = 0.0

        self.front = math.inf
        self.left = math.inf
        self.right = math.inf

        # +1 = об'їзд зліва, -1 = об'їзд справа
        self.avoid_side = +1

        self.passed = set()
        self.target_index = 0

        self.last_log = 0.0

        # ---------------- ROS ----------------
        self.create_subscription(
            Float32MultiArray,
            self.aruco_topic,
            self.aruco_cb,
            10
        )

        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_cb,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.buzzer_pub = self.create_publisher(Int32, self.buzzer_topic, 10)

        rate = float(self.get_parameter('rate_hz').value)
        self.timer = self.create_timer(1.0 / max(rate, 1.0), self.loop)

        self.get_logger().info("✅ mission_auto optimized запущено")
        self.get_logger().info(
            f"targets={self.target_ids if self.target_ids else 'ANY'} | "
            f"stop={self.stop_dist:.2f}m | avoid={self.avoid_dist:.2f}m"
        )

    # ------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------

    def aruco_cb(self, msg):
        """
        Очікуваний формат від camera_node:
        [seen, id, dx, dy, marker_width]
        """

        if len(msg.data) < 5:
            return

        if msg.data[0] < 0.5:
            self.marker_seen = False
            return

        self.marker_seen = True
        self.marker_id = int(msg.data[1])
        self.dx = float(msg.data[2])
        self.dy = float(msg.data[3])
        self.marker_w = float(msg.data[4])
        self.last_marker_time = time.monotonic()

    def scan_cb(self, msg):
        self.ranges = list(msg.ranges)
        self.angle_min = float(msg.angle_min)
        self.angle_inc = float(msg.angle_increment)
        self.range_min = float(msg.range_min)
        self.range_max = float(msg.range_max)
        self.last_scan_time = time.monotonic()

        self.front = self.sector_min(-self.front_deg, self.front_deg)
        self.left = self.sector_min(self.left_min_deg, self.left_max_deg)
        self.right = self.sector_min(self.right_min_deg, self.right_max_deg)

    # ------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------

    def parse_ids(self, text):
        text = str(text).strip()

        if not text:
            return []

        ids = []

        for part in text.split(','):
            try:
                ids.append(int(part.strip()))
            except ValueError:
                pass

        return ids

    def now(self):
        return time.monotonic()

    def set_mode(self, mode):
        if self.mode != mode:
            self.mode = mode
            self.mode_time = self.now()
            self.get_logger().info(f"➡️ MODE: {self.mode_name()}")

    def mode_age(self):
        return self.now() - self.mode_time

    def mode_name(self):
        names = {
            self.SEARCH: "SEARCH",
            self.TRACK: "TRACK",
            self.AVOID: "AVOID",
            self.ARRIVED: "ARRIVED",
        }
        return names.get(self.mode, "UNKNOWN")

    def fresh_marker(self):
        return (
            self.marker_seen and
            self.now() - self.last_marker_time <= self.marker_timeout
        )

    def fresh_scan(self):
        return self.now() - self.last_scan_time <= self.scan_timeout

    def target_done(self):
        return bool(self.target_ids) and self.target_index >= len(self.target_ids)

    def current_target(self):
        if not self.target_ids:
            return None

        if self.target_index >= len(self.target_ids):
            return None

        return self.target_ids[self.target_index]

    def marker_allowed(self):
        if not self.fresh_marker():
            return False

        target = self.current_target()

        if target is not None and self.marker_id != target:
            return False

        if self.remember_passed and self.marker_id in self.passed:
            return False

        return True

    def normalize_180(self, deg):
        return (deg + 180.0) % 360.0 - 180.0

    def valid_range(self, r):
        if math.isnan(r) or math.isinf(r):
            return False

        if r < max(0.03, self.range_min):
            return False

        if r > self.range_max:
            return False

        return True

    def sector_min(self, min_deg, max_deg):
        if not self.ranges or self.angle_inc == 0.0:
            return math.inf

        vals = []

        for i, r in enumerate(self.ranges):
            if not self.valid_range(r):
                continue

            angle = self.angle_min + i * self.angle_inc
            deg = self.normalize_180(math.degrees(angle))

            if min_deg <= deg <= max_deg:
                vals.append(float(r))

        if not vals:
            return math.inf

        vals.sort()

        # 3-тя найменша точка замість першої — менше реакції на шум
        return vals[min(2, len(vals) - 1)]

    def obstacle_front(self):
        return self.fresh_scan() and self.front <= self.avoid_dist

    def front_clear(self):
        return self.fresh_scan() and self.front >= self.clear_dist

    def clamp(self, x, lo, hi):
        return max(lo, min(hi, x))

    def cmd(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def stop(self):
        self.cmd(0.0, 0.0)

    def log(self, text, period=0.5):
        t = self.now()

        if t - self.last_log < period:
            return

        self.last_log = t
        self.get_logger().info(text)

    # ------------------------------------------------------------
    # Decisions
    # ------------------------------------------------------------

    def choose_avoid_side(self):
        if self.right > self.left:
            self.avoid_side = -1
            side = "RIGHT"
        else:
            self.avoid_side = +1
            side = "LEFT"

        self.get_logger().info(
            f"🧭 Об'їзд: {side} | front={self.front:.2f} "
            f"left={self.left:.2f} right={self.right:.2f}"
        )

    def arrived(self):
        if not self.marker_allowed():
            return False

        if self.marker_w >= self.marker_stop_px:
            return True

        if (
            self.fresh_scan() and
            self.front <= self.stop_dist and
            self.marker_w >= self.marker_arrived_min_px and
            abs(self.dx) <= self.hard_px
        ):
            return True

        return False

    def mark_arrived(self):
        reached_id = self.marker_id

        self.stop()

        if self.remember_passed:
            self.passed.add(reached_id)

        if self.target_ids and reached_id == self.current_target():
            self.target_index += 1

        self.buzzer_pub.publish(Int32(data=1))

        self.get_logger().info(
            f"✅ ARRIVED ID={reached_id} | "
            f"front={self.front:.3f}m | marker_w={self.marker_w:.0f}px"
        )

        self.set_mode(self.ARRIVED)

    # ------------------------------------------------------------
    # Motion modes
    # ------------------------------------------------------------

    def do_search(self):
        if self.marker_allowed():
            self.set_mode(self.TRACK)
            return

        self.cmd(0.0, self.search_w)

        target = self.current_target()
        if target is None:
            self.log(f"🔎 SEARCH any | front={self.front:.2f}")
        else:
            self.log(f"🔎 SEARCH ID={target} | front={self.front:.2f}")

    def do_track(self):
        if not self.marker_allowed():
            self.set_mode(self.SEARCH)
            return

        if self.obstacle_front():
            self.choose_avoid_side()
            self.set_mode(self.AVOID)
            return

        w = 0.0
        if abs(self.dx) > self.center_px:
            w = -self.kp_w * self.dx
            w = self.clamp(w, -self.max_w, self.max_w)

        if abs(self.dx) > self.hard_px:
            v = 0.0
        else:
            v = self.track_v_fast

            if self.fresh_scan() and self.front <= self.slow_dist:
                v = self.track_v_slow

            if abs(self.dx) > self.center_px:
                v = min(v, self.track_v_slow)

        self.cmd(v, w)

        self.log(
            f"🎯 TRACK ID={self.marker_id} dx={self.dx:.0f} "
            f"w={self.marker_w:.0f}px front={self.front:.2f}"
        )

    def do_avoid(self):
        # Якщо знову бачимо потрібний маркер і попереду чисто — повертаємось на маршрут
        if self.marker_allowed() and self.front_clear():
            self.set_mode(self.TRACK)
            return

        # Якщо прямо перед роботом перешкода — крутимось у сторону об'їзду
        if self.obstacle_front():
            self.cmd(0.0, self.avoid_side * self.avoid_turn_w)
            self.log(
                f"↪️ AVOID turn | front={self.front:.2f} "
                f"left={self.left:.2f} right={self.right:.2f}"
            )
            return

        # Якщо попереду чисто — їдемо дугою навколо перешкоди
        self.cmd(
            self.avoid_v,
            -self.avoid_side * self.avoid_curve_w
        )

        self.log(
            f"🚗 AVOID curve | front={self.front:.2f} "
            f"left={self.left:.2f} right={self.right:.2f}"
        )

    def do_arrived(self):
        self.stop()

        if self.mode_age() < self.arrived_pause:
            return

        if self.target_done():
            self.stop()
            self.log("🏁 Всі маркери пройдено. Стою.")
            return

        self.set_mode(self.SEARCH)

    # ------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------

    def loop(self):
        if self.target_done():
            self.stop()
            self.log("🏁 Маршрут завершено.")
            return

        if self.arrived():
            self.mark_arrived()
            return

        if self.mode == self.SEARCH:
            self.do_search()

        elif self.mode == self.TRACK:
            self.do_track()

        elif self.mode == self.AVOID:
            self.do_avoid()

        elif self.mode == self.ARRIVED:
            self.do_arrived()

        else:
            self.stop()
            self.set_mode(self.SEARCH)

    def destroy_node(self):
        try:
            self.stop()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = MissionAuto()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
