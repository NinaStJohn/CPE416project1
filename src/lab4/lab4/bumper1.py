import math
from enum import Enum

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class FSMState(Enum):
    FORWARD = 0
    BACK    = 1
    TURN    = 2
    STOP    = 3


class Direction(Enum):
    AHEAD   = 0
    BEHIND  = 1
    LEFT    = 2
    RIGHT   = 3


class BumperBot(Node):
    def __init__(self):
        super().__init__('bumper')

        # --- Subscribers ---
        # Change '/scan' to whatever your LaserScan topic actually is.
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # --- Publisher ---
        # Most robots accept geometry_msgs/Twist on '/cmd_vel' or a vendor-specific topic.
        # You had "/gobilda/cmd_vel" and TwistStamped; we'll use Twist for simplicity.
        self.cmd_pub = self.create_publisher(Twist, '/gobilda/cmd_vel', 10)

        # --- Timer ---
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        # --- State ---
        self.state = FSMState.FORWARD
        self.have_scan = False
        self.ranges = []           # store latest ranges (list of floats)
        self.angle_min = 0.0
        self.angle_increment = 0.0

        # --- Pre-built Twist messages ---
        self.forward_msg = Twist()
        self.forward_msg.linear.x = 0.2  # m/s forward

        self.backward_msg = Twist()
        self.backward_msg.linear.x = -0.2  # m/s backward

        self.turn_msg = Twist()
        self.turn_msg.angular.z = math.pi / 4.0  # rad/s (~45 deg/s)

        self.stop_msg = Twist()  # all zeros

    # ============ Callbacks ============
    def scan_callback(self, msg: LaserScan):
        self.ranges = list(msg.ranges) if msg.ranges is not None else []
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        self.have_scan = True

    # ============ Helpers ============
    def obstacle_ahead(self, threshold: float = 1.0, window: int = 20) -> bool:
        """Return True if there is an obstacle within `threshold` meters in the front sector.
        This uses a simple 'center window' heuristic. Adjust if your scanner's 0-angle is not front.
        """
        if not self.have_scan or not self.ranges:
            return False  # no data yet → assume clear, or you could be conservative and return True

        n = len(self.ranges)
        if n == 0:
            return False

        # Heuristic: consider the "front" to be the middle of the array.
        # (Some robots have 0° at index 0; if so, change 'center' to 0 or compute by angle.)
        center = n // 2
        start = max(0, center - window)
        end = min(n, center + window)

        sector = [r for r in self.ranges[start:end] if not math.isinf(r) and not math.isnan(r)]
        if not sector:
            return False

        return min(sector) < threshold

    # ============ Actions ============
    def do_forward(self):
        if not self.obstacle_ahead(threshold=0.6):
            self.cmd_pub.publish(self.forward_msg)
        else:
            self.state = FSMState.TURN  # switch to turn if blocked

    def do_backward(self):
        self.cmd_pub.publish(self.backward_msg)

    def do_turn(self):
        # Simple reactive turn-in-place when blocked
        self.cmd_pub.publish(self.turn_msg)
        # You could add logic to switch back to forward after some time.

    def do_stop(self):
        self.cmd_pub.publish(self.stop_msg)

    # ============ FSM ============
    def fsm_step(self):
        if self.state == FSMState.FORWARD:
            self.do_forward()
        elif self.state == FSMState.BACK:
            self.do_backward()
        elif self.state == FSMState.TURN:
            self.do_turn()
        elif self.state == FSMState.STOP:
            self.do_stop()
        else:
            self.do_stop()

    # ============ Timer ============
    def timer_callback(self):
        self.fsm_step()


def main(args=None):
    rclpy.init(args=args)
    node = BumperBot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
