import rclpy
from rclpy.node import Node
from enum import Enum
import math 

'''
    bumper FSM
'''

# We have to use the geometry_msgs/msg/Twist to control robots
# # A twist with reference coordinate frame and timestamp
# std_msgs/Header header
#   builtin_interfaces/msg/Time stamp
#   string frame_id
# Twist twist
#   Vector3 linear
#       float64 x
#       float64 y
#       float64 z
#   Vector3 angular
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped 

from sensor_msgs.msg import LaserScan

from std_msgs.msg import String

# -- enums ---
class FSMState(Enum):
    FORWARD = 0
    BACK    = 1
    TURN    = 2
    STOP    = 3

class direction(Enum):
    AHEAD   = 0
    BEHIND  = 1
    LEFT    = 2
    RIGHT   = 3

class BumperBot(Node):

    def __init__(self):
        # Init the node with a name (this is the name that appears when running)
        # 'ros2 node list'
        super().__init__('bumper')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription

        
        # Remember that the 'create_publisher' function takes in three arguments
        # Message Type | Topic Name | Queue Length
        # Fill in those values here

        self.publisher_ = self.create_publisher(TwistStamped, "/gobilda/cmd_vel", 10)

        # Functions running at 1Hz
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Think of this flag as a FSM, 
        # or that the turtle has two modes of operation.
        # The robot is either turning in place, or not turning in place
        # i.e. moving forward.
        self.forward = False
        self.backward = False
        self.turn = False

        # Let's create two messages to send to the robot depending
        # on the mode its in.
        # What should their type be (should the same as 'Import' above)

        # If I want the robot to move "1m forward" what should
        # the speed be, given the timer is running at 1hz?
        # (Note that values are in m/s)
        # Along which axis should I move in?
        self.forward_msg = Twist()
        self.forward_msg.linear.x = 1.0

        self.backward_msg = Twist()
        self.backward_msg.linear.x = -1.0


        # What if I want the robot to turn 90 degrees?
        # Along which axis?
        # (Note that values are in rad/s)
        self.turn_msg = Twist()
        self.turn_msg.angular.z = math.pi * 1.5

        self.ranges = []

        self.left_index_start   = 0;
        self.front_index_start  = 0;
        self.right_index_start  = 0;
        self.back_index_start   = 0;

        self.have_scan= False


    def listener_callback(self, msg):
        self.ranges = list(msg.ranges)
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        self.have_scan = True
        
    # -- detect if lidar is 1m away
    def detect(self, direction, distance):
        if (direction == direction.AHEAD):
            index = self.front_index_start
        elif (direction == direction.BEHIND):
            index = self.back_index_start
        elif (direction == direction.LEFT):
            index = self.left_index_start
        elif (direction == direction.RIGHT):
            index = self.right_index_start

        for i in range(10):
            if self.ranges[i+index] < distance:
                return True
        return False

    def move_forward(self):
        if not self.detect(direction.AHEAD, 1):
            # post move ahead
            self.publisher_.publish(self.forward_msg)

    def move_backward(self):
        if not self.detect(direction.BEHIND, 1):
            # post move backwards
            # for 1 second? timer
            self.publisher_.publish(self.backward_msg)

    def turn(self):
        if self.detect(direction.AHEAD, 1):
            #post turn
            self.publisher_.publish(self.turn_msg)


    def stop(self):
        # post stop
        self.forward = False
        self.backwards = False
        self.turn = False


    def FSM(self):
        # forward, back, turn, stop
        state = FSMState.FORWARD

        if state == FSMState.FORWARD:
            self.move_forward()
        elif state == FSMState.BACK:
            self.move_backward()
        elif state == FSMState.TURN:
            self.turn()
        elif state == FSMState.STOP:
            self.stop()
        else:
            # default
            self.stop()

    # Callback for the events
    def timer_callback(self):
        if not self.have_scan:
            return
        self.FSM()


def main(args=None):
    rclpy.init(args=args)

    bumper = BumperBot()

    # Spin function *important*
    # makes sure that the program does not terminate
    # immediately
    rclpy.spin(bumper)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bumper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


