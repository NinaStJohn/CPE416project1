import rclpy
from rclpy.node import Node
from enum import Enum
import math
import numpy as np

'''
    occupancy grid builder
'''

from sensor_msgs.msg import LaserScan

from nav_msgs.msg import OccupancyGrid

from nav_msgs.msg import MapMetaData


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



class OccupancyGrid(Node):

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

        # note that (0,0) is in the top left corner
        self.map_size = (15,15)
        self.robot_start = (0,0)
        self.resolution = 1         # 1 meter
        
        
        # Remember that the 'create_publisher' function takes in three arguments
        # Message Type | Topic Name | Queue Length
        # Fill in those values here


        # pub to nav_msgs/OccupancyGrid - get type
        self.publisher_ = self.create_publisher(OccupancyGrid, "nav_msgs/OccupancyGrid", 10)

        # Functions running at 1Hz
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


        self.have_scan= False

    def build_map(self):
        # Initialize a 30x30 map with all cells set to 0 (free space)
        # NOTE: the resolution of the map of one cell = 0.05m
        # NOTE: the index (0,0) is in the top left corner went visualized using PyGame
        occupancy_grid = np.full(self.map_size, -1, dtype=int)

        return occupancy_grid


    def listener_callback(self, msg):
        self.ranges = list(msg.ranges)
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        self.have_scan = True

    def plot(self, x, y):
        self.occupancy_grid[x,y] = 100
        return
        
    # 1. compute direction of x and y coords
    # 2. computer error term
    # 3. start the while loop by illuminate(x,y)
    # 4. Use the error term to decide whether to increment either the x coordinate, the y coordinate, or both
    # 5. Increment/decrement the error term as necessary

    def plotLine(x0, y0, x1, y1):
        dx = abs(x1 - x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0)
        sy = 1 if y0 < y1 else -1
        error = dx + dy

        while True:
            plot(x0, y0)
            e2 = 2 * error
            if e2 >= dy:
                if x0 == x1:
                    break
                error += dy
                x0 += sx
            if e2 <= dx:
                if y0 == y1:
                    break
                error += dx
                y0 += sy


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

    occupancy = OccupancyGrid()

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


