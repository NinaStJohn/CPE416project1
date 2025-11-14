import rclpy
from rclpy.node import Node

# Needed imports
from math import cos, sin, isfinite
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion

# ssh calpoly@10.40.68.3
# source /opt/ros/humble/setup.bash
# source /gobilda_sim/ros_ws/install/setup.bash   <----- different 
# ros2 pgk create project1                        <----- make package (in src)
# edit setup.py                                         'occupancy_grid = project1.occupancy_grid:main'
# colcon build --symlink-install                  <----- build (ros_ws)
# ros2 run project1 occupancy                     <----- add debud flag if needed
# foxglove ws://10.40.68.3:8765                   <----- allow unsafe scripts



class LocalCostmap(Node):
    def __init__(self):
        super().__init__('local_costmap')

        # Map config
        self.map_width = 300            # cells
        self.map_height = 300           # cells
        self.map_resolution = 0.05      # resolution * cells  => 15 m x 15 m map

        # Occupancy conventions
        self.UNKNOWN = -1
        self.FREE = 0
        self.OCCUPIED = 100

        # Outgoing OccupancyGrid message (we'll fill .info and .header)
        self.publish_map = OccupancyGrid()
        self._init_map_info()

        # Precompute robot's cell (center of grid)
        self.cx = self.map_width  // 2
        self.cy = self.map_height // 2

        # sub to laser scan
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10)
        self.subscription

        # pub to nav_msgs/OccupancyGrid
        self.publisher_ = self.create_publisher(OccupancyGrid, "nav_msgs/OccupancyGrid", 10)

        # Functions running at 1Hz
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.build_occupancy_grid)
        self.have_scan= False

        self.angle_min = 0
        self.angle_max = 360
        self.angle_increment = 1000

        self.grid_size = self.map_width * self.map_height
        self.grid = [self.UNKNOWN] * self.grid_size
        self.publish_map.data = list(self.grid)  # initial


    ''' Initialize static OccupancyGrid.info and origin so the robot is at the map center. '''
    def _init_map_info(self):
        self.publish_map.info.resolution = self.map_resolution
        self.publish_map.info.width = self.map_width
        self.publish_map.info.height = self.map_height

        # Place (0,0) of the grid so that the robot (base frame origin) is at the center cell.
        # That means the map origin (bottom-left corner in world coords) is shifted negative by half-size.
        origin = Pose()
        origin.position.x = - (self.map_width * self.map_resolution) / 2.0
        origin.position.y = - (self.map_height * self.map_resolution) / 2.0
        origin.position.z = 0.0
        origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.publish_map.info.origin = origin

        

    # returns true if out of bounds
    def out_of_bounds(self,x,y):
        if (x < 0 or x > self.map_width) or (y < 0 or y > self.map_height):
            return True
        return False

    ''' Meters in robot frame -> map indices (mx, my). Returns None if out of bounds. '''
    # Input: (x, y) coordinates of a point in the Cartesian plane
    # Output: Corresponding cell in the occupancy grid
    def world_to_map(self, x_m, y_m):
        mx = int(self.cx + x_m/self.map_resolution)
        my = int(self.cy+ y_m/self.map_resolution)          # calc exact location
        if self.out_of_bounds(mx, my):
            return None
        return mx, my

    # Bresenham's Line Algorithm: inclusive endpoints
    # Input: 2-points on the Cartesian plane (i.e. a line)
    # (The first point is the robot origin, while the sencond is a single beam's endpoint)
    # Output: All the cells that that the beam crosses. i.e. the free cells.
    def bresenham_line_algorithm(self, x0, y0, x1, y1):   
        dx = abs(x1 - x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0)
        sy = 1 if y0 < y1 else -1
        error = dx + dy

        free_space_cells = []
        while True:
            free_space_cells.append((x0,y0))
            
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



        return free_space_cells

    ''' Cache the most recent LaserScan'''
    def laser_callback(self, msg: LaserScan):
        self.ranges = list(msg.ranges)
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment
        self.have_scan = True
        return

    # Input: x & y coordinates;
    # Output: list of free cells along the ray (excludes the last cell)
    def raytrace(self, x_cell, y_cell):
        # Compute free cells for a single beam
        # This function should call self.bresenham_line_algorithm
        idx = self.idx(self, x_cell, y_cell)
        self.grid(idx) = self.OCCUPIED
        free_cells = self.bresenham_line_algorithm(self.cx , self.cy, x_cell, y_cell)

        return free_cells[:-1]
    
    def idx(self, x, y):
        return y * self.map_width + x


    ''' Build and Publish the Occupancy Grid from the most recent LiDAR Scan '''
    def build_occupancy_grid(self):
        # First, check that the scan data is ready
        # Second, iterate through beams to create the map!
        
        if not self.have_scan:
            return
        
        all_free_cells = []
        
        for i in range(len(self.ranges)):
            theda = self.angle_min + self.angle_increment * i  # NOTE: INDEX 0 is AHEAD, cuz
            m_x = self.ranges[i]* sin(theda)
            m_y = self.ranges[i]* cos(theda)
            mx, my = self.world_to_map(m_x, m_y)
            all_free_cells.append(self.raytrace(mx, my))

        for cell in all_free_cells:
            idx = self.idx(self, cell[0], cell[1])
            self.grid[idx] = self.FREE


        self.publish_map.data = self.grid
        # Populate OccupancyGrid message
        self.publish_map.header.stamp = self.get_clock().now().to_msg()
        # Set frame to match your robot frame that LaserScan is in (commonly "base_link" or "laser")
        # The simulation and hardware will have different names for this frame
        self.publish_map.header.frame_id = 'diff_drive/lidar_link'
        # Publish
        self.publisher_.publish(self.publish_map)

    def timer_callback(self):
        if not self.have_scan:
            return
        self.build_occupancy_grid()


def main(args=None):
    rclpy.init(args=args)
    
    # Node creation and spin
    local_costmap = LocalCostmap()
    rclpy.spin(local_costmap)
    
    # Node cleanupidx = self.idx(self, cell[0], cell[1])
    local_costmap.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
