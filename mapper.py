#Name: SHEHRYAR
#ID: P2952028
#!/usr/bin/env python
""" Simple occupancy-grid-based mapping with odometry and laser scan

Subscribed topics:
/scan
/odom

Published topics:
/map
/map_metadata
"""
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import numpy as np
import math

class Map(object):
    """ Stores a 2D occupancy grid """
    def __init__(self, origin_x=-2.5, origin_y=-2.5, resolution=.1, width=50, height=50):
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.resolution = resolution
        self.width = width 
        self.height = height 
        self.grid = np.zeros((height, width))  # Occupancy values from 0.0 to 1.0

    def to_message(self):
        """ Convert the numpy grid to OccupancyGrid ROS message """
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height
        grid_msg.info.origin = Pose(Point(self.origin_x, self.origin_y, 0),
                                    Quaternion(0, 0, 0, 1))
        # Flatten grid and scale to 0-100 for ROS
        flat_grid = self.grid.reshape((self.grid.size,)) * 100
        grid_msg.data = list(np.round(flat_grid).astype('int8'))
        return grid_msg

    def set_cell(self, x, y, val):
        """ Set the value of a cell in the occupancy grid """
        i = int(round((x - self.origin_x)/self.resolution))
        j = int(round((y - self.origin_y)/self.resolution))
        if 0 <= i < self.width and 0 <= j < self.height:
            self.grid[j, i] = val  # j=row, i=column

class Mapper(object):
    """ Creates a map from laser scan and odometry """
    
    def __init__(self):
        rospy.init_node('mapper')
        self._map = Map()

        # Robot pose (updated from /odom)
        self.x_r = 0.0
        self.y_r = 0.0
        self.theta_r = 0.0

        # Subscribers
        rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Publishers
        self._map_pub = rospy.Publisher('map', OccupancyGrid, latch=True)
        self._map_data_pub = rospy.Publisher('map_metadata', MapMetaData, latch=True)
        
        rospy.spin()

    def odom_callback(self, msg):
        """ Step 6: Update robot pose from /odom """
        self.x_r = msg.pose.pose.position.x
        self.y_r = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        (_, _, self.theta_r) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        # Debug: print robot pose
        rospy.loginfo("Robot Pose: x=%.2f, y=%.2f, yaw=%.2f", self.x_r, self.y_r, self.theta_r)

    def scan_callback(self, scan):
        """ Steps 8-12: Update the occupancy grid using laser scan """
        print('--------------------------------')
        print ('Number of laser beams:', len(scan.ranges))

        for i in range(len(scan.ranges)):
            r = scan.ranges[i]

            # Only consider valid readings
            if scan.range_min < r < scan.range_max:
                # Convert laser beam to local coordinates
                theta_s = scan.angle_min + i * scan.angle_increment  # angle of the i-th beam
                x_s = r * math.cos(theta_s)
                y_s = r * math.sin(theta_s)

                # Transform local coordinates to global coordinates
                x_prime = math.cos(self.theta_r) * x_s - math.sin(self.theta_r) * y_s
                y_prime = math.sin(self.theta_r) * x_s + math.cos(self.theta_r) * y_s
                x_global = x_prime + self.x_r
                y_global = y_prime + self.y_r

                # Step 11: Convert global coordinates to grid indices and mark obstacle
                self._map.set_cell(x_global, y_global, 1.0)

        # see some changes instantly
        self._map.grid[0, 1] = 0.9
        self._map.grid[0, 2] = 0.7
        self._map.grid[1, 0] = 0.5
        self._map.grid[2, 0] = 0.3

        #  Publish updated map
        rospy.loginfo("Scan processed, publishing updated map.")
        self.publish_map()

    def get_indix(self, x, y):
        """ Helper function to get grid indices from global coordinates """
        x = x - self._map.origin_x
        y = y - self._map.origin_y
        i = int(round(x / self._map.resolution))
        j = int(round(y / self._map.resolution))
        return i, j

    def publish_map(self):
        """ Publish the occupancy grid """
        grid_msg = self._map.to_message()
        self._map_data_pub.publish(grid_msg.info)
        self._map_pub.publish(grid_msg)


if __name__ == '__main__':
    try:
        m = Mapper()
    except rospy.ROSInterruptException:
        pass




