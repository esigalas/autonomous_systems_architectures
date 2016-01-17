#!/usr/bin/env python

import rospy
import tf
import numpy

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# Class implementing the robot perception: Reading the map, the coverage map
# and the robot pose
class RobotPerception:

    # Constructor
    def __init__(self):

        # Flags for debugging and synchronization
        self.print_robot_pose = False
        self.have_map = False
        self.map_token = False
        self.map_compute = False

        # Holds the occupancy grid map
        self.ogm = 0
        self.ogm_copy = 0

        # Holds the ogm info for copying reasons -- do not change
        self.ogm_info = 0

        # Holds the robot's total path
        self.robot_trajectory = []

        # Holds the coverage information. This has the same size as the ogm
        # If a cell has the value of 0 it is uncovered
        # In the opposite case the cell's value will be 100
        self.coverage = 0

        # Holds the resolution of the occupancy grid map
        self.resolution = 0.2
        
        # Origin is the translation between the (0,0) of the robot pose and the
        # (0,0) of the map
        self.origin = {}
        self.origin['x'] = 0
        self.origin['y'] = 0

        # Initialization of robot pose
        # x,y are in meters
        # x_px, y_px are in pixels
        self.robot_pose = {}
        self.robot_pose['x'] = 0
        self.robot_pose['y'] = 0
        self.robot_pose['th'] = 0
        self.robot_pose['x_px'] = 0
        self.robot_pose['y_px'] = 0

        # Use tf to read the robot pose
        self.listener = tf.TransformListener()

        # Read robot pose with a timer
        rospy.Timer(rospy.Duration(0.11), self.readRobotPose)

        # ROS Subscriber to the occupancy grid map
        ogm_topic = rospy.get_param('ogm_topic')
        rospy.Subscriber(ogm_topic, OccupancyGrid, self.readMap) 

        # Publisher of the robot trajectory
        robot_trajectory_topic = rospy.get_param('robot_trajectory_topic')
        self.robot_trajectory_publisher = rospy.Publisher(robot_trajectory_topic,\
                Path, queue_size = 10)

        # Publisher of the coverage field
        coverage_pub_topic = rospy.get_param('coverage_pub_topic')
        self.coverage_publisher = rospy.Publisher(coverage_pub_topic, \
            OccupancyGrid, queue_size = 10)

        # Get the frames from the param file
        self.map_frame = rospy.get_param('map_frame')
        self.base_footprint_frame = rospy.get_param('base_footprint_frame')

    # Getter for OGM. Must use flags since its update is asynchronous
    def getMap(self):
      print "Robot perception: Map requested"
      # The map is being processed ... waiting
      while self.map_compute == True:
        pass

      # Locking the map
      self.map_token = True
      # Copying it
      cp = numpy.copy(self.ogm)
      # Unlocking it
      self.map_token = False

      # Return the copy
      return cp

    # Getter for Coverage
    def getCoverage(self):
      return numpy.copy(self.coverage)

    # Reading the robot pose
    def readRobotPose(self, event):
        try:
            # Reads the robot pose from tf
            (translation, rotation) = self.listener.lookupTransform\
                    (self.map_frame, self.base_footprint_frame, rospy.Time(0))
        # Catch the exception if something is wrong
        except (tf.LookupException, tf.ConnectivityException, \
                tf.ExtrapolationException):
            # Just print the error to try again
            print "Error in tf"
            return

        # Updating the robot pose
        self.robot_pose['x'] = translation[0]
        self.robot_pose['y'] = translation[1]
        self.robot_pose['x_px'] = int(self.robot_pose['x'] / self.resolution)
        self.robot_pose['y_px'] = int(self.robot_pose['y'] / self.resolution)

        # Getting the Euler angles
        angles = tf.transformations.euler_from_quaternion(rotation)
        self.robot_pose['th'] = angles[2]

        # For debugging purposes
        if self.print_robot_pose == True:
            print self.robot_pose

        # YOUR CODE HERE ------------------------------------------------------
        # Update the robot path. This is a python list. Since we use the path
        # only for updating the coverage, try not to add duplicates
        # Each point should be in the form of [x,y] (theta does not concern us)
        position = (self.robot_pose['x'],self.robot_pose['y'])
        
        if position not in self.robot_trajectory:
            self.robot_trajectory.append(position)
        
        # ---------------------------------------------------------------------

        t_path = Path()
        t_path.header.frame_id = "map"
        for p in self.robot_trajectory:
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x = p[0]
            ps.pose.position.y = p[1]
            t_path.poses.append(ps)
        self.robot_trajectory_publisher.publish(t_path)

    # Getting the occupancy grid map
    def readMap(self, data):
        # OGM is a 2D array of size width x height
        # The values are from 0 to 100
        # 0 is an unoccupied pixel
        # 100 is an occupied pixel
        # 50 is the unknown

        # Locking the map
        self.map_compute = True

        # Reading the map pixels
        self.ogm_info = data.info
        self.ogm = numpy.zeros((data.info.width, data.info.height), dtype = numpy.int)
        for x in range(0, data.info.width):
          for y in range(0, data.info.height):
            self.ogm[x][y] = data.data[x + data.info.width * y]
        
        # Get the map's resolution - each pixel's side in meters
        self.resolution = data.info.resolution
        
        # Get the map's origin
        self.origin['x'] = data.info.origin.position.x
        self.origin['y'] = data.info.origin.position.y
        
        # Keep a copy
        self.ogm_copy = numpy.copy(self.ogm)

        # Unlock the map
        self.map_compute = False
        
        # If it is copied wait ...
        while self.map_token == True:
          pass
        
        # This is for the navigation
        if self.have_map == False:
          self.have_map = True
          print "Robot perception: Map initialized"

    # Function that updates the coverage field
    def updateCoverage(self):
        
        # Reinitialize coverage map
        ogm_shape = self.ogm.shape
        self.coverage = numpy.zeros(ogm_shape)
        
        # YOUR CODE HERE ------------------------------------------------------
        # Update the coverage field using the self.robot_path veriable.
        # We suppose that in every pose the robot covers an area of 2m x 2m
        # around it
        # 0 is for the uncovered, 100 is for the covered
        # PS. Try to make it fast :)
        # PS2. Do not have coverage values on obstacles or unknown space!
        # If done correctly, the coverage will appear purple in rviz
    
        area_size = 1 / self.resolution
        for path_coordinates in self.robot_trajectory:
            robot_pose_x_px = int(path_coordinates[0]/ self.resolution)
            robot_pose_y_px = int(path_coordinates[1]/ self.resolution)
            [posx,posy] =  [\
                            robot_pose_x_px - \
                                self.origin['x']   / self.resolution,\
                            robot_pose_y_px - \
                                self.origin['y'] / self.resolution\
                                ]
            print posx,posy
            # print self.origin['x'],self.origin['y']
            
            x_min_size = int(posx)-int(area_size)
            x_max_size = int(posx)+int(area_size)
            y_min_size = int(posy)-int(area_size)
            y_max_size = int(posy)+int(area_size)
            mask1 = self.ogm[x_min_size:x_max_size,y_min_size:y_max_size] <50
            mask2 = self.ogm[x_min_size:x_max_size,y_min_size:y_max_size] >=0
            mask = mask2+mask1
            # print mask 
            self.coverage[ x_min_size:x_max_size,y_min_size:y_max_size] = numpy.where(mask,100,0)

          
        # ---------------------------------------------------------------------

        # Publishing coverage ogm to see it in rviz
        coverage_ogm = OccupancyGrid()
        coverage_ogm.header.frame_id = "map"
        coverage_ogm.info = self.ogm_info
        coverage_ogm.data = numpy.zeros(self.ogm_info.width * self.ogm_info.height)
        for i in range(0, self.ogm_info.width):
            for j in range(0, self.ogm_info.height):
                coverage_ogm.data[i + self.ogm_info.width * j] = self.coverage[i][j]

        self.coverage_publisher.publish(coverage_ogm)
    
    # Transforms relative coordinates to global
    def getGlobalCoordinates(self, p, with_resolution = True):
      # If we want coordinates in pixels
      if with_resolution == True:
        return [\
            p[0] - int(self.origin['x'] / self.resolution),\
            p[1] - int(self.origin['y'] / self.resolution)\
            ]
      # If we want the coordinates in meters
      else:
        return [\
            p[0] - self.origin['x'],\
            p[1] - self.origin['y']\
            ]

