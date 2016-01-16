#!/usr/bin/env python

import rospy
import math
from robot_perception import RobotPerception
from target_selection import TargetSelection
from path_planning import PathPlanning

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

# Class for implementing the navigation module of the robot
class Navigation:

    # Constructor
    def __init__(self):

        # Initializations
        self.robot_perception = RobotPerception()
        self.target_selection = TargetSelection()
        self.path_planning = PathPlanning()

        # Check if the robot moves with target or just wanders
        self.move_with_target = rospy.get_param("calculate_target")

        # Flag to check if the vehicle has a target or not
        self.target_exists = False
        self.inner_target_exists = False

        # Container for the current path
        self.path = []
        # Container for the subgoals in the path
        self.subtargets = []

        # Container for the next subtarget. Holds the index of the next subtarget
        self.next_subtarget = 0

        # Check if subgoal is reached via a timer callback
        rospy.Timer(rospy.Duration(0.10), self.checkTarget)

        # ROS Publisher for the path
        self.path_publisher = rospy.Publisher(rospy.get_param('path_pub_topic'), \
            Path, queue_size = 10)
        # ROS Publisher for the subtargets
        self.subtargets_publisher = rospy.Publisher(rospy.get_param('subgoals_pub_topic'),\
            MarkerArray, queue_size = 10)
        # ROS Publisher for the current target
        self.current_target_publisher = rospy.Publisher(rospy.get_param('curr_target_pub_topic'),\
            Marker, queue_size = 10)


    def checkTarget(self, event):
        # Check if we have a target or if the robot just wanders
        if self.inner_target_exists == False or self.move_with_target == False or\
                self.next_subtarget == len(self.subtargets):
          return

        # Get the robot pose in pixels
        [rx, ry] = [\
            self.robot_perception.robot_pose['x_px'] - \
                    self.robot_perception.origin['x'] / self.robot_perception.resolution,\
            self.robot_perception.robot_pose['y_px'] - \
                    self.robot_perception.origin['y'] / self.robot_perception.resolution\
                    ]

        # YOUR CODE HERE ------------------------------------------------------
        # Here the next subtarget is checked for proximity. If the robot is too
        # close to the subtarget it is considered approached and the next
        # subtarget is assigned as next. However there are cases where the
        # robot misses one subtarget due to obstacle detection. Enhance the below
        # functionality by checking all the subgoals (and the final goal) for
        # proximity and assign the proper next subgoal

        # Find the distance between the robot pose and the next subtarget
        dist = math.hypot(\
            rx - self.subtargets[self.next_subtarget][0], \
            ry - self.subtargets[self.next_subtarget][1])

        if self.next_subtarget < len(self.subtargets)-1:
            dist2 = math.hypot(\
                rx - self.subtargets[self.next_subtarget+1][0], \
                ry - self.subtargets[self.next_subtarget+1][1])

            if dist2<dist:
                dist = dist2
                self.next_subtarget += 1

        # Check if distance is less than 7 px (14 cm)
        if dist < 7:
          print "Sub target reached!"
          self.next_subtarget += 1

          # Check if the final subtarget has been approached
          if self.next_subtarget == len(self.subtargets):
            print "Final goal reached!"
            self.target_exists = False

        # ---------------------------------------------------------------------

        # Publish the current target
        if self.next_subtarget == len(self.subtargets):
            return
        st = Marker()
        st.header.frame_id = "map"
        st.ns = 'as_curr_namespace'
        st.id = 0
        st.header.stamp = rospy.Time(0)
        st.type = 1 # arrow
        st.action = 0 # add
        st.pose.position.x = self.subtargets[self.next_subtarget][0]\
                * self.robot_perception.resolution + \
                self.robot_perception.origin['x']
        st.pose.position.y = self.subtargets[self.next_subtarget][1]\
                * self.robot_perception.resolution + \
                self.robot_perception.origin['y']

        st.color.r = 0
        st.color.g = 0
        st.color.b = 0.8
        st.color.a = 0.8
        st.scale.x = 0.2
        st.scale.y = 0.2
        st.scale.z = 0.2

        self.current_target_publisher.publish(st)

    # Function that seelects the next target, produces the path and updates
    # the coverage field. This is called from the speeds assignment code, since
    # it contains timer callbacks
    def selectTarget(self):
        # IMPORTANT: The robot must be stopped if you call this function until
        # it is over
        # Check if we have a map
        while self.robot_perception.have_map == False:
          return

        print "Navigation: Producing new target"
        # We are good to continue the exploration
        # Make this true in order not to call it again from the speeds assignment
        self.target_exists = True

        # Manually update the coverage field
        self.robot_perception.updateCoverage()
        print "Navigation: Coverage updated"
      
        # Gets copies of the map and coverage
        local_ogm = self.robot_perception.getMap()
        local_coverage = self.robot_perception.getCoverage()
        print "Got the map and Coverage"
  
        # Call the target selection function to select the next best goal
        target = self.target_selection.selectTarget(\
            local_ogm,\
            local_coverage,\
            self.robot_perception.robot_pose,\
            self.robot_perception.origin)
        print "Navigation: New target: " + str(target)

        # Once the target has been found, find the path to it
        # Get the global robot pose
        g_robot_pose = self.robot_perception.getGlobalCoordinates(\
            [self.robot_perception.robot_pose['x_px'],\
            self.robot_perception.robot_pose['y_px']])

        # Need to reverse the path??
        self.path = self.path_planning.createPath(\
            local_ogm,\
            g_robot_pose,\
            target)
        print "Navigation: Path for target found with " + str(len(self.path)) +\
            " points"
        # Reverse the path to start from the robot
        self.path = self.path[::-1]

        # Break the path to subgoals every 10 pixels (0.5m)
        step = 10
        n_subgoals = (int) (len(self.path)/step)
        self.subtargets = []
        for i in range(0, n_subgoals):
          self.subtargets.append(self.path[i * step])
        self.subtargets.append(self.path[-1])
        self.next_subtarget = 0
        print "The path produced " + str(len(self.subtargets)) + " subgoals"

        # Publish the path for visualization purposes
        ros_path = Path()
        ros_path.header.frame_id = "map"
        for p in self.path:
          ps = PoseStamped()
          ps.header.frame_id = "map"
          ps.pose.position.x = p[0] * self.robot_perception.resolution + \
                  self.robot_perception.origin['x']
          ps.pose.position.y = p[1] * self.robot_perception.resolution + \
                  self.robot_perception.origin['y']
          ros_path.poses.append(ps)

        self.path_publisher.publish(ros_path)

        # Publish the subtargets for visualization purposes
        ros_subgoals = MarkerArray()
        count = 0
        for s in self.subtargets:
            st = Marker()
            st.header.frame_id = "map"
            st.ns = 'as_namespace'
            st.id = count
            st.header.stamp = rospy.Time(0)
            st.type = 2 # sphere
            st.action = 0 # add
            st.pose.position.x = s[0] * self.robot_perception.resolution + \
                    self.robot_perception.origin['x']
            st.pose.position.y = s[1] * self.robot_perception.resolution + \
                    self.robot_perception.origin['y']

            st.color.r = 0
            st.color.g = 0.8
            st.color.b = 0
            st.color.a = 0.8
            st.scale.x = 0.2
            st.scale.y = 0.2
            st.scale.z = 0.2
            ros_subgoals.markers.append(st)
            count += 1

        self.subtargets_publisher.publish(ros_subgoals)

        self.inner_target_exists = True

    def velocitiesToNextSubtarget(self):
        
        [linear, angular] = [0, 0]

        # YOUR CODE HERE ------------------------------------------------------
        # The velocities of the robot regarding the next subtarget should be 
        # computed. The known parameters are the robot pose [x,y,th] from 
        # robot_perception and the next_subtarget [x,y]. From these, you can 
        # compute the robot velocities for the vehicle to approach the target.
        # Hint: Trigonometry is required

        #if we have a target
        if self.inner_target_exists: 
            #get robot x,y coordinates in pixels
            [rx, ry] = [\
                        self.robot_perception.robot_pose['x_px'] - \
                                self.robot_perception.origin['x'] / self.robot_perception.resolution,\
                        self.robot_perception.robot_pose['y_px'] - \
                                self.robot_perception.origin['y'] / self.robot_perception.resolution\
                                ]

            print "rx,ry",[rx,ry]
            dist = math.hypot(\
                rx - self.subtargets[self.next_subtarget][0], \
                ry - self.subtargets[self.next_subtarget][1])

            # print "dist : ",dist
            

            #apply trigonometry to calculate the angle between robot and target
            adjacent = rx - self.subtargets[self.next_subtarget][0]
            opposite = ry - self.subtargets[self.next_subtarget][1]
            param  = abs(adjacent)/abs(opposite)
            degrees = math.atan(param)*180/math.pi

            robot_angle = self.robot_perception.robot_pose['th']*180/math.pi
            
            #convert robot angle to 0-360 system
            if robot_angle <0:
                robot_angle = robot_angle +360
            
            #convet relative degrees of robot and target to an angle in 0-360 system
            if(rx<=self.subtargets[self.next_subtarget][0]) and (ry>=self.subtargets[self.next_subtarget][1]):
                angle  = degrees+270 
                # print "4th"
            if(rx<=self.subtargets[self.next_subtarget][0]) and (ry<=self.subtargets[self.next_subtarget][1]):
                angle = 90-degrees
                # print "1st"
            if(rx>=self.subtargets[self.next_subtarget][0]) and (ry<=self.subtargets[self.next_subtarget][1]):
                angle = degrees +90
                # print "2nd"
            if(rx>=self.subtargets[self.next_subtarget][0]) and (ry>=self.subtargets[self.next_subtarget][1]):
                angle = (90-degrees)+180
                # print "3rd"

            # print "robot angle : ",robot_angle
            # print "angle : ", angle

            if abs(angle - robot_angle) >180:
                angular = robot_angle-angle
                #scaling angular velocity
                angular = angular/1440

            else:
                angular = angle - robot_angle
                #scaling angular velocity
                angular = angular/360

            if abs(angle - robot_angle) >60:
                linear = 0
            else:
                #convert pixels to milimeters
                linear = dist*0.002

            #keep a minimum angular vel
            if angular<0.05 and angular>=0:
                angular = 0.05
            if angular>-0.05 and angular<0:
                angular = -0.05

            if linear < 0.06:
                linear = 0.06

            return [linear, angular]

        # ---------------------------------------------------------------------

        return [linear, angular]

