#!/usr/bin/env python

import rospy
import math

from random import random
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from sonar_data_aggregator import SonarDataAggregator
from laser_data_aggregator import LaserDataAggregator
from navigation import Navigation

# Class for assigning the robot speeds 
class RobotController:

    # Constructor
    def __init__(self):
        
      # Debugging purposes
      self.print_velocities = rospy.get_param('print_velocities')

      # Where and when should you use this?
      self.stop_robot = False

      # Create the needed objects
      self.sonar_aggregation = SonarDataAggregator()
      self.laser_aggregation = LaserDataAggregator()
      self.navigation  = Navigation()

      self.linear_velocity  = 0
      self.angular_velocity = 0

      # Check if the robot moves with target or just wanders
      self.move_with_target = rospy.get_param("calculate_target")

      # The timer produces events for sending the speeds every 110 ms
      rospy.Timer(rospy.Duration(0.11), self.publishSpeeds)
      self.velocity_publisher = rospy.Publisher(\
              rospy.get_param('speeds_pub_topic'), Twist,\
              queue_size = 10)

      # Read the velocities architecture
      self.velocity_arch = rospy.get_param("velocities_architecture")
      print "The selected velocities architecture is " + self.velocity_arch

    # This function publishes the speeds and moves the robot
    def publishSpeeds(self, event):
        
      # Choose architecture
      if self.velocity_arch == "subsumption":
        self.produceSpeedsSubsumption()
      else:
        self.produceSpeedsMotorSchema()

      # Create the commands message
      twist = Twist()
      twist.linear.x = self.linear_velocity
      twist.linear.y = 0
      twist.linear.z = 0
      twist.angular.x = 0 
      twist.angular.y = 0
      twist.angular.z = self.angular_velocity

      # Send the command
      self.velocity_publisher.publish(twist)

      # Print the speeds for debuggind purposes
      if self.print_velocities == True:
        print "[L,R] = [" + str(twist.linear.x) + " , " + \
            str(twist.angular.z) + "]"

    # Produce speeds from sonars
    def produceSpeedsSonars(self):
      # Get the sonars' measurements
      front   = self.sonar_aggregation.sonar_front_range
      left    = self.sonar_aggregation.sonar_left_range
      right   = self.sonar_aggregation.sonar_right_range
      r_left  = self.sonar_aggregation.sonar_rear_left_range
      r_right = self.sonar_aggregation.sonar_rear_right_range


      # YOUR CODE HERE ------------------------------------------------------
      # Adjust the linear and angular velocities using the five sonars values
    
      # linear = front - (r_left+r_right)/4 +0.2
      # angular =  ( ((abs(left-right))/2)*left-((abs(left-right))/2)*right )/(3*front)
      
      # linear = front/(r_left+r_right) 
      # angular = (left-right)/(2*front+0.2)

      # linear =  (front - (r_left+r_right)/4 ) / (r_left+r_right)     

      #calculate linear velocity according to front sonar and rear sonars values
      linear =  (front ) / (r_left+r_right)

      #theshold robot linear velocity to a maximum
      if linear > .7:
        linear = .7

      #calculate angular velocity sign mainly according to left and right sonars
      #also try to decrease angular velocity value if we have big values from front and rear sonars
      angular = ( (left-right)+ 0.5*(r_left-r_right) )/(front+0.5*(r_left+r_right))
      
      #threshold robot angular velocity to a maximum
      if angular > .7:
        angular = .7
      elif angular < -.7:
        angular = -.7
 
      # ---------------------------------------------------------------------

      return [linear, angular]

    # Produces speeds from the laser
    def produceSpeedsLaser(self):

      # Get the laser scan
      scan   = self.laser_aggregation.laser_scan
      #mapping the laser scan in three parts : right side, middle and left side
      #geting the median value from each side as most representative

      right_side = scan[:222]
      right_side_length = len(right_side)
      
      right_side.sort()
      right_value = right_side[right_side_length/2]
      # right_value = sum(right_side)/right_side_length

      middle = scan[222:444]
      middle_length = len(middle)
      
      middle.sort()
      middle_value = middle[middle_length/2]
      # middle_value = sum(middle)/middle_length

      left_side = scan[444:]
      left_side_length = len(left_side)
      
      left_side.sort()
      left_value =  left_side[left_side_length/2]
      # left_value = sum(left_side)/right_side_length

      #calculate linear velocity according to front, right and left lazer scan median 
      linear  = (middle_value/2 ) / (left_value+right_value)
      #thresholding linear velocity to 0.7 maximum
      if linear > .7:
        linear = .7

      #calculate angular sign velocity according to left and right medians
      #decrease angular velovity value if we have big front median value
      angular = (left_value -right_value)/(2*middle_value)

      #thresholding angular velocity to .7 maximum
      if angular > .7:
        angular = .7
      elif angular < -.7:
        angular = -.7


      # YOUR CODE HERE ------------------------------------------------------
      # Adjust the linear and angular velocities using the laser scan
     
      # ---------------------------------------------------------------------

      return [linear, angular]

    # Combines the speeds into one output using a subsumption approach
    def produceSpeedsSubsumption(self):
      
      # Produce target if not existent
      if self.move_with_target == True and self.navigation.target_exists == False:
        # Create the commands message
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0 
        twist.angular.y = 0
        twist.angular.z = 0
        # Send the command
        self.velocity_publisher.publish(twist)

        self.navigation.selectTarget()

      #get sonar info
      front   = self.sonar_aggregation.sonar_front_range
      left    = self.sonar_aggregation.sonar_left_range
      right   = self.sonar_aggregation.sonar_right_range
      r_left  = self.sonar_aggregation.sonar_rear_left_range
      r_right = self.sonar_aggregation.sonar_rear_right_range

      # Get the submodules' speeds
      [l_sonar, a_sonar] = self.produceSpeedsSonars()
      [l_laser, a_laser] = self.produceSpeedsLaser()
      [l_goal, a_goal] = self.navigation.velocitiesToNextSubtarget()

      self.linear_velocity  = 0
      self.angular_velocity = 0

      # Combine the speeds following the subsumption architecture
      # YOUR CODE HERE ------------------------------------------------------
      #challenge_3
      # if front > 2 or front < 0.15:
      # # if front > 2 or front < 0.15 and r_left > 1.5 and r_right > 1.5:
      #   self.linear_velocity = l_laser 
      # else:
      #   self.linear_velocity = l_sonar

      # if (left > 2 or left < 0.15 )and (right >2 or right < 0.15):
      #   self.angular_velocity = a_laser
      # else:
      #   self.angular_velocity = a_sonar

      # ---------------------------------------------------------------------
      #Challenge_8
      if front > 2 or front < 0.15:
        if l_goal ==0:
          self.linear_velocity  = l_goal
        else:
          self.linear_velocity = l_laser*.6 
      
      else:
        if l_goal ==0:
          self.linear_velocity  = l_goal
        else:
          self.linear_velocity = l_sonar*.6

      c = (left+right)/2*3

      if (left > 2 or left < 0.15 )and (right >2 or right < 0.15):
        
        self.angular_velocity = (a_laser/c + a_goal*c )*.5

      else:
        
        self.angular_velocity = (a_sonar/c + a_goal*c)*.5

    # Combines the speeds into one output using a motor schema approach
    def produceSpeedsMotorSchema(self):
 
      # Produce target if not existent
      if self.move_with_target == True and self.navigation.target_exists == False:
        # Create the commands message
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0 
        twist.angular.y = 0
        twist.angular.z = 0
        # Send the command
        self.velocity_publisher.publish(twist)
        self.navigation.selectTarget()
      
      #get sonar info
      front   = self.sonar_aggregation.sonar_front_range
      left    = self.sonar_aggregation.sonar_left_range
      right   = self.sonar_aggregation.sonar_right_range
      r_left  = self.sonar_aggregation.sonar_rear_left_range
      r_right = self.sonar_aggregation.sonar_rear_right_range

      # Get the submodule's speeds
      [l_sonar, a_sonar] = self.produceSpeedsSonars()
      [l_laser, a_laser] = self.produceSpeedsLaser()
      [l_goal, a_goal] = self.navigation.velocitiesToNextSubtarget()

      #challenge_2
      # self.linear_velocity  = l_laser
      # self.angular_velocity = a_laser
      #challenge_1
      self.linear_velocity  = l_sonar
      self.angular_velocity = a_sonar


      # print "a laser : ",a_laser
      # print "a sonar : ",a_sonar
      # print "a goal : ",a_goal
      # Get the speeds using the motor schema approach
      # YOUR CODE HERE ------------------------------------------------------
      #challenge_4
      # self.linear_velocity  = l_laser*0.3 + l_sonar*0.7
      # self.angular_velocity = a_laser*0.6 + a_sonar*0.4
      
      # ---------------------------------------------------------------------
      #challenge_7
      # self.linear_velocity  = l_goal
      # self.angular_velocity = a_goal
      # ---------------------------------------------------------------------


      #trials
      # self.linear_velocity  = l_laser*0.1 + l_sonar*0.1 + l_goal 
      # self.angular_velocity = (a_laser*0.2 +a_sonar*0.2)*math.copysign(1,a_goal)
      # ---------------------------------------------------------------------

      # self.linear_velocity  = l_goal 

      # self.angular_velocity = (a_laser*0.6 +a_sonar*0.4)*math.copysign(1,a_goal)
      # self.angular_velocity = a_laser*0.6 +a_sonar*0.4 -l_goal
      # ---------------------------------------------------------------------

      #challenge_9
      # if l_goal == 0:
      #   self.linear_velocity  = l_laser*0.1 + l_sonar*0.1
      # else:
      #   self.linear_velocity  = l_laser*0.4 + l_sonar*0.3
      
      # c = (left+right)/2*3
      # self.angular_velocity = ( a_goal*(c/2) + (a_laser*0.6 +a_sonar*0.4)/c ) *.5
      # # self.angular_velocity = a_goal*( (front+left+right+r_right+r_left)/(5*3) ) + (a_laser*0.6 +a_sonar*0.4)/ ( (front+left+right+r_right+r_left)/(5*3) )

      # # self.angular_velocity = a_goal
      # if self.angular_velocity < 0.05 and self.angular_velocity>=0: 
      #   self.angular_velocity = 0.05
      # if self.angular_velocity > -0.05 and self.angular_velocity<0: 
      #   self.angular_velocity = -0.05 
      # ---------------------------------------------------------------------

      # print "linear : ", self.linear_velocity
      # print "angular : ", self.angular_velocity

    # Assistive functions - Do you have to call them somewhere?
    def stopRobot(self):
      self.stop_robot = True

    def resumeRobot(self):
      self.stop_robot = False
