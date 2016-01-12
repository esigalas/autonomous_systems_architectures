#!/usr/bin/env python

import rospy
import random

# Class for selecting the next best target
class TargetSelection:

    # Constructor
    def __init__(self):
        pass

    def selectTarget(self, ogm, coverage, robot_pose):
        
        # The next target in pixels
        next_target = [0, 0] 
        
        # YOUR CODE HERE ------------------------------------------------------
        # Here you must select the next target of the robot. The next target
        # should exist in unoccupied and uncovered space. Thus, you must use the
        # ogm and coverage variables or / and the robot pose. The easier way is to
        # randomly select points of the map until one such point can be a target
        # Of course you should try something smarter...!
        
        found = False
        while not found:
          x_rand = random.randint(0, ogm.shape[0] - 1)
          y_rand = random.randint(0, ogm.shape[1] - 1)
          # x_rand = 260
          # y_rand = 290
          # x_rand = 300
          # y_rand = 250
          # x_rand = 275
          # y_rand = 240
          
          #case 4
          # x_rand = 241
          # y_rand = 230

          #case 2
          # x_rand = 260
          # y_rand = 275
          # y_rand = 300

          #case 1
          # x_rand = 270
          # y_rand = 240

          #case 3
          # x_rand = 241
          # y_rand = 265

          if ogm[x_rand][y_rand] < 50 and coverage[x_rand][y_rand] != 100:
            next_target = [x_rand, y_rand]
            found = True
        
        # ---------------------------------------------------------------------

        return next_target

