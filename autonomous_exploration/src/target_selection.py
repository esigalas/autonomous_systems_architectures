#!/usr/bin/env python

import rospy
import random
import numpy 
# Class for selecting the next best target
class TargetSelection:

    # Constructor
    def __init__(self):
        pass

    def selectTarget(self, ogm, coverage, robot_pose, origin):
        
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
          # x_rand = random.randint(0, ogm.shape[0] - 1)
          # y_rand = random.randint(0, ogm.shape[1] - 1)
          # print "pos:", robot_pose[0],robot_pose[1]
          
          distances = []
          indexes = []
          #check all the rows and cols of ogm
          for (i,j), value in numpy.ndenumerate(ogm):
            # print "coverage:",i,j,val
            #if the value in ogm is free and if there is not covered space nearby in a specific range
            if value < 50 and -1 not in ogm[i-50:i+50,j-50:j+50] and 100 not in coverage[i-25:i+25,j-25:j+25]:
            # if value < 50 and 100 not in coverage[i-25:i+25,j-25:j+25]:            
              #calculate the distance from the row and column from current position
              a = numpy.array((robot_pose[0],robot_pose[1]))
              b = numpy.array((i,j))
              #keep distances and indexes of row and column in two lists
              distances.append(numpy.linalg.norm(a-b))
              index = (i,j)
              indexes.append(index)
          #find the index of the minimum distance and set them as next target

          min_index = distances.index(min(distances))
          next_target = indexes[min_index]
          found = True
          # print "indexes:",indexes[min_index]
          # if ogm[x_rand][y_rand] < 50 and coverage[x_rand][y_rand] != 100:
          #   next_target = [x_rand, y_rand]
          #   found = True
        
        # ---------------------------------------------------------------------

        return next_target

