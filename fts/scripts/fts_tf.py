#!/usr/bin/env python
# coding=utf-8

"""
This node does the transform tree setup for my robot 'minibot'.
It also publishes these transforms into ROS.
Author:  Markus Knapp, 2017
Website: https://direcs.de
"""

import logging
import rospy
import tf

if __name__ == '__main__':
    # init node
    rospy.init_node('robot_tf_broadcaster', anonymous=False)
    rospy.loginfo("robot_tf_broadcaster started.")

    # the broacaster (publisher)
    #
    # see also http://wiki.ros.org/tf/Tutorials/Adding%20a%20frame%20%28Python%29
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        """ base_link transform (base_link connected 1:1 to base_footprint. NEEDED?) """
        br.sendTransform((0.0, 0.0, 0.0),       # translation (x,y,z) in meters
                         (0.0, 0.0, 0.0, 1.0),  # rotation
                         rospy.Time.now(),      # time
                         "base_link",      # parent node
                         "base_footprint")           # child node

        """ base_laser transform (the laser rangefinder) """
        br.sendTransform((0.6, 0.0, 0.01),   # translation (x,y,z) in meters
                         (0.0,  0.0, 0.0, 1.0), # rotation
                         rospy.Time.now(),      # time
                         "base_link",           # parent node
                         "base_laser")          # child node

                         """ base_laser2 transform (the laser rangefinder) """
        br.sendTransform((0.6, 0.0, 0.01),   # translation (x,y,z) in meters
                         (0.0,  0.0, 0.0, 1.0), # rotation
                         rospy.Time.now(),      # time
                         "base_link",           # parent node
                         "base_laser2")          # child node

       

        # "base_link" is a more common name for a chassis in ROS.
        #
        # parent > child relation. earth > map > odom > base_link > base_laser
        # see also http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF
        # and also http://www.ros.org/reps/rep-0105.html#relationship-between-frames

        rate.sleep()