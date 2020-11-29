#!/usr/bin/env python

"""
    pick_and_place.py - Version 0.1 2014-08-01
    
    Command the gripper to grasp a target object and move it to a new
    location, all while avoiding simulated obstacles.
    
      Before running, set environment variable TURTLEBOT_ARM1 to either:
       turtlebot - for original turtlebot arm
       pincher - for PhantomX Pincher arm
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel. All rights reserved.
    
    Adapted to the Turtlebot arm by Jorge Santos

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import sys
import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import moveit_commander

from math import atan2
from copy import deepcopy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from moveit_msgs.msg import Grasp, GripperTranslation
from moveit_msgs.msg import MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from tf.transformations import quaternion_from_euler, quaternion_matrix, quaternion_from_matrix

GROUP_NAME_ARM = 'arm'
GROUP_NAME_GRIPPER = 'gripper'

GRIPPER_FRAME = 'gripper_link'
GRIPPER_JOINT_NAMES = ['gripper_joint']
GRIPPER_EFFORT = [1.0]
GRIPPER_PARAM = '/gripper_controller'

REFERENCE_FRAME = 'base_link'
ARM_BASE_FRAME = 'arm_base_link'

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node('moveit_demo')

        # We need a tf2 listener to convert poses into arm reference base
        try:
            self._tf2_buff = tf2_ros.Buffer()
            self._tf2_list = tf2_ros.TransformListener(self._tf2_buff)
        except rospy.ROSException as err:
            rospy.logerr("MoveItDemo: could not start tf buffer client: " + str(err))
            raise err

        self.gripper_opened = [rospy.get_param(GRIPPER_PARAM + "/max_opening") - 0.01]
        self.gripper_closed = [rospy.get_param(GRIPPER_PARAM + "/min_opening") + 0.01]
        self.gripper_neutral = [rospy.get_param(GRIPPER_PARAM + "/neutral",
                                                (self.gripper_opened[0] + self.gripper_closed[0])/2.0) ]
        
        self.gripper_tighten = rospy.get_param(GRIPPER_PARAM + "/tighten", 0.0) 

        # Use the planning scene object to add or remove objects
        self.scene = PlanningSceneInterface()

        # Create a scene publisher to push changes to the scene
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=10)

        # Create a publisher for displaying gripper poses
        self.gripper_pose_pub = rospy.Publisher('target_pose', PoseStamped, queue_size=10)

        # Create a dictionary to hold object colors
        self.colors = dict()

        # Initialize the move group for the right arm
        arm = MoveGroupCommander(GROUP_NAME_ARM)

        # Initialize the move group for the right gripper
        gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)

        # Get the name of the end-effector link
        end_effector_link = arm.get_end_effector_link()

        # Allow some leeway in position (meters) and orientation (radians)
        arm.set_goal_position_tolerance(0.04)
        arm.set_goal_orientation_tolerance(0.01)

        # Allow replanning to increase the odds of a solution
        arm.allow_replanning(True)

        # Set the right arm reference frame
        arm.set_pose_reference_frame(REFERENCE_FRAME)

        # Allow 5 seconds per planning attempt
        arm.set_planning_time(5)

        # Set a limit on the number of pick attempts before bailing
        max_pick_attempts = 3

        # Set a limit on the number of place attempts
        max_place_attempts = 3
        rospy.loginfo("Scaling for MoveIt timeout=" + str(rospy.get_param('/move_group/trajectory_execution/allowed_execution_duration_scaling')))

        # Give each of the scene objects a unique name
        table_id = 'table'
        box1_id = 'box1'
        box2_id = 'box2'
        target_id = 'target'
        tool_id = 'tool'

        # Remove leftover objects from a previous run
        self.scene.remove_world_object(table_id)
        self.scene.remove_world_object(box1_id)
        self.scene.remove_world_object(box2_id)
        self.scene.remove_world_object(target_id)
        self.scene.remove_world_object(tool_id)

        # Remove any attached objects from a previous session
        self.scene.remove_attached_object(GRIPPER_FRAME, target_id)

        # Give the scene a chance to catch up
        rospy.sleep(1)
 
        # Start the arm in the "arm_up" pose stored in the SRDF file
        rospy.loginfo("Set Arm: down")
        arm.set_named_target('down')
        if arm.go() != True:
            rospy.logwarn("  Go failed")

        # Move the gripper to the open position
        # rospy.loginfo("Set Gripper: Open " +  str(self.gripper_opened))
        # gripper.set_joint_value_target(self.gripper_opened)
        #if gripper.go() != True:
        #    rospy.logwarn("  Go failed")

if __name__ == "__main__":
    MoveItDemo()
