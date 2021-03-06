#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @author Tim Bohne

import actionlib
import numpy as np
import rospy
import tf2_geometry_msgs
import tf2_ros
from arox_navigation_flex.msg import drive_to_goalGoal as dtg_Goal
from mbf_msgs.msg import RecoveryAction, RecoveryGoal
from osgeo import osr
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler


def clear_costmaps():
    """
    Clears the robot's global and local costmaps to eliminate unwanted artifacts.
    """
    rospy.wait_for_service('/move_base_flex/clear_costmaps')
    clear_costmaps_service = rospy.ServiceProxy('/move_base_flex/clear_costmaps', Empty)
    rec_client = actionlib.SimpleActionClient("move_base_flex/recovery", RecoveryAction)
    rec_client.wait_for_server()

    rospy.loginfo("clearing costmaps..")
    try:
        clear_costmaps_service()
    except rospy.ServiceException as e:
        rospy.loginfo("error: %s", e)

    clear_local_costmap_goal = RecoveryGoal('clear_costmap', 3)  # concurrency_slot 3
    rec_client.send_goal(clear_local_costmap_goal)
    res = rec_client.wait_for_result()
    if res:
        rospy.loginfo("cleared costmap..")


def parse_mission_name(mission_name):
    """
    Parses the mission name from the specified message.

    @param mission_name: callback message containing the mission name to be parsed
    @return: parsed mission name
    """
    return mission_name.data.strip().replace(" ", "_").replace(",", "_").replace("__", "_").replace(":", "_")


def create_nav_goal(pose, yaw):
    """
    Creates a navigation target for the robot to pursue.

    @param pose: target pose used for the position (in lat / lng)
    @param yaw: target orientation (yaw angle)
    @return: created navigation goal
    """
    nav_goal = dtg_Goal()
    nav_goal.target_pose.header.frame_id = "utm"
    # plan coordinates in wgs84 - transform
    source = osr.SpatialReference()
    source.ImportFromEPSG(4326)
    target = osr.SpatialReference()
    target.ImportFromEPSG(32632)
    transform = osr.CoordinateTransformation(source, target)
    x, y = transform.TransformPoint(pose[1], pose[0])[0:2]  # swap lat / lng
    nav_goal.target_pose.pose.position.x = x
    nav_goal.target_pose.pose.position.y = y

    if yaw is None:
        # to radians
        yaw = np.pi * (pose[2] + 90) / 180

    q = quaternion_from_euler(0, 0, yaw)
    nav_goal.target_pose.pose.orientation.x = q[0]
    nav_goal.target_pose.pose.orientation.y = q[1]
    nav_goal.target_pose.pose.orientation.z = q[2]
    nav_goal.target_pose.pose.orientation.w = q[3]
    return nav_goal


def transform_pose(tf_buffer, pose_stamped, target_frame):
    """
    Transforms the specified pose to the specified target frame.

    @param tf_buffer: transform buffer to be used
    @param pose_stamped: pose to be transformed to another frame
    @param target_frame: frame to transform the pose to
    @return: transformed pose
    """
    pose_transformed = None
    try:
        transform = tf_buffer.lookup_transform(target_frame,
                                               pose_stamped.header.frame_id,  # source frame
                                               rospy.Time(0),  # get transform as fast as possible
                                               rospy.Duration(1.0))  # waiting time of 1s

        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.loginfo("exception during transformation from %s to %s", pose_stamped.header.frame_id, target_frame)
    return pose_transformed
