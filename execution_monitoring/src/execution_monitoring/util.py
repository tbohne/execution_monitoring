#!/usr/bin/env python
from arox_navigation_flex.msg import drive_to_goalGoal as dtg_Goal
from osgeo import osr
from tf.transformations import quaternion_from_euler
import numpy as np
import tf2_geometry_msgs
import tf2_ros
import rospy

def parse_mission_name(mission_name):
    return mission_name.data.strip().replace(" ", "_").replace(",", "_").replace("__", "_").replace(":", "_")

def create_dtg_goal(pose, yaw):
    action_goal = dtg_Goal()
    # TODO: could we use wgs84 here?
    action_goal.target_pose.header.frame_id = "utm"

    # the coordinates in the plan are wgs84 - transform
    source = osr.SpatialReference()
    source.ImportFromEPSG(4326)
    target = osr.SpatialReference()
    target.ImportFromEPSG(32632)
    transform = osr.CoordinateTransformation(source, target)
    x, y = transform.TransformPoint(pose[1], pose[0])[0:2]  # switch lat / lon
    action_goal.target_pose.pose.position.x = x
    action_goal.target_pose.pose.position.y = y

    if yaw is None:
        yaw = np.pi * (pose[2] + 90) / 180

    q = quaternion_from_euler(0, 0, yaw)
    action_goal.target_pose.pose.orientation.x = q[0]
    action_goal.target_pose.pose.orientation.y = q[1]
    action_goal.target_pose.pose.orientation.z = q[2]
    action_goal.target_pose.pose.orientation.w = q[3]

    return action_goal

def transform_pose(tf_buffer, pose_stamped, target_frame):
    try:
        transform = tf_buffer.lookup_transform(target_frame,
                                               pose_stamped.header.frame_id,  # source frame
                                               rospy.Time(0),  # get tf at first available time
                                               rospy.Duration(1.0))  # wait for one second

        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print("Exception while trying to transform pose stamped from %s to %s", pose_stamped.header.frame_id,
              target_frame)
        raise

    return pose_transformed