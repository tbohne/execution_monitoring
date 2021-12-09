#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Bool
from execution_monitoring import util, config
from datetime import datetime
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

class LocalizationMonitoring:
    """
    Monitoring for localization failures -> measure quality of localization.
    -> estimate plausibility of localization based on sensor data 
    Our localization: IMU + odometry + GNSS -> Kalman filter to bring these information together (robot_localization pkg)
    """

    def __init__(self):
        rospy.Subscriber('/imu_data', Imu, self.imu_callback, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber('/odometry/filtered_odom', Odometry, self.filtered_odom_callback, queue_size=1)
        rospy.Subscriber('/odometry/gps', Odometry, self.gps_as_odom_callback, queue_size=1)

    def gps_as_odom_callback(self, gps_as_odom):
        position = gps_as_odom.pose.pose.position
        orientatoin = gps_as_odom.pose.pose.orientation
        pose_cov = gps_as_odom.pose.covariance

    def filtered_odom_callback(self, filtered_odom):
        position = filtered_odom.pose.pose.position
        orientatoin = filtered_odom.pose.pose.orientation
        pose_cov = filtered_odom.pose.covariance
        linear_twist = filtered_odom.twist.twist.linear
        angular_twist = filtered_odom.twist.twist.angular
        twist_cov = filtered_odom.twist.covariance

    def odom_callback(self, odom):
        position = odom.pose.pose.position
        orientatoin = odom.pose.pose.orientation
        pose_cov = odom.pose.covariance
        linear_twist = odom.twist.twist.linear
        angular_twist = odom.twist.twist.angular
        twist_cov = odom.twist.covariance

    def imu_callback(self, imu):
        orientation = imu.orientation
        orientation_cov = imu.orientation_covariance
        angular_velocity = imu.angular_velocity
        angular_velocity_cov = imu.angular_velocity_covariance
        linear_acceleration = imu.linear_acceleration
        linear_acceleration_cov = imu.linear_acceleration_covariance


def node():
    rospy.init_node('localization_monitoring')
    #rospy.wait_for_message('SMACH_runnning', String)
    rospy.loginfo("launch localization monitoring..")
    LocalizationMonitoring()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
