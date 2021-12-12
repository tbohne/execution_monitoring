#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String, Bool
from execution_monitoring import util, config
from datetime import datetime
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf
from actionlib_msgs.msg import GoalStatusArray, GoalStatus

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
        rospy.Subscriber('/move_base_flex/exe_path/status', GoalStatusArray, self.mbf_status_callback, queue_size=1)
        self.imu_data = None
        self.odom_data = None
        self.odom_filtered_data = None
        self.gps_as_odom_data_latest = None
        self.gps_as_odom_data_second_latest = None
        self.initial_GPS = None
        self.initial_odom = None

        self.mbf_status = None
        self.localization_monitoring()

    def mbf_status_callback(self, mbf_status):
        if len(mbf_status.status_list) > 0:
            # the last element in the list is the latest (most recent)
            self.mbf_status = mbf_status.status_list[-1].status

    def localization_monitoring(self):
        while not rospy.is_shutdown():
            self.monitor_imu()
            self.monitor_odom(True)
            self.monitor_odom(False)
            self.relative_monitoring()
            rospy.sleep(2)

    def relative_monitoring(self):

        if self.odom_data is not None and self.odom_filtered_data is not None and self.gps_as_odom_data_latest is not None:
            # IMU orientation
            # odom position + orientation
            # GNSS position

            # -> can compare position of odom and GNSS
            # ->             orientation of IMU and odom
            
            #self.orientation_z_component_monitoring()
            self.odom_and_gnss_dist_diverge_monitoring()

    def odom_and_gnss_dist_diverge_monitoring(self):
        rospy.loginfo("################################################################")
        rospy.loginfo("odom position: %s", self.odom_data.pose.pose.position)
        rospy.loginfo("odom filtered position: %s", self.odom_filtered_data.pose.pose.position)
        rospy.loginfo("gnss position: %s", self.gps_as_odom_data_latest.pose.pose.position)

        gps_x = self.gps_as_odom_data_latest.pose.pose.position.x
        gps_y = self.gps_as_odom_data_latest.pose.pose.position.y

        odom_x = self.odom_filtered_data.pose.pose.position.x
        odom_y = self.odom_filtered_data.pose.pose.position.y

        if self.initial_GPS is None:
            self.initial_GPS = self.gps_as_odom_data_latest.pose.pose.position
        if self.initial_odom is None:
            self.initial_odom = self.odom_filtered_data.pose.pose.position
        
        if self.initial_GPS is not None and self.initial_odom is not None:
            odom_dist = math.sqrt((self.initial_odom.x - odom_x) ** 2 + (self.initial_odom.y - odom_y) ** 2)
            gps_dist = math.sqrt((self.initial_GPS.x - gps_x) ** 2 + (self.initial_GPS.y - gps_y) ** 2)
            rospy.loginfo("2D dist between initial filtered odom and current: %s", odom_dist)
            rospy.loginfo("2D dist between initial GPS and current: %s", gps_dist)

            if abs(odom_dist - gps_dist) > 0.5:
                # TODO: robot_info
                rospy.loginfo("GNSS (initial-current) and odometry (initial-current) distances are slightly diverging -> indicator for minor localization issue")
            if abs(odom_dist - gps_dist) > 2.0:
                # TODO: contingency
                rospy.loginfo("GNSS (initial-current) and odometry (initial-current) distances are diverging quite a bit -> indicator for localization issue")
            if abs(odom_dist - gps_dist) > 4.0:
                # TODO: contingency
                rospy.loginfo("GNSS (initial-current) and odometry (initial-current) distances are diverging quite heavily -> indicator for localization issue")

        rospy.loginfo("################################################################")

    def orientation_z_component_monitoring(self):
        """
        Arguably the most important component of the orientation.
        GNSS interpolation of orientation (z component).
        Just comparing with IMU and filtered odometry. Odometry itself is always too bad.
        """
        rospy.loginfo("################################################################")
        rospy.loginfo("imu orientation z: %s", self.imu_data.orientation.z)
        rospy.loginfo("odom orientation z: %s", self.odom_data.pose.pose.orientation.z)
        rospy.loginfo("odom filtered orientation z: %s", self.odom_filtered_data.pose.pose.orientation.z)

        # interpolate orientation based on two GPS points
        x1 = self.gps_as_odom_data_latest.pose.pose.position.x
        x2 = self.gps_as_odom_data_second_latest.pose.pose.position.x
        y1 = self.gps_as_odom_data_latest.pose.pose.position.y
        y2 = self.gps_as_odom_data_second_latest.pose.pose.position.y

        vector_x = x1 - x2
        vector_y = y1 - y2
        
        dist = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
        if dist > config.DIST_THRESH_FOR_INTERPOLATION_BETWEEN_GNSS_POS:
            rospy.loginfo("gnss interpolation distance between points: %s", dist)
            angle = math.atan2(vector_y, vector_x)
            quaternion = tf.transformations.quaternion_from_euler(0, 0, angle)
            gnss_orientation_z = quaternion[2]
            rospy.loginfo("gnss orientation interpolation z component: %s", gnss_orientation_z)

            if abs(gnss_orientation_z) - abs(self.imu_data.orientation.z) > config.Z_COMP_DIFF_UB:
                rospy.loginfo("CONTINGENCY -- z component diff between GNSS interpolation and IMU too high")

            if abs(gnss_orientation_z) - abs(self.odom_filtered_data.pose.pose.orientation.z) > config.Z_COMP_DIFF_UB:
                rospy.loginfo("CONTINGENCY -- z component diff between GNSS interpolation and filtered odometry too high")
        rospy.loginfo("################################################################")

    
    def monitor_imu(self):

        if self.imu_data is not None:
            # MONITOR ANGULAR VELOCITY + LINEAR ACCELERATION
            # if the robot is not moving, the corresponding IMU values should be ~0.0
            if self.mbf_status != GoalStatus.ACTIVE:
                # angular velocity in rad/sec
                if abs(self.imu_data.angular_velocity.x) > config.NOT_MOVING_ANG_VELO_UB \
                    or abs(self.imu_data.angular_velocity.y) > config.NOT_MOVING_ANG_VELO_UB \
                    or abs(self.imu_data.angular_velocity.z) > config.NOT_MOVING_ANG_VELO_UB:
                        # TODO: contingency
                        rospy.loginfo("contingency..... IMU angular velo")
                        # rospy.loginfo("CONTINGENCY DETECTED ###################################")
                        # rospy.loginfo("ang velo: %s", self.imu_data.angular_velocity)
                        # rospy.loginfo("##########################################################")

                # linear acceleration in m/s^2 (z-component is g (~9.81))
                if abs(self.imu_data.linear_acceleration.x) > config.NOT_MOVING_LIN_ACC_UB \
                    or abs(self.imu_data.linear_acceleration.y) > config.NOT_MOVING_LIN_ACC_UB:
                        # TODO: contingency
                        rospy.loginfo("contingency..... IMU lin acc")
                        # rospy.loginfo("CONTINGENCY DETECTED ###################################")
                        # rospy.loginfo("lin acc: %s", self.imu_data.linear_acceleration)
                        # rospy.loginfo("##########################################################")
            # robot is moving -> should be visible in IMU
            else:
                pass
                # rospy.loginfo("now there should be high values - we are moving::")
                # rospy.loginfo("ang velo: %s", self.imu_data.angular_velocity)
                # rospy.loginfo("lin acc: %s", self.imu_data.linear_acceleration)
                # however, should it be visible at all times when ACTIVE?

            # MONITOR COVARIANCE -> diagonals contain variances (x, y, z)
            # --> all zeros -> covariance unknown
            # --> 1st element of  matrix -1 -> no estimate for the data elements (e.g. no orientation provided by IMU)
            # covariance known and data provided by IMU -> monitor
            if sum(self.imu_data.orientation_covariance) != 0.0 and self.imu_data.orientation_covariance[0] != -1.0:
                for val in self.imu_data.orientation_covariance:
                    if val > config.IMU_ORIENTATION_COV_UB:
                        rospy.loginfo("contingency -> IMU covariance (ori)")
                        break
                        # contingency
            # covariance known and data provided by IMU -> monitor
            if sum(self.imu_data.angular_velocity_covariance) != 0.0 and self.imu_data.angular_velocity_covariance[0] != -1.0:
                for val in self.imu_data.angular_velocity_covariance:
                    if val > config.IMU_ANGULAR_VELO_COV_UB:
                        rospy.loginfo("contingency -> IMU covariance (angu velo)")
                        break
                        # contingency
            # covariance known and data provided by IMU -> monitor
            if sum(self.imu_data.linear_acceleration_covariance) != 0.0 and self.imu_data.linear_acceleration_covariance[0] != -1.0:
                for val in self.imu_data.linear_acceleration_covariance:
                    if val > config.IMU_LINEAR_ACC_COV_UB:
                        rospy.loginfo("contingency -> IMU covariance (lin acc)")
                        break
                        # contingency


    def monitor_odom(self, filtered):
        name = "odometry" if not filtered else "filtered odometry"
        rospy.loginfo("monitor %s", name)
        odom_data = self.odom_data if not filtered else self.odom_filtered_data
        
        if self.odom_data is not None:
            # MONITOR LINEAR + ANGULAR TWIST
            # if the robot is not moving, the corresponding IMU values should be ~0.0
            if self.mbf_status != GoalStatus.ACTIVE:
                if abs(odom_data.twist.twist.linear.x) > config.NOT_MOVING_LINEAR_TWIST_UB \
                    or abs(odom_data.twist.twist.linear.y) > config.NOT_MOVING_LINEAR_TWIST_UB \
                    or abs(odom_data.twist.twist.linear.z) > config.NOT_MOVING_LINEAR_TWIST_UB:
                        # TODO: contingency
                        rospy.loginfo("%s CONTINGENCY DETECTED ###################################", name)
                        rospy.loginfo("not moving, but linear twist: %s", odom_data.twist.twist.linear)
                        rospy.loginfo("##########################################################")
                if abs(odom_data.twist.twist.angular.x) > config.NOT_MOVING_ANGULAR_TWIST_UB \
                    or abs(odom_data.twist.twist.angular.y) > config.NOT_MOVING_ANGULAR_TWIST_UB \
                    or abs(odom_data.twist.twist.angular.z) > config.NOT_MOVING_ANGULAR_TWIST_UB:
                        # TODO: contingency
                        rospy.loginfo("%s CONTINGENCY DETECTED ###################################", name)
                        rospy.loginfo("not moving, but angular twist: %s", odom_data.twist.twist.angular)
                        rospy.loginfo("##########################################################")
            # moving
            else:
                # moving is hard to monitor -> there can be pauses with (0, 0, 0)
                pass
                # # at least one linear component should be significantly higher than 0
                # if abs(self.odom_data.twist.twist.linear.x) < config.MOVING_LINEAR_TWIST_LB \
                #     and abs(self.odom_data.twist.twist.linear.y) < config.MOVING_LINEAR_TWIST_LB \
                #     and abs(self.odom_data.twist.twist.linear.z) < config.MOVING_LINEAR_TWIST_LB:
                #         # TODO: contingency
                #         pass
                #         rospy.loginfo("CONTINGENCY DETECTED ###################################")
                #         rospy.loginfo("moving - linear twist: %s", self.odom_data.twist.twist.linear)
                #         rospy.loginfo("##########################################################")

            # POSE + TWIST COVARIANCE (6x6 matrices (x, y, z, rot_x, rot_y, rot_z))
            # TODO: only due to the very high covariance of the filtered version -> to be checked later
            if not filtered:
                for val in odom_data.pose.covariance:
                        if val > config.ODOM_POSE_COV_UP:
                            # TODO: contingency
                            rospy.loginfo("contingency: %s -> pose cov", name)
                            break
                for val in odom_data.twist.covariance:
                    if val > config.ODOM_TWIST_COV_UP:
                        # TODO: contingency
                        rospy.loginfo("contingency: %s -> twist cov", name)
                        break

    def gps_as_odom_callback(self, gps_as_odom):
        self.gps_as_odom_data_second_latest = self.gps_as_odom_data_latest
        self.gps_as_odom_data_latest = gps_as_odom

    def filtered_odom_callback(self, filtered_odom):
        self.odom_filtered_data = filtered_odom

    def odom_callback(self, odom):
        self.odom_data = odom

    def imu_callback(self, imu):
        self.imu_data = imu

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
