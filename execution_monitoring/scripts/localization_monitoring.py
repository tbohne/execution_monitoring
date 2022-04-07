#!/usr/bin/env python
import rospy
import math
from execution_monitoring import config
from datetime import datetime
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
import collections
import numpy as np
import copy

class LocalizationMonitoring:
    """
    Monitoring for localization failures -> measure quality of localization.
    -> estimate plausibility of localization based on sensor data
    -> our localization: IMU + odometry + GNSS -> Kalman filter to bring these information together (robot_localization pkg)
    """

    def __init__(self):
        self.init_vars()
        self.contingency_pub = rospy.Publisher('/contingency_preemption', String, queue_size=1)
        self.aggravate_pub = rospy.Publisher('/aggravate', String, queue_size=1)
        self.robot_info_pub = rospy.Publisher('/robot_info', String, queue_size=1)
        self.interrupt_reason_pub = rospy.Publisher('/interrupt_reason', String, queue_size=1)

        rospy.Subscriber('/imu_data', Imu, self.imu_callback, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber('/odometry/filtered_odom', Odometry, self.filtered_odom_callback, queue_size=1)
        rospy.Subscriber('/odometry/gps', Odometry, self.gps_as_odom_callback, queue_size=1)
        rospy.Subscriber('/move_base_flex/exe_path/status', GoalStatusArray, self.mbf_status_callback, queue_size=1)
        rospy.Subscriber('/resolve_localization_failure_success', Bool, self.re_init, queue_size=1)
        rospy.Subscriber('/deactivate_localization_monitoring', String, self.deactivate, queue_size=1)
        rospy.Subscriber('/activate_localization_monitoring', String, self.activate, queue_size=1)
        self.init()

    def init_vars(self):
        self.active_monitoring = True
        self.status_before = None
        self.status_switch_time = None
        self.active_imu_data = collections.deque([], config.IMU_ENTRIES)
        self.passive_imu_data = collections.deque([], config.IMU_ENTRIES)
        self.imu_latest = None
        self.odom_data = None
        self.odom_filtered_data = None
        self.gps_as_odom_data_latest = None
        self.gps_as_odom_data_second_latest = None
        self.initial_GPS = None
        self.initial_odom = None
        self.mbf_status = None
        self.lin_acc_active_history = collections.deque([], config.LIN_ACC_HISTORY_LEN)
        self.lin_acc_passive_history = collections.deque([], config.LIN_ACC_HISTORY_LEN)

    def init(self):
        self.init_vars()
        self.localization_monitoring()

    def deactivate(self, msg):
        rospy.loginfo("DEACTIVATING LOCALIZATION MONITORING..")
        self.robot_info_pub.publish("deactivating localization monitoring")
        self.active_monitoring = False

    def activate(self, msg):
        rospy.loginfo("ACTIVATING LOCALIZATION MONITORING..")
        self.robot_info_pub.publish("activating localization monitoring")
        self.active_monitoring = True
        self.localization_monitoring()

    def re_init(self, msg):
        if msg.data:
            rospy.loginfo("re-initialiizing the localiization monitoring..")
            self.robot_info_pub.publish("re-initialiizing localiization monitoring")
            rospy.sleep(10)
            self.init()
        else:
            self.interrupt_reason_pub.publish(config.LOCALIZATION_CATA)
            self.aggravate_pub.publish(config.LOCALIZATION_CATA)

    def mbf_status_callback(self, mbf_status):
        if self.active_monitoring and len(mbf_status.status_list) > 0:
            # the last element in the list is the latest (most recent)
            self.mbf_status = mbf_status.status_list[-1].status
            if self.mbf_status != self.status_before:
                self.status_switch_time = datetime.now()
                self.status_before = self.mbf_status

    def localization_monitoring(self):
        """
        IMU      -> orientation (relative to a global reference frame)
        Odometry -> position + orientation (relative to initial pose)
        GNSS     -> position + interpolated orientation (absolute)
        """
        while self.active_monitoring:

            if self.status_switch_time is not None and (datetime.now() - self.status_switch_time).total_seconds() > config.STATUS_SWITCH_DELAY:
                self.monitor_imu()
                self.monitor_odom(True)
                self.monitor_odom(False)

                if self.odom_data is not None and self.odom_filtered_data is not None and self.gps_as_odom_data_latest is not None:
                    self.yaw_monitoring()
                    self.odom_gnss_dist_divergence_monitoring()

            rospy.sleep(config.LOCALIZATION_MON_FREQ)

    def odom_gnss_dist_divergence_monitoring(self):
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

            if abs(odom_dist - gps_dist) > 2.0:
                rospy.loginfo("CONTINGENCY: GNSS (initial-current) and odometry (initial-current) distances are diverging quite heavily -> indicator for localization issue")
                self.contingency_pub.publish(config.LOCALIZATION_FAILURE_ONE)
                self.active_monitoring = False
            elif abs(odom_dist - gps_dist) > 1.5:
                rospy.loginfo("CONTINGENCY: GNSS (initial-current) and odometry (initial-current) distances are diverging quite a bit -> indicator for localization issue")
                self.contingency_pub.publish(config.LOCALIZATION_FAILURE_TWO)
                self.active_monitoring = False
            elif abs(odom_dist - gps_dist) > 1:
                rospy.loginfo("INFO: GNSS (initial-current) and odometry (initial-current) distances are slightly diverging -> indicator for minor localization issue")
                self.robot_info_pub.publish(config.LOCALIZATION_FAILURE_THREE)
            if abs(odom_dist - gps_dist) > 1:
                rospy.loginfo("2D dist between initial filtered odom and current: %s", odom_dist)
                rospy.loginfo("2D dist between initial GPS and current: %s", gps_dist)

    def yaw_monitoring(self):
        """
        Arguably the most important component of the orientation.
        GNSS interpolation of orientation (z (yaw) component).
        Just comparing with IMU and filtered odometry. Odometry itself is always too bad.
        """

        # interpolate orientation based on two GPS points
        x1 = self.gps_as_odom_data_latest.pose.pose.position.x
        x2 = self.gps_as_odom_data_second_latest.pose.pose.position.x
        y1 = self.gps_as_odom_data_latest.pose.pose.position.y
        y2 = self.gps_as_odom_data_second_latest.pose.pose.position.y

        vector_x = x1 - x2
        vector_y = y1 - y2
        
        dist = math.sqrt((vector_x) ** 2 + (vector_y) ** 2)

        if (self.mbf_status == GoalStatus.ACTIVE and dist > config.DIST_THRESH_FOR_INTERPOLATION_BETWEEN_GNSS_POS):

            #rospy.loginfo("yaw mon dist: %s", dist)
            angle = math.atan2(vector_y, vector_x)
            quaternion = tf.transformations.quaternion_from_euler(0, 0, angle)
            #rospy.loginfo("diff: %s", abs(abs(quaternion[2]) - abs(self.imu_latest.orientation.z)))
            gnss_orientation_z = quaternion[2]
            contingency = False

            if abs(abs(gnss_orientation_z) - abs(self.imu_latest.orientation.z)) > config.Z_COMP_DIFF_UB:
                rospy.loginfo("CONTINGENCY -- yaw diff between GNSS interpolation and IMU too high")
                self.contingency_pub.publish(config.LOCALIZATION_FAILURE_FOUR)
                self.active_monitoring = False
                contingency = True
            if abs(abs(gnss_orientation_z) - abs(self.odom_filtered_data.pose.pose.orientation.z)) > config.Z_COMP_DIFF_UB:
                rospy.loginfo("CONTINGENCY -- yaw diff between GNSS interpolation and filtered odometry too high")
                self.contingency_pub.publish(config.LOCALIZATION_FAILURE_FIVE)
                self.active_monitoring = False
                contingency = True
            if contingency:
                rospy.loginfo("imu orientation z: %s", self.imu_latest.orientation.z)
                rospy.loginfo("odom orientation z: %s", self.odom_data.pose.pose.orientation.z)
                rospy.loginfo("odom filtered orientation z: %s", self.odom_filtered_data.pose.pose.orientation.z)
                rospy.loginfo("gnss orientation interpolation z component: %s", gnss_orientation_z)

    def imu_std_dev_monitoring(self, cov_matrix, std_dev_UB):
        # MONITOR COVARIANCE -> diagonals contain variances (x, y, z)
        # --> all zeros -> covariance unknown
        # --> 1st element of  matrix -1 -> no estimate for the data elements (e.g. no orientation provided by IMU)

        # covariance known and data provided by IMU -> monitor diagonal (variances or standard deviations)
        if sum(cov_matrix) != 0.0 and cov_matrix[0] != -1.0:
            for i in range(0, len(cov_matrix), 4):
                if math.sqrt(cov_matrix[i]) > std_dev_UB:
                    rospy.loginfo("CONTINGENCY -> IMU standard deviations too high")
                    self.contingency_pub.publish(config.LOCALIZATION_FAILURE_SIX)
                    self.active_monitoring = False
                    return

    def monitor_imu(self):

        if len(self.active_imu_data) > 0 and len(self.passive_imu_data) > 0:

            passive_imu_copy = copy.deepcopy(self.passive_imu_data)
            active_imu_copy = copy.deepcopy(self.active_imu_data)

            # MONITOR ANGULAR VELOCITY + LINEAR ACCELERATION
            # if the robot is not moving, the corresponding IMU values should be ~0.0
            # angular velocity in rad/sec
            x_avg = np.average([abs(data.angular_velocity.x) for data in passive_imu_copy])
            y_avg = np.average([abs(data.angular_velocity.y) for data in passive_imu_copy])
            z_avg = np.average([abs(data.angular_velocity.z) for data in passive_imu_copy])

            if x_avg > config.NOT_MOVING_ANG_VELO_UB or y_avg > config.NOT_MOVING_ANG_VELO_UB or z_avg > config.NOT_MOVING_ANG_VELO_UB:
                rospy.loginfo("CONTINGENCY..... IMU angular velocity too high for passive state")
                rospy.loginfo("ang velo: %s, %s, %s", x_avg, y_avg, z_avg)
                self.contingency_pub.publish(config.LOCALIZATION_FAILURE_SEVEN)
                self.active_monitoring = False

            #rospy.loginfo("passive, active, total: %s, %s, %s", len(passive_imu_copy), len(active_imu_copy), config.IMU_ENTRIES)

            if len(passive_imu_copy) == config.IMU_ENTRIES and len(active_imu_copy) == config.IMU_ENTRIES:
                entries = int(config.IMU_PERCENTAGE * config.IMU_ENTRIES)
                self.lin_acc_passive_history.appendleft(np.median(sorted([max(abs(data.linear_acceleration.x), abs(data.linear_acceleration.y)) for data in passive_imu_copy])[::-1][:entries]))
                self.lin_acc_active_history.appendleft(np.median(sorted([max(abs(data.linear_acceleration.x), abs(data.linear_acceleration.y)) for data in active_imu_copy])[::-1][:entries]))

                # linear acceleration in m/s^2 (z-component is g (~9.81))
                avg_ratio = 0.0
                for i in range(config.COVARIANCE_HISTORY_LENGTH):
                    if i < len(self.lin_acc_active_history) and i < len(self.lin_acc_passive_history):
                        avg_ratio += self.lin_acc_active_history[i] / self.lin_acc_passive_history[i]
                        if self.lin_acc_passive_history[i] > config.NOT_MOVING_LIN_ACC_UB:
                            rospy.loginfo("mbf status: %s", self.mbf_status)
                            rospy.loginfo("CONTINGENCY..... linear acceleration too high for passive state: %s",self.lin_acc_passive_history[i])
                            self.contingency_pub.publish(config.LOCALIZATION_FAILURE_EIGHT)
                            self.active_monitoring = False
                avg_ratio /= config.COVARIANCE_HISTORY_LENGTH

                if len(self.lin_acc_active_history) == config.COVARIANCE_HISTORY_LENGTH \
                    and len(self.lin_acc_passive_history) == config.COVARIANCE_HISTORY_LENGTH \
                    and avg_ratio < config.ACTIVE_PASSIVE_FACTOR_LB:

                    rospy.loginfo("CONTINGENCY: linear acceleration during movement not considerably higher compared to standing still")
                    self.contingency_pub.publish(config.LOCALIZATION_FAILURE_NINE)
                    self.active_monitoring = False
                    rospy.loginfo("active values: %s", self.lin_acc_active_history)
                    rospy.loginfo("passive values: %s", self.lin_acc_passive_history)
                    
        # MONITOR COVARIANCE
        if self.imu_latest is not None:
            self.imu_std_dev_monitoring(self.imu_latest.orientation_covariance, config.IMU_ORIENTATION_STD_DEV_UB)
            self.imu_std_dev_monitoring(self.imu_latest.angular_velocity_covariance, config.IMU_ANGULAR_VELO_STD_DEV_UB)
            self.imu_std_dev_monitoring(self.imu_latest.linear_acceleration_covariance, config.IMU_LINEAR_ACC_STD_DEV_UB)

    def monitor_odom(self, filtered):
        name = "odometry" if not filtered else "filtered odometry"
        odom_data = self.odom_data if not filtered else self.odom_filtered_data

        if self.status_switch_time is None or (datetime.now() - self.status_switch_time).total_seconds() < config.STATUS_SWITCH_DELAY:
            return
        
        if self.odom_data is not None:
            # MONITOR LINEAR + ANGULAR TWIST
            # if the robot is not moving, the corresponding odom values should be ~0.0
            if self.mbf_status != GoalStatus.ACTIVE and self.mbf_status != GoalStatus.ABORTED and self.mbf_status != GoalStatus.PREEMPTED:
                if abs(odom_data.twist.twist.linear.x) > config.NOT_MOVING_LINEAR_TWIST_UB \
                    or abs(odom_data.twist.twist.linear.y) > config.NOT_MOVING_LINEAR_TWIST_UB \
                    or abs(odom_data.twist.twist.linear.z) > config.NOT_MOVING_LINEAR_TWIST_UB:
                        rospy.loginfo("%s CONTINGENCY DETECTED ###################################", name)
                        rospy.loginfo("not moving, but linear twist: %s", odom_data.twist.twist.linear)
                        self.contingency_pub.publish(config.LOCALIZATION_FAILURE_TEN)
                        self.active_monitoring = False
                if abs(odom_data.twist.twist.angular.x) > config.NOT_MOVING_ANGULAR_TWIST_UB \
                    or abs(odom_data.twist.twist.angular.y) > config.NOT_MOVING_ANGULAR_TWIST_UB \
                    or abs(odom_data.twist.twist.angular.z) > config.NOT_MOVING_ANGULAR_TWIST_UB:
                        rospy.loginfo("%s CONTINGENCY DETECTED ###################################", name)
                        rospy.loginfo("not moving, but angular twist: %s", odom_data.twist.twist.angular)
                        self.contingency_pub.publish(config.LOCALIZATION_FAILURE_ELEVEN)
                        self.active_monitoring = False

            # POSE + TWIST COVARIANCE (6x6 matrices (x, y, z, rot_x, rot_y, rot_z))
            # TODO: only due to the very high covariance of the filtered version -> to be checked later
            if not filtered:
                # uncertainty in pose
                for i in range(0, len(odom_data.pose.covariance), 7):
                    if math.sqrt(odom_data.pose.covariance[i]) > config.ODOM_POSE_STD_DEV_UB:
                        rospy.loginfo("CONTINGENCY: %s -> pose standard deviation", name)
                        self.contingency_pub.publish(config.LOCALIZATION_FAILURE_TWELVE)
                        self.active_monitoring = False
                        break
                # velocity in free space with uncertainty
                for i in range(0, len(odom_data.twist.covariance), 7):
                    if math.sqrt(odom_data.twist.covariance[i]) > config.ODOM_TWIST_STD_DEV_UB:
                        rospy.loginfo("CONTINGENCY: %s -> twist standard deviation", name)
                        self.contingency_pub.publish(config.LOCALIZATION_FAILURE_THIRTEEN)
                        self.active_monitoring = False
                        break

    def gps_as_odom_callback(self, gps_as_odom):
        if self.active_monitoring:
            self.gps_as_odom_data_second_latest = self.gps_as_odom_data_latest
            self.gps_as_odom_data_latest = gps_as_odom

    def filtered_odom_callback(self, filtered_odom):
        if self.active_monitoring:
            self.odom_filtered_data = filtered_odom

    def odom_callback(self, odom):
        if self.active_monitoring:
            self.odom_data = odom

    def imu_callback(self, imu):
        if self.active_monitoring:
            self.imu_latest = imu
            # after status switch - block 10s # TODO: does it have to be that long?
            if self.status_switch_time is not None and (datetime.now() - self.status_switch_time).total_seconds() > config.STATUS_SWITCH_DELAY:
                # aborted state should not be collected at all
                if self.mbf_status == GoalStatus.SUCCEEDED: #self.mbf_status != GoalStatus.ACTIVE and self.mbf_status != GoalStatus.ABORTED:
                    self.passive_imu_data.appendleft(imu)
                elif self.mbf_status == GoalStatus.ACTIVE: #self.mbf_status != GoalStatus.ABORTED:
                    self.active_imu_data.appendleft(imu)

def node():
    rospy.init_node('localization_monitoring')
    rospy.wait_for_message('SMACH_running', String)
    rospy.loginfo("launch localization monitoring..")
    LocalizationMonitoring()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
