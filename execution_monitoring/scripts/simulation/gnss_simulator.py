#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @author Tim Bohne

import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from execution_monitoring import config


class GNSSSimulator:
    """
    Acts as a "man-in-the-middle" to enrich the simulated data coming from GazeboRosGps (`libhector_gazebo_ros_gps`).
    For this purpose, it receives the `NavSatFix` message from the /fix_plugin topic and enriches it with
    user-configurable service, status and covariance information.
    """

    def __init__(self):
        self.sim_timeout = False
        self.sim_good_quality = True
        self.sim_med_quality = False
        self.sim_low_quality = False
        self.sim_unknown_status = False
        self.sim_no_fix = False
        self.sim_no_rtk = False
        self.sim_unknown_service = False
        self.sim_infeasible_lat_lng = False
        self.sim_variance_history_failure = False
        self.sim_high_dev = False
        self.sim_teleport = False
        self.timeout_fail_successful = False

        self.gps_publisher = rospy.Publisher('/fix', NavSatFix, queue_size=1)
        self.sim_info_pub = rospy.Publisher('/sim_info', String, queue_size=1)

        # `libhector_gazebo_ros_gps.so` -> gazebo plugin that simulates GPS data
        self.gnss_sub = rospy.Subscriber('/fix_plugin', NavSatFix, self.sim_gps_callback, queue_size=1)

        rospy.Subscriber('/toggle_simulated_timeout_failure', String, self.toggle_timeout_callback, queue_size=1)
        rospy.Subscriber('/set_simulated_good_quality', String, self.set_good_quality_callback, queue_size=1)
        rospy.Subscriber('/set_simulated_med_quality', String, self.set_med_quality_callback, queue_size=1)
        rospy.Subscriber('/set_simulated_low_quality', String, self.set_low_quality_callback, queue_size=1)
        rospy.Subscriber('/set_simulated_unknown_status', String, self.set_unknown_status_callback, queue_size=1)
        rospy.Subscriber('/set_simulated_no_fix', String, self.set_no_fix_callback, queue_size=1)
        rospy.Subscriber('/set_simulated_no_rtk', String, self.set_no_rtk_callback, queue_size=1)
        rospy.Subscriber('/toggle_simulated_unknown_service', String, self.toggle_unknown_service, queue_size=1)
        rospy.Subscriber('/toggle_simulated_infeasible_lat_lng', String, self.toggle_infeasible_lat_lng, queue_size=1)
        rospy.Subscriber('/toggle_simulated_variance_history_failure', String, self.toggle_var_hist_fail, queue_size=1)
        rospy.Subscriber('/toggle_simulated_high_deviation', String, self.toggle_high_dev_callback, queue_size=1)
        rospy.Subscriber('/toggle_simulated_teleport', String, self.toggle_sim_teleport_callback, queue_size=1)
        rospy.Subscriber('/contingency_preemption', String, self.contingency_callback, queue_size=1)

    def contingency_callback(self, msg):
        """
        Called in contingency situations.
        Checks the result of timeout failure simulations.

        @param msg: reason for contingency
        """
        if msg.data == config.GNSS_FAILURES[0]:
            self.timeout_fail_successful = True

    def toggle_sim_teleport_callback(self, msg):
        """
        (De)activates simulated teleport.

        @param msg: callback message
        """
        self.sim_teleport = not self.sim_teleport

    def toggle_high_dev_callback(self, msg):
        """
        (De)activates simulated high standard deviations.

        @param msg: callback message
        """
        self.sim_high_dev = not self.sim_high_dev

    def toggle_var_hist_fail(self, msg):
        """
        (De)activates simulated variance history failure.

        @param msg: callback message
        """
        self.sim_variance_history_failure = not self.sim_variance_history_failure

    def toggle_infeasible_lat_lng(self, msg):
        """
        (De)activates simulated infeasible lat / lng values.

        @param msg: callback message
        """
        self.sim_infeasible_lat_lng = not self.sim_infeasible_lat_lng

    def toggle_unknown_service(self, msg):
        """
        (De)activates simulated unknown service situations.

        @param msg: callback message
        """
        self.sim_unknown_service = not self.sim_unknown_service

    def set_unknown_status_callback(self, msg):
        """
        Prepares unknown status simulation.

        @param msg: callback message
        """
        self.sim_no_fix = self.sim_no_rtk = False
        self.sim_unknown_status = True

    def set_no_fix_callback(self, msg):
        """
        Prepares no fix simulation (GNSS not able to estimate position).

        @param msg: callback message
        """
        self.sim_unknown_status = self.sim_no_rtk = False
        self.sim_no_fix = True

    def set_no_rtk_callback(self, msg):
        """
        Prepares no RTK simulation.

        @param msg: callback message
        """
        self.sim_unknown_status = self.sim_no_fix = False
        self.sim_no_rtk = True

    def toggle_timeout_callback(self, msg):
        """
        (De)activates GNSS timeout simulation.

        @param msg: callback message
        """
        self.sim_timeout = not self.sim_timeout

    def set_good_quality_callback(self, msg):
        """
        Prepares simulation of good quality GNSS data.

        @param msg: callback message
        """
        self.sim_med_quality = self.sim_low_quality = False
        self.sim_good_quality = True

    def set_med_quality_callback(self, msg):
        """
        Prepares simulation of medium quality GNSS data.

        @param msg: callback message
        """
        self.sim_good_quality = self.sim_low_quality = False
        self.sim_med_quality = True

    def set_low_quality_callback(self, msg):
        """
        Prepares simulation of low quality GNSS data.

        @param msg: callback message
        """
        self.sim_good_quality = self.sim_med_quality = False
        self.sim_low_quality = True

    def timeout_sim(self):
        """
        Simulates GNSS connection timeout.
        """
        self.sim_timeout = False
        self.gnss_sub.unregister()
        self.sim_info_pub.publish("GNSS simulator: sim GNSS timeout")
        while not self.timeout_fail_successful:
            rospy.sleep(config.SHORT_DELAY)
        self.timeout_fail_successful = False
        self.gnss_sub = rospy.Subscriber('/fix_plugin', NavSatFix, self.sim_gps_callback, queue_size=1)

    @staticmethod
    def good_quality_sim(nav_sat_fix):
        """
        Simulating GNSS data of good quality.

        @param nav_sat_fix: GNSS data to be enriched
        """
        # default -- no need to log
        nav_sat_fix.status.status = config.GNSS_STATUS_GBAS_FIX
        nav_sat_fix.position_covariance_type = config.GNSS_COVARIANCE_TYPE_DIAGONAL_KNOWN
        nav_sat_fix.position_covariance = config.GOOD_QUALITY_COVARIANCE_SIM
        nav_sat_fix.status.service = config.GNSS_SERVICE_GPS

    def med_quality_sim(self, nav_sat_fix):
        """
        Simulating GNSS data of medium quality.

        @param nav_sat_fix: GNSS data to be enriched
        """
        self.sim_info_pub.publish("GNSS simulator: sim medium GNSS quality")
        nav_sat_fix.status.status = config.GNSS_STATUS_FIX
        nav_sat_fix.position_covariance_type = config.GNSS_COVARIANCE_TYPE_APPROXIMATED
        nav_sat_fix.position_covariance = config.GOOD_QUALITY_COVARIANCE_SIM
        nav_sat_fix.status.service = config.GNSS_SERVICE_GALILEO

    def low_quality_sim(self, nav_sat_fix):
        """
        Simulating GNSS data of low quality.

        @param nav_sat_fix: GNSS data to be enriched
        """
        self.sim_info_pub.publish("GNSS simulator: sim low GNSS quality")
        nav_sat_fix.status.status = config.GNSS_STATUS_FIX
        nav_sat_fix.position_covariance_type = config.GNSS_COVARIANCE_TYPE_UNKNOWN
        nav_sat_fix.status.service = config.GNSS_SERVICE_GALILEO

    def quality_sim(self, nav_sat_fix):
        """
        Initiates GNSS quality sim.

        @param nav_sat_fix: GNSS data to be enriched
        """
        if self.sim_good_quality:
            self.good_quality_sim(nav_sat_fix)
        elif self.sim_med_quality:
            self.med_quality_sim(nav_sat_fix)
        elif self.sim_low_quality:
            self.low_quality_sim(nav_sat_fix)

    def meta_info_sim(self, nav_sat_fix):
        """
        Simulates several GNSS meta information.

        @param nav_sat_fix: GNSS data to be enriched
        """
        if self.sim_unknown_status:
            self.sim_info_pub.publish("GNSS simulator: sim unknown GNSS status")
            nav_sat_fix.status.status = config.UNKNOWN_STATUS_SIM
            self.sim_unknown_status = False
        elif self.sim_no_fix:
            self.sim_info_pub.publish("GNSS simulator: sim no fix")
            nav_sat_fix.status.status = config.GNSS_STATUS_NO_FIX
            self.sim_no_fix = False
        elif self.sim_no_rtk:
            self.sim_info_pub.publish("GNSS simulator: sim no RTK")
            nav_sat_fix.status.status = config.GNSS_STATUS_FIX
            self.sim_no_rtk = False
        elif self.sim_unknown_service:
            self.sim_unknown_service = False
            self.sim_info_pub.publish("GNSS simulator: sim unknown service")
            nav_sat_fix.status.service = config.UNKNOWN_SERVICE_SIM

    def infeasible_lat_lng_sim(self, nav_sat_fix):
        """
        Simulates infeasible lat / lng values.

        @param nav_sat_fix: GNSS data to be enriched
        """
        self.sim_infeasible_lat_lng = False
        self.sim_info_pub.publish("GNSS simulator: sim infeasible lat / lng")
        nav_sat_fix.latitude = config.INFEASIBLE_LAT_SIM
        nav_sat_fix.longitude = config.INFEASIBLE_LNG_SIM

    def variance_history_fail_simulation(self, nav_sat_fix):
        """
        Simulates variance history failure - increasing (co)variances over time.

        @param nav_sat_fix: GNSS data to be enriched
        """
        self.sim_variance_history_failure = False
        self.sim_info_pub.publish("GNSS simulator: sim covariance failure")
        nav_sat_fix.position_covariance_type = config.GNSS_COVARIANCE_TYPE_DIAGONAL_KNOWN
        east = north = up = config.VAR_HISTORY_SIM_START_VAL
        # fill history
        for _ in range(config.COVARIANCE_HISTORY_LENGTH):
            nav_sat_fix.position_covariance = [east, 0.0, 0.0, 0.0, north, 0.0, 0.0, 0.0, up]
            east += config.VAR_HISTORY_SIM_INC_VAL
            self.gps_publisher.publish(nav_sat_fix)
            rospy.sleep(1)

    def high_standard_deviation_fail_simulation(self, nav_sat_fix):
        """
        Simulates high standard deviation.

        @param nav_sat_fix: GNSS data to be enriched
        """
        self.sim_high_dev = False
        self.sim_info_pub.publish("GNSS simulator: sim high standard deviation")
        nav_sat_fix.position_covariance_type = config.GNSS_COVARIANCE_TYPE_DIAGONAL_KNOWN
        nav_sat_fix.position_covariance[0] = config.HIGH_STANDARD_DEV_VAL

    def gnss_jump_simulation(self, nav_sat_fix):
        """
        Simulates GNSS jump (teleport).

        @param nav_sat_fix: GNSS data to be enriched
        """
        self.sim_info_pub.publish("GNSS simulator: sim GNSS jump")
        nav_sat_fix.latitude -= config.GNSS_JUMP_VAL

    def sim_gps_callback(self, nav_sat_fix):
        """
        Enriches simulated GNSS input data / simulates failure situations.

        @param nav_sat_fix: GNSS input data (coming from GazeboRosGps) to be enriched
        """
        if self.sim_timeout:
            self.timeout_sim()
        self.quality_sim(nav_sat_fix)
        self.meta_info_sim(nav_sat_fix)
        if self.sim_infeasible_lat_lng:
            self.infeasible_lat_lng_sim(nav_sat_fix)
        if self.sim_variance_history_failure:
            self.variance_history_fail_simulation(nav_sat_fix)
        if self.sim_high_dev:
            self.high_standard_deviation_fail_simulation(nav_sat_fix)
        if self.sim_teleport:
            self.gnss_jump_simulation(nav_sat_fix)
        self.gps_publisher.publish(nav_sat_fix)


def node():
    """
    GNSS (failure) simulation node.
    """
    rospy.init_node('gnss_simulator')
    GNSSSimulator()
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
