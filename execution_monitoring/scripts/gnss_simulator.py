#!/usr/bin/env python
import rospy
from execution_monitoring import config
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

class GNSSSimulator:
    """
    Simple node that receives the simulated GPS data from libhector_gazebo_ros_gps and acts as a "man-in-the-middle".
    Enriches the data published by libhector_gazebo_ros_gps with further information.
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

        # subscribe to topic of quadrotor_gps_sim (libhector_gazebo_ros_gps.so)
        #    -> gazebo plugin that simulates GPS data
        self.gnss_sub = rospy.Subscriber('/fix_plugin', NavSatFix, self.sim_gps_callback, queue_size=1)

        rospy.Subscriber("/toggle_simulated_timeout_failure", String, self.toggle_timeout_callback, queue_size=1)
        rospy.Subscriber("/set_simulated_good_quality", String, self.set_good_qualitiy_callback, queue_size=1)
        rospy.Subscriber("/set_simulated_med_quality", String, self.set_med_qualitiy_callback, queue_size=1)
        rospy.Subscriber("/set_simulated_low_quality", String, self.set_low_qualitiy_callback, queue_size=1)
        rospy.Subscriber("/set_simulated_unknown_status", String, self.set_unknown_status_callback, queue_size=1)
        rospy.Subscriber("/set_simulated_no_fix", String, self.set_no_fix_callback, queue_size=1)
        rospy.Subscriber("/set_simulated_no_rtk", String, self.set_no_rtk_callback, queue_size=1)
        rospy.Subscriber("/toggle_simulated_unknown_service", String, self.toggle_unknown_service_callback, queue_size=1)
        rospy.Subscriber("/toggle_simulated_infeasible_lat_lng", String, self.toggle_infeasible_lat_lng_callback, queue_size=1)
        rospy.Subscriber("/toggle_simulated_variance_history_failure", String, self.toggle_var_history_failure_callback, queue_size=1)
        rospy.Subscriber("/toggle_simulated_high_deviation", String, self.toggle_high_dev_callback, queue_size=1)
        rospy.Subscriber("/toggle_simulated_teleport", String, self.toggle_sim_teleport_callback, queue_size=1)

        self.gps_publisher = rospy.Publisher('/fix', NavSatFix, queue_size=1)

    def toggle_sim_teleport_callback(self, msg):
        self.sim_teleport = not self.sim_teleport

    def toggle_high_dev_callback(self, msg):
        self.sim_high_dev = not self.sim_high_dev

    def toggle_var_history_failure_callback(self, msg):
        self.sim_variance_history_failure = not self.sim_variance_history_failure

    def toggle_infeasible_lat_lng_callback(self, msg):
        self.sim_infeasible_lat_lng = not self.sim_infeasible_lat_lng

    def toggle_unknown_service_callback(self, msg):
        self.sim_unknown_service = not self.sim_unknown_service

    def set_unknown_status_callback(self, msg):
        self.sim_no_fix = self.sim_no_rtk = False
        self.sim_unknown_status = True

    def set_no_fix_callback(self, msg):
        self.sim_unknown_status = self.sim_no_rtk = False
        self.sim_no_fix = True

    def set_no_rtk_callback(self, msg):
        self.sim_unknown_status = self.sim_no_fix = False
        self.sim_no_rtk = True

    def toggle_timeout_callback(self, msg):
        self.sim_timeout = not self.sim_timeout

    def set_good_qualitiy_callback(self, msg):
        self.sim_med_quality = self.sim_low_quality = False
        self.sim_good_quality = True

    def set_med_qualitiy_callback(self, msg):
        self.sim_good_quality = self.sim_low_quality = False
        self.sim_med_quality = True

    def set_low_qualitiy_callback(self, msg):
        self.sim_good_quality = self.sim_med_quality = False
        self.sim_low_quality = True

    def sim_gps_callback(self, nav_sat_fix):
        #rospy.loginfo("receiving GNSS data: %s", nav_sat_fix)

        # timeout sim
        if self.sim_timeout:
            self.gnss_sub.unregister()
            rospy.sleep(config.GPS_TIMEOUT)
            self.sim_timeout = False
            self.gnss_sub = rospy.Subscriber('/fix_plugin', NavSatFix, self.sim_gps_callback, queue_size=1)

        # quality sim
        if self.sim_good_quality:
            nav_sat_fix.status.status = config.GNSS_STATUS_GBAS_FIX
            nav_sat_fix.position_covariance_type = config.GNSS_COVARIANCE_TYPE_DIAGONAL_KNOWN
            nav_sat_fix.position_covariance = [6, 0.0, 0.0, 0.0, 6, 0.0, 0.0, 0.0, 6]
            nav_sat_fix.status.service = config.GNSS_SERVICE_GPS
        elif self.sim_med_quality:
            nav_sat_fix.status.status = config.GNSS_STATUS_FIX
            nav_sat_fix.position_covariance_type = config.GNSS_COVARIANCE_TYPE_APPROXIMATED
            nav_sat_fix.position_covariance = [6, 0.0, 0.0, 0.0, 6, 0.0, 0.0, 0.0, 6]
            nav_sat_fix.status.service = config.GNSS_SERVICE_GALILEO
        elif self.sim_low_quality:
            nav_sat_fix.status.status = config.GNSS_STATUS_FIX
            nav_sat_fix.position_covariance_type = config.GNSS_COVARIANCE_TYPE_UNKNOWN
            nav_sat_fix.status.service = config.GNSS_SERVICE_GALILEO

        # status sim
        if self.sim_unknown_status:
            nav_sat_fix.status.status = 5
            self.sim_unknown_status = False
        elif self.sim_no_fix:
            nav_sat_fix.status.status = config.GNSS_STATUS_NO_FIX
            self.sim_no_fix = False
        elif self.sim_no_rtk:
            nav_sat_fix.status.status = config.GNSS_STATUS_FIX
            self.sim_no_rtk = False

        # service sim
        if self.sim_unknown_service:
            nav_sat_fix.status.service = 3
            self.sim_unknown_service = False

        # lat / lng belief state sim
        if self.sim_infeasible_lat_lng:
            nav_sat_fix.latitude = -120
            nav_sat_fix.longitude = 200
            self.sim_infeasible_lat_lng = False

        # covariance sim
        if self.sim_variance_history_failure:
            nav_sat_fix.position_covariance_type = config.GNSS_COVARIANCE_TYPE_DIAGONAL_KNOWN
            east = north = up = 1.0
            # fill history
            for _ in range(config.COVARIANCE_HISTORY_LENGTH):
                nav_sat_fix.position_covariance = [east, 0.0, 0.0, 0.0, north, 0.0, 0.0, 0.0, up]
                east += 10.0
                self.gps_publisher.publish(nav_sat_fix)
                rospy.sleep(1)
            self.sim_variance_history_failure = False
        if self.sim_high_dev:
            nav_sat_fix.position_covariance_type = config.GNSS_COVARIANCE_TYPE_DIAGONAL_KNOWN
            nav_sat_fix.position_covariance[0] = 101
            self.sim_high_dev = False

        if self.sim_teleport:
            # simulate GNSS jump
            nav_sat_fix.latitude -= 0.0001

        self.gps_publisher.publish(nav_sat_fix)

def node():
    rospy.init_node('gnss_simulator')
    GNSSSimulator()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
