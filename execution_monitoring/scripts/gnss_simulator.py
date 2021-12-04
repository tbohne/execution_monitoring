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
        self.sim_timeout = False
        self.sim_good_quality = True
        self.sim_med_quality = False
        self.sim_low_quality = False
        self.sim_unknown_status = False
        self.sim_no_fix = False
        self.sim_no_rtk = False
        self.sim_unknown_service = False
        self.sim_infeasible_lat_lng = False

        self.gps_publisher = rospy.Publisher('/fix', NavSatFix, queue_size=1)

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
            nav_sat_fix.position_covariance = [3.5, 0.0, 0.0, 0.0, 5.7, 0.0, 0.9, 0.9, 9.9]
            nav_sat_fix.status.service = config.GNSS_SERVICE_GPS
        elif self.sim_med_quality:
            nav_sat_fix.status.status = config.GNSS_STATUS_FIX
            nav_sat_fix.position_covariance_type = config.GNSS_COVARIANCE_TYPE_APPROXIMATED
            nav_sat_fix.position_covariance = [3.5, 0.0, 0.0, 0.0, 5.7, 0.0, 0.9, 0.9, 9.9]
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
