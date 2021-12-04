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
        self.sim_timeout = False

        self.gps_publisher = rospy.Publisher('/fix', NavSatFix, queue_size=1)

    def toggle_timeout_callback(self, msg):
        self.sim_timeout = not self.sim_timeout

    def sim_gps_callback(self, nav_sat_fix):
        #rospy.loginfo("receiving GNSS data: %s", nav_sat_fix)

        if self.sim_timeout:
            self.gnss_sub.unregister()
            rospy.sleep(config.GPS_TIMEOUT)
            self.sim_timeout = False
            self.gnss_sub = rospy.Subscriber('/fix_plugin', NavSatFix, self.sim_gps_callback, queue_size=1)

        nav_sat_fix.status.status = config.GNSS_STATUS
        nav_sat_fix.status.service = config.GNSS_SERVICE
        nav_sat_fix.position_covariance = config.GNSS_COVARIANCES
        nav_sat_fix.position_covariance_type = config.GNSS_COVARIANCE_TYPE
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
