#!/usr/bin/env python
import rospy
from execution_monitoring import config
from sensor_msgs.msg import NavSatFix

class GPSSimulator:
    """
    Simple node that receives the simulated GPS data from libhector_gazebo_ros_gps and acts as a "man-in-the-middle".
    Enriches the data published by libhector_gazebo_ros_gps with further information.
    """

    def __init__(self):
        # subscribe to topic of quadrotor_gps_sim (libhector_gazebo_ros_gps.so)
        #    -> gazebo plugin that simulates GPS data
        rospy.Subscriber('/fix_plugin', NavSatFix, self.sim_gps_callback, queue_size=1)
        self.gps_publisher = rospy.Publisher('/fix', NavSatFix, queue_size=1)

    def sim_gps_callback(self, nav_sat_fix):
        #rospy.loginfo("receiving GNSS data: %s", nav_sat_fix)
        nav_sat_fix.status.status = config.GNSS_STATUS
        nav_sat_fix.status.service = config.GNSS_SERVICE
        nav_sat_fix.position_covariance = config.GNSS_COVARIANCES
        nav_sat_fix.position_covariance_type = config.GNSS_COVARIANCE_TYPE
        self.gps_publisher.publish(nav_sat_fix)

def node():
    rospy.init_node('gps_simulator')
    GPSSimulator()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
