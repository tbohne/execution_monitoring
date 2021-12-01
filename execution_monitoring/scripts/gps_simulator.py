#!/usr/bin/env python
import rospy
import datetime
from execution_monitoring import config
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

class GPSSimulator:

    def __init__(self):
        # subscribe to topic of quadrotor_gps_sim (libhector_gazebo_ros_gps.so)
        #    -> gazebo plugin that simulates GPS data
        rospy.Subscriber('/fix_plugin', NavSatFix, self.sim_gps_callback, queue_size=1)

    def sim_gps_callback(self, msg):
        rospy.loginfo("receiving gps: %s", msg)

def node():
    rospy.init_node('gps_simulator')
    GPSSimulator()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
