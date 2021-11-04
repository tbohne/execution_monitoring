#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class SensorFailureResolver:

    def __init__(self):
        self.resolve_sub = rospy.Subscriber('/resolve_sensor_failure', String, self.resolve_callback, queue_size=1)
        self.toggle_sim_sensor_failure_pub = rospy.Publisher("/toggle_simulated_sensor_failure", String, queue_size=1)

    def resolve_callback(self, msg):
        rospy.loginfo("launch sensor failure resolver..")
        self.toggle_sim_sensor_failure_pub.publish("")
        rospy.loginfo("sensor failure resolved..")

def node():
    rospy.init_node('sensor_failure_resolver')
    rospy.wait_for_message('SMACH_runnning', String)
    SensorFailureResolver()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
