#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class RepublishVelodyne:
    """
    Minimalistic node that republishes a Velodyne scan under the RIEGL topic after a scan action has been initiated.
    Since the Velodyne LiDAR sensor is already part of the simulation and the type of incoming data is essentially the same, a scan action just leads to one 
    repuplished Velodyne scan under the /RIEGL topic. Thus, the idea is that the Velodyne sensor briefly pretends to be a RIEGL sensor. This approach has the useful
    side effect of making it relatively easy to simulate sensor failures by simply stopping to republish the Velodyne after a scan action.
    """

    def __init__(self):
        self.simulate_sensor_failure = False
        self.sensor_failure_sub = rospy.Subscriber("/toggle_simulated_sensor_failure", String, self.toggle_sensor_failure_callback, queue_size=1)
        self.scan_action_sub = rospy.Subscriber('/scan_action', String, self.action_callback, queue_size=1)
        self.scan_pub = rospy.Publisher("/RIEGL", LaserScan, queue_size=1)

    def toggle_sensor_failure_callback(self, msg):
        self.simulate_sensor_failure = not self.simulate_sensor_failure

    def action_callback(self, msg):
        # create a new subscription to the topic, receive one message, then unsubscribe
        scan = rospy.wait_for_message("/scanVelodyne", LaserScan, timeout=60)
        if not self.simulate_sensor_failure:
            self.scan_pub.publish(scan)


def node():
    rospy.init_node('republish_velodyne')
    rospy.wait_for_message('SMACH_runnning', String)
    server = RepublishVelodyne()
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass