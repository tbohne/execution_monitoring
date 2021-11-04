#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class FakeRiegl:
    """
    Publishes fake laser scans simulating the results of a RIEGL recording.
    Republishes the Velodyne scans under some kind of fake RIEGL topic.
    The idea is to have the control over the publishing in order to be able to simulate sensor failures.
    """

    def __init__(self):
        self.simulate_sensor_failure = False
        self.sensor_failure_sub = rospy.Subscriber("/simulate_sensor_failure", String, self.toggle_sensor_failure_callback, queue_size=1)
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
    rospy.init_node('fake_riegl')
    rospy.wait_for_message('SMACH_runnning', String)
    FakeRiegl()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
