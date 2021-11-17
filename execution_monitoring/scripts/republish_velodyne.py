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
        self.simulate_empty_ranges = False
        self.simulate_look_to_sky = False
        self.simulate_scan_repetition = False

        self.previous_scan = None

        rospy.Subscriber("/toggle_simulated_sensor_failure", String, self.toggle_sensor_failure_callback, queue_size=1)
        rospy.Subscriber("/toggle_simulated_empty_ranges", String, self.toggle_empty_ranges_callback, queue_size=1)
        rospy.Subscriber("/toggle_simulated_look_to_sky", String, self.toggle_look_to_sky_callback, queue_size=1)
        rospy.Subscriber("/toggle_simulated_scan_repetition", String, self.toggle_scan_repetition_callback, queue_size=1)

        self.scan_action_sub = rospy.Subscriber('/scan_action', String, self.action_callback, queue_size=1)
        self.scan_pub = rospy.Publisher("/RIEGL", LaserScan, queue_size=1)

    def toggle_sensor_failure_callback(self, msg):
        self.simulate_sensor_failure = not self.simulate_sensor_failure

    def toggle_empty_ranges_callback(self, msg):
        self.simulate_empty_ranges = not self.simulate_empty_ranges

    def toggle_look_to_sky_callback(self, msg):
        self.simulate_look_to_sky = not self.simulate_look_to_sky

    def toggle_scan_repetition_callback(self, msg):
        self.simulate_scan_repetition = not self.simulate_scan_repetition

    def action_callback(self, msg):
        # create a new subscription to the topic, receive one message, then unsubscribe
        scan = rospy.wait_for_message("/scanVelodyne", LaserScan, timeout=60)

        if not self.simulate_sensor_failure:
            # no total sensor failure
            if self.simulate_empty_ranges:
                rospy.loginfo("SIMULATING EMPTY RANGES FAILURE")
                scan.ranges = []

            if self.simulate_look_to_sky:
                rospy.loginfo("SIMULATING LOOK TO SKY FAILURE")
                fake_ranges = [float('inf') for _ in range(len(scan.ranges))]
                scan.ranges = fake_ranges

            if self.simulate_scan_repetition:
                if self.previous_scan:
                    rospy.loginfo("SIMULATING SCAN REPETITION FAILURE")
                    scan = self.previous_scan
                else:
                    rospy.loginfo("cannot simulate scan repitition failure - there is no previous scan - wait until there is one..")
            
            rospy.loginfo("publishing scan with: %s", scan.header)
            self.scan_pub.publish(scan)
        else:
            rospy.loginfo("SIMULATING TOTAL SENSOR FAILURE..")

        self.previous_scan = scan


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
