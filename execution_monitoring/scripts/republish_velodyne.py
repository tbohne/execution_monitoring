#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from execution_monitoring import config

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
        rospy.Subscriber("/toggle_simulated_total_sensor_failure", String, self.toggle_sensor_failure_callback, queue_size=1)
        rospy.Subscriber("/toggle_simulated_empty_ranges", String, self.toggle_empty_ranges_callback, queue_size=1)
        rospy.Subscriber("/toggle_simulated_impermissible_ranges", String, self.toggle_look_to_sky_callback, queue_size=1)
        rospy.Subscriber("/toggle_simulated_scan_repetition", String, self.toggle_scan_repetition_callback, queue_size=1)
        rospy.Subscriber('/scan_action', String, self.action_callback, queue_size=1)
        self.scan_pub = rospy.Publisher("/RIEGL", LaserScan, queue_size=1)
        self.sim_info_pub = rospy.Publisher('/sim_info', String, queue_size=1)

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
        scan = rospy.wait_for_message("/scanVelodyne", LaserScan, timeout=config.SCAN_TIME_LIMIT)

        if not self.simulate_sensor_failure:
            # no total sensor failure
            if self.simulate_empty_ranges:
                rospy.loginfo("SIMULATING EMPTY RANGES FAILURE")
                self.sim_info_pub.publish("republish velodyne: sim empty ranges fail")
                scan.ranges = []
                self.simulate_empty_ranges = False

            if self.simulate_look_to_sky:
                rospy.loginfo("SIMULATING LOOK TO SKY FAILURE")
                self.sim_info_pub.publish("republish velodyne: sim 'look to sky' fail")
                fake_ranges = [float('inf') for _ in range(len(scan.ranges))]
                scan.ranges = fake_ranges
                self.simulate_look_to_sky = False

            if self.simulate_scan_repetition:
                if self.previous_scan:
                    rospy.loginfo("SIMULATING SCAN REPETITION FAILURE")
                    self.sim_info_pub.publish("republish velodyne: sim scan repetition fail")
                    scan = self.previous_scan
                    self.simulate_scan_repetition = False
                else:
                    rospy.loginfo("cannot simulate scan repitition failure - there is no previous scan - wait until there is one..")
            
            self.scan_pub.publish(scan)
        else:
            rospy.loginfo("SIMULATING TOTAL SENSOR FAILURE..")
            self.sim_info_pub.publish("republish velodyne: sim total sensor fail")
            self.simulate_sensor_failure = False

        self.previous_scan = scan


def node():
    rospy.init_node('republish_velodyne')
    rospy.wait_for_message('SMACH_running', String)
    server = RepublishVelodyne()
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
