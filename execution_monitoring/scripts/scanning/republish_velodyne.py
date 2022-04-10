#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from execution_monitoring import config


class RepublishVelodyne:
    """
    Minimalistic class that republishes a Velodyne scan under the "/RIEGL" topic after a scan action has been
    initiated. Since the Velodyne lidar sensor is already part of the simulation and the type of incoming data is
    essentially the same, a scan action just leads to one republished Velodyne scan under the "/RIEGL" topic. Thus,
    the idea is that the Velodyne sensor briefly pretends to be a RIEGL sensor. This approach has the useful side
    effect of making it relatively easy to simulate sensor failures by simply stopping to republish the Velodyne after
    a scan action.
    """

    def __init__(self):
        self.simulate_sensor_failure = False
        self.simulate_empty_ranges = False
        self.simulate_invalid_ranges = False
        self.simulate_scan_repetition = False
        self.previous_scan = None

        self.scan_pub = rospy.Publisher("/RIEGL", LaserScan, queue_size=1)
        self.sim_info_pub = rospy.Publisher('/sim_info', String, queue_size=1)

        rospy.Subscriber("/toggle_simulated_total_sensor_failure", String, self.sensor_fail_callback, queue_size=1)
        rospy.Subscriber("/toggle_simulated_empty_ranges", String, self.toggle_empty_ranges_callback, queue_size=1)
        rospy.Subscriber("/toggle_simulated_impermissible_ranges", String, self.invalid_ranges_callback, queue_size=1)
        rospy.Subscriber("/toggle_simulated_scan_repetition", String, self.scan_repetition_callback, queue_size=1)
        rospy.Subscriber('/scan_action', String, self.action_callback, queue_size=1)

    def sensor_fail_callback(self, msg):
        """
        Initiates the simulation of a total sensor failure.

        @param msg: callback message
        """
        self.simulate_sensor_failure = not self.simulate_sensor_failure

    def toggle_empty_ranges_callback(self, msg):
        """
        Initiates the simulation of empty ranges in the scanning data.

        @param msg: callback message
        """
        self.simulate_empty_ranges = not self.simulate_empty_ranges

    def invalid_ranges_callback(self, msg):
        """
        Initiates the simulation of impermissible range values in the scanning data.

        @param msg: callback message
        """
        self.simulate_invalid_ranges = not self.simulate_invalid_ranges

    def scan_repetition_callback(self, msg):
        """
        Initiates the simulation of repeated scans.

        @param msg: callback message
        """
        self.simulate_scan_repetition = not self.simulate_scan_repetition

    def action_callback(self, msg):
        """
        Performs one simulated scan action - republishes one Velodyne scan under the RIEGL topic.

        @param msg: callback message
        """
        # create a new subscription to the topic, receive one message, then unsubscribe
        scan = rospy.wait_for_message("/scanVelodyne", LaserScan, timeout=config.SCAN_TIME_LIMIT)

        if not self.simulate_sensor_failure:
            if self.simulate_empty_ranges:
                rospy.loginfo("SIMULATING EMPTY RANGES FAILURE")
                self.sim_info_pub.publish("republish velodyne: sim empty ranges fail")
                scan.ranges = []
                self.simulate_empty_ranges = False

            if self.simulate_invalid_ranges:
                rospy.loginfo("SIMULATING INVALID RANGES FAILURE")
                self.sim_info_pub.publish("republish velodyne: sim invalid ranges fail")
                fake_ranges = [float('inf') for _ in range(len(scan.ranges))]
                scan.ranges = fake_ranges
                self.simulate_invalid_ranges = False

            if self.simulate_scan_repetition:
                if self.previous_scan:
                    rospy.loginfo("SIMULATING SCAN REPETITION FAILURE")
                    self.sim_info_pub.publish("republish velodyne: sim scan repetition fail")
                    scan = self.previous_scan
                    self.simulate_scan_repetition = False
                else:
                    rospy.loginfo("cannot simulate scan repetition failure - there is no previous scan"
                                  + " - wait until there is one..")
            self.scan_pub.publish(scan)
        else:
            rospy.loginfo("SIMULATING TOTAL SENSOR FAILURE..")
            self.sim_info_pub.publish("republish velodyne: sim total sensor fail")
            self.simulate_sensor_failure = False
        self.previous_scan = scan


def node():
    """
    Minimalistic node that republishes a Velodyne scan under the "/RIEGL" topic after a scan action has been initiated.
    """
    rospy.init_node('republish_velodyne')
    rospy.wait_for_message('SMACH_running', String)
    RepublishVelodyne()
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
