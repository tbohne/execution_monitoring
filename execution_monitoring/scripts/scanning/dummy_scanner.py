#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @author Tim Bohne

import actionlib
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from execution_monitoring import util, config
from execution_monitoring.msg import ScanAction, ScanResult


class DummyScanner:
    """
    Dummy scanner to simulate the scanning procedure, i.e., to scan on command in simulation.
    """

    def __init__(self):
        rospy.loginfo("initializing dummy scanner - waiting for tasks..")
        self.simulate_scan_logging_failure = False
        self.result = ScanResult()
        self.mission_name = ""

        self.perform_action_pub = rospy.Publisher('/scan_action', String, queue_size=1)
        self.completed_action_pub = rospy.Publisher('/scan_completed', String, queue_size=1)
        self.sim_info_pub = rospy.Publisher('/sim_info', String, queue_size=1)

        rospy.Subscriber('/mission_name', String, self.mission_name_callback, queue_size=1)
        rospy.Subscriber("/toggle_simulated_scan_logging_failure", String, self.toggle_scan_failure_callback,
                         queue_size=1)
        self.server = actionlib.SimpleActionServer('dummy_scanner', ScanAction, execute_cb=self.execute_cb,
                                                   auto_start=False)
        self.server.start()

    def mission_name_callback(self, mission_name):
        """
        Called when mission name is communicated.

        @param mission_name: communicated mission name
        """
        self.mission_name = util.parse_mission_name(mission_name)

    def toggle_scan_failure_callback(self, msg):
        """
        (De)activates scanning failure simulation flag.

        @param msg: callback message
        """
        self.simulate_scan_logging_failure = not self.simulate_scan_logging_failure

    def execute_cb(self, goal):
        """
        Executes a scanning procedure.

        @param goal: action goal
        """
        rospy.loginfo("start scanning procedure..")
        scan = None
        try:
            self.perform_action_pub.publish("action")
            # create a new subscription to the topic, receive one message, then unsubscribe
            scan = rospy.wait_for_message(config.SCAN_TOPIC, LaserScan, timeout=config.SCAN_TIME_LIMIT)
        except rospy.ROSException as e:
            rospy.loginfo("problem retrieving laser scan: %s", e)
        rospy.sleep(config.SCAN_TIME)

        if scan:
            rospy.loginfo("recorded scan..")
            if self.mission_name == "":
                self.mission_name = "unknown"
                rospy.loginfo("no mission name - storing scans in %s", config.SCAN_PATH + self.mission_name + ".txt")

            if not self.simulate_scan_logging_failure:
                try:
                    with open(config.SCAN_PATH + self.mission_name + config.SCAN_FILE_EXTENSION, 'a') as out_file:
                        out_file.write(str(scan))
                        out_file.write("\n############################################\n"
                                       + "############################################\n")
                except Exception as e:
                    rospy.loginfo("EXCEPTION during scan logging: %s", e)
            else:
                rospy.loginfo("sim scan failure..")
                self.sim_info_pub.publish("dummy scanner: sim scan failure")
                self.simulate_scan_logging_failure = False

            self.result.result = "scanning successfully completed"
            self.server.set_succeeded(self.result)
            rospy.loginfo("scanning completed..")
            self.completed_action_pub.publish("")
        else:
            self.result.result = "scanning failed"
            self.server.set_aborted(self.result)


def node():
    """
    Dummy node to simulate the scanning procedure, i.e., to scan on command.
    """
    rospy.init_node('dummy_scanner')
    rospy.wait_for_message('SMACH_running', String)
    DummyScanner()
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
