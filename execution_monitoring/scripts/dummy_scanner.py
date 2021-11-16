#!/usr/bin/env python
import rospy
import actionlib
from execution_monitoring.msg import ScanAction, ScanResult
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class DummyScanner():
    """
    Dummy node implementation that fakes the results of the RIEGL scanner. A scan action is expected to cause an incoming scan on the /RIEGL topic,
    which will then be written to a file to also have some form of data management. Since the Velodyne LiDAR sensor is already part of
    the simulation and the type of incoming data is essentially the same, a scan action just leads to one repuplished Velodyne scan under the /RIEGL topic.
    Thus, the idea is that the Velodyne sensor briefly pretends to be a RIEGL sensor. This approach has the useful side effect of making it relatively easy
    to simulate sensor failures by simply stopping to republish the Velodyne after a scan action.
    """

    def __init__(self):
        rospy.loginfo("initializing dummy scanner - waiting for tasks..")
        self.simulate_sensor_failure = False
        self.sensor_failure_sub = rospy.Subscriber("/toggle_simulated_sensor_failure", String, self.toggle_sensor_failure_callback, queue_size=1)
        self.scan_pub = rospy.Publisher("/RIEGL", LaserScan, queue_size=1)
        self.perform_action_pub = rospy.Publisher('/scan_action', String, queue_size=1)
        self.server = actionlib.SimpleActionServer('dummy_scanner', ScanAction, execute_cb=self.execute_cb, auto_start=False)
        self.result = ScanResult()
        self.server.start()

    def republish_velodyne(self):
        # create a new subscription to the topic, receive one message, then unsubscribe
        scan = rospy.wait_for_message("/scanVelodyne", LaserScan, timeout=60)
        if not self.simulate_sensor_failure:
            self.scan_pub.publish(scan)

    def toggle_sensor_failure_callback(self, msg):
        self.simulate_sensor_failure = not self.simulate_sensor_failure

    def execute_cb(self, goal):
        # TODO put name of the mission in goal to set it as file name of the scan results
        file_name = "test.txt"
        rospy.loginfo(goal)
        rospy.loginfo("start scanning procedure..")
        scan = None

        try:
            self.perform_action_pub.publish("action")
            self.republish_velodyne()
            # create a new subscription to the topic, receive one message, then unsubscribe
            scan = rospy.wait_for_message("/RIEGL", LaserScan, timeout=60)
        except rospy.ROSException as e:
            rospy.loginfo("problem retrieving laser scan: %s", e)
        rospy.sleep(3)
        
        if scan:
            rospy.loginfo("recorded scan..")
            rospy.loginfo("scan header: %s", scan.header)
            # TODO: replace absolute path - should be launch parameter
            with open("/home/docker/catkin_ws/src/execution_monitoring/execution_monitoring/scans/" + file_name, 'a') as out_file:
                out_file.write(str(scan.header))
                out_file.write("\n############################################\n############################################\n")

        self.result.result = "scanning successfully completed"
        self.server.set_succeeded(self.result)
        rospy.loginfo("scanning completed..")


def node():
    """
    Dummy node to simulate the scanning procedure, i.e. to scan on command.
    """
    rospy.init_node('dummy_scanner')
    rospy.wait_for_message('SMACH_runnning', String)
    server = DummyScanner()
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
