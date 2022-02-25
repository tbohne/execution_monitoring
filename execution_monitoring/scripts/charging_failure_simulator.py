#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float64

class ChargingFailureSimulator:

    def __init__(self):
        self.sim_undocking_fail = False
        rospy.Subscriber('/sim_undocking_failure', String, self.undocking_fail_callback, queue_size=1)
        rospy.Subscriber('/sim_docking_failure_raised_ramp', String, self.docking_fail_callback, queue_size=1)
        rospy.Subscriber('/charge_action', String, self.charge_callback, queue_size=1)
        self.sim_info_pub = rospy.Publisher('/sim_info', String, queue_size=1)

    def docking_fail_callback(self, msg):
        rospy.loginfo("sim docking fail..")
        self.raise_container_ramp()

    def charge_callback(self, msg):
        if self.sim_undocking_fail:
            rospy.loginfo("sim undocking fail..")
            self.sim_info_pub.publish("charging failure monitoring: sim undocking failure")
            self.raise_container_ramp()
            self.sim_undocking_fail = False

    def undocking_fail_callback(self, msg):
        rospy.loginfo("preparing undocking fail sim..")
        self.sim_undocking_fail = True

    def raise_container_ramp(self):
        rospy.loginfo("raising container ramp..")
        self.sim_info_pub.publish("charging failure monitoring: sim failure -- raising container ramp")
        container_pub = rospy.Publisher('/container/rampB_position_controller/command', Float64, queue_size=1)
        for _ in range(3):
            container_pub.publish(0.0)
            rospy.sleep(0.5)

def node():
    rospy.init_node('charging_failure_simulator')
    rospy.loginfo("launch charging failure simulator..")
    ChargingFailureSimulator()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
