#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Bool
from execution_monitoring import config
from arox_performance_parameters.msg import arox_battery_params

class ChargingFailureMonitoring:

    def __init__(self):
        rospy.Subscriber('/explicit_charging_failure', String, self.explicit_failure_callback, queue_size=1)
        rospy.Subscriber('/charge_action', String, self.charge_monitoring, queue_size=1)
        rospy.Subscriber('/arox/battery_param', arox_battery_params, self.battery_callback, queue_size=1)
        rospy.Subscriber('/resolve_charging_failure_success', Bool, self.resolve_callback, queue_size=1)
        self.contingency_pub = rospy.Publisher('/contingency_preemption', String, queue_size=1)
        self.catastrophe_pub = rospy.Publisher('/catastrophe_preemption', String, queue_size=1)
        self.latest_charge_level = 0.0

    def resolve_callback(self, msg):
        if not msg.data:
            self.catastrophe_pub.publish(config.CHARGING_CATA)

    def battery_callback(self, msg):
        self.latest_charge_level = msg.charge

    def charge_monitoring(self, msg):
        start_charge = self.latest_charge_level
        rospy.sleep(config.CHARGING_FAILURE_TIME)
        if self.latest_charge_level <= start_charge:
            rospy.loginfo("CONTINGENCY: charge level not increasesing although it should..")
            self.contingency_pub.publish(config.CHARGING_FAILURE_THREE)

    def explicit_failure_callback(self, msg):
        rospy.loginfo("explicit charging fail communicated by operation smach..")
        self.contingency_pub.publish(msg.data)

def node():
    rospy.init_node('charging_failure_monitoring')
    rospy.loginfo("launch charging failure monitoring..")
    ChargingFailureMonitoring()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
