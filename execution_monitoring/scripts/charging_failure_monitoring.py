#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Bool
from execution_monitoring import config
from arox_performance_parameters.msg import arox_battery_params


class ChargingFailureMonitoring:
    """
    Provides monitoring solutions for charging failures.
    """

    def __init__(self):
        self.latest_charge_level = 0.0
        self.contingency_pub = rospy.Publisher('/contingency_preemption', String, queue_size=1)
        self.aggravate_pub = rospy.Publisher('/aggravate', String, queue_size=1)
        self.interrupt_reason_pub = rospy.Publisher('/interrupt_reason', String, queue_size=1)

        rospy.Subscriber('/explicit_charging_failure', String, self.explicit_failure_callback, queue_size=1)
        rospy.Subscriber('/charge_action', String, self.charge_monitoring, queue_size=1)
        rospy.Subscriber('/arox/battery_param', arox_battery_params, self.battery_callback, queue_size=1)
        rospy.Subscriber('/resolve_charging_failure_success', Bool, self.resolve_callback, queue_size=1)

    def resolve_callback(self, msg):
        """
        Resolver communication callback - determines whether resolution attempt was successful.

        @param msg: callback message - True: resolution successful / False: resolution failed
        """
        if not msg.data:
            self.interrupt_reason_pub.publish(config.CHARGING_CATA)
            self.aggravate_pub.publish(config.CHARGING_CATA)

    def battery_callback(self, msg):
        """
        Battery information callback - provides information about the current state of the battery parameters.

        @param msg: callback message - battery parameters
        """
        self.latest_charge_level = msg.charge

    def charge_monitoring(self, msg):
        """
        Monitors the state of charge after a charging action has been initiated.

        @param msg: callback message
        """
        start_charge = self.latest_charge_level
        rospy.sleep(config.CHARGING_FAILURE_TIME)
        if self.latest_charge_level <= start_charge:
            rospy.loginfo("CONTINGENCY: the level of charge does not increase, although it should..")
            self.contingency_pub.publish(config.CHARGING_FAILURE_THREE)

    def explicit_failure_callback(self, msg):
        """
        Handles explicit charging failures communicated by the operation state machine.

        @param msg: callback message - particular type of charging failure
        """
        rospy.loginfo("explicit charging failure communicated by operation SMACH..")
        self.contingency_pub.publish(msg.data)


def node():
    """
    Charging failure monitoring node.
    """
    rospy.init_node('charging_failure_monitoring')
    rospy.loginfo("launch charging failure monitoring..")
    ChargingFailureMonitoring()
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
