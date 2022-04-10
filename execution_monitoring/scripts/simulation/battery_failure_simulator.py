#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float64, Bool
from execution_monitoring import config
import dynamic_reconfigure.client


class BatteryFailureSimulator:
    """
    Provides simulations of battery failure cases.
    """

    def __init__(self):
        self.client = dynamic_reconfigure.client.Client("arox_battery", timeout=30)
        self.sim_contingency = False
        self.sim_catastrophe = False
        rospy.Subscriber('/sim_power_management_contingency', String, self.sim_contingency_callback, queue_size=1)
        rospy.Subscriber('/sim_power_management_catastrophe', String, self.sim_catastrophe_callback, queue_size=1)
        rospy.Subscriber('/reset_discharge_rate', String, self.reset_discharge_rate_callback, queue_size=1)

    def reset_discharge_rate_callback(self, msg):
        """
        Resets the discharge rate of the battery model to the default value after failure.

        @param msg: callback message - type of power management failure
        """
        if msg.data == config.POWER_MANAGEMENT_FAILURES[0] and self.sim_contingency:
            rospy.loginfo("resetting discharge rate after contingency (back to normal)..")
            self.sim_contingency = False
            self.client.update_configuration({"discharge_rate": config.NORMAL_DISCHARGE_RATE})
        elif msg.data == config.POWER_MANAGEMENT_CATA and self.sim_catastrophe:
            rospy.loginfo("resetting discharge rate after catastrophe (back to normal)..")
            self.sim_catastrophe = False
            self.client.update_configuration({"discharge_rate": config.NORMAL_DISCHARGE_RATE})

    def sim_contingency_callback(self, msg):
        """
        Initiates power management contingency by manipulating the battery model's discharge rate.

        @param msg: callback message
        """
        rospy.loginfo("sim PM contingency..")
        self.sim_contingency = True
        self.client.update_configuration({"discharge_rate": config.CONTINGENCY_DISCHARGE_RATE})

    def sim_catastrophe_callback(self, msg):
        """
        Initiates power management catastrophe by manipulating the battery model's discharge rate.

        @param msg: callback message
        """
        rospy.loginfo("sim PM catastrophe..")
        self.sim_catastrophe = True
        self.client.update_configuration({"discharge_rate": config.CATASTROPHE_DISCHARGE_RATE})


def node():
    """
    Battery failure simulation node.
    """
    rospy.init_node('battery_failure_simulator')
    rospy.loginfo("launch battery failure simulator..")
    BatteryFailureSimulator()
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
