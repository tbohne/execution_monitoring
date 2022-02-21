#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float64, Bool
from execution_monitoring import config
from arox_performance_parameters.msg import arox_battery_params
import dynamic_reconfigure.client

class BatteryFailureSimulator:

    def __init__(self):
        rospy.Subscriber('/sim_power_management_contingency', String, self.sim_contingency_callback, queue_size=1)
        rospy.Subscriber('/sim_power_management_catastrophe', String, self.sim_catastrophe_callback, queue_size=1)
        rospy.Subscriber('/reset_discharge_rate', String, self.reset_discharge_rate_callback, queue_size=1)
        self.client = dynamic_reconfigure.client.Client("arox_battery", timeout=30)

    def reset_discharge_rate_callback(self, msg):
        rospy.loginfo("resetting discharge rate (back to normal)..")
        self.client.update_configuration({"discharge_rate": config.NORMAL_DISCHARGE_RATE})

    def sim_contingency_callback(self, msg):
        rospy.loginfo("sim PM contingency..")
        self.client.update_configuration({"discharge_rate": config.CONTINGENCY_DISCHARGE_RATE})

    def sim_catastrophe_callback(self, msg):
        rospy.loginfo("sim PM catastrohe..")
        self.client.update_configuration({"discharge_rate": config.CATASTROPHE_DISCHARGE_RATE})

def node():
    rospy.init_node('battery_failure_simulator')
    rospy.loginfo("launch battery failure simulator..")
    BatteryFailureSimulator()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
