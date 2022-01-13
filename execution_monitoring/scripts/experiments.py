#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from arox_performance_parameters.msg import arox_operational_param
from arox_performance_parameters.msg import arox_battery_params

class Experiment:

    def __init__(self):
        self.contingency_cnt = 0
        self.catastrophe_cnt = 0
        self.total_completed_goals = 0
        self.completed_goals_current_mission = 0
        self.total_goals_current_mission = 0
        self.operation_mode = None
        self.operation_time = 0
        self.battery_charge = 0
        self.battery_charging_cycle = 0

        rospy.Subscriber("/contingency_preemption", String, self.contingency_callback)
        rospy.Subscriber("/catastrophe_preemption", String, self.catastrophe_callback)
        rospy.Subscriber("/arox/ongoing_operation", arox_operational_param, self.operation_callback)
        rospy.Subscriber("/arox/battery_param", arox_battery_params, self.battery_callback)

        self.log_info()

    def operation_callback(self, msg):
        self.operation_mode = msg.operation_mode
        self.completed_goals_current_mission = msg.rewards_gained
        self.total_goals_current_mission = msg.total_tasks

        # when one mission is complete, increase the total counter
        if msg.rewards_gained == msg.total_tasks:
            self.total_completed_goals += msg.rewards_gained

    def battery_callback(self, msg):
        self.battery_charge = msg.charge
        self.battery_charging_cycle = msg.charging_cycle
        self.operation_time = msg.operation_time

    def contingency_callback(self, msg):
        self.contingency_cnt += 1

    def catastrophe_callback(self, msg):
        self.catastrophe_cnt += 1

    def log_info(self):
        while not rospy.is_shutdown():
            rospy.loginfo("###########################################################")
            rospy.loginfo("contingency cnt: %s", self.contingency_cnt)
            rospy.loginfo("catastrophe cnt: %s", self.catastrophe_cnt)
            rospy.loginfo("operation mode: %s", self.operation_mode)
            rospy.loginfo("operation time: %s", self.operation_time)
            rospy.loginfo("total completed goals: %s", self.total_completed_goals + self.completed_goals_current_mission)
            rospy.loginfo("goals curr mission: %s / %s", self.completed_goals_current_mission, self.total_goals_current_mission)
            rospy.loginfo("battery charge: %s, cycle: %s", self.battery_charge, self.battery_charging_cycle)
            rospy.loginfo("###########################################################")
            rospy.sleep(10)

def node():
    rospy.init_node('experiments')
    Experiment()
    rospy.loginfo("launch experiments node..")
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
