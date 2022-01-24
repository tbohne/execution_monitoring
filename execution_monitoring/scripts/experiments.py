#!/usr/bin/env python
import rospy
from datetime import datetime
import random
from std_msgs.msg import String
from arox_performance_parameters.msg import arox_operational_param
from arox_performance_parameters.msg import arox_battery_params

FAILURE_TOPICS = ["/toggle_simulated_total_sensor_failure", "/toggle_simulated_empty_ranges", "/toggle_simulated_impermissible_ranges", "/toggle_simulated_scan_repetition",
    "/toggle_simulated_bad_wifi_link", "/toggle_simulated_bad_wifi_signal", "/toggle_simulated_bad_wifi_bit_rate", "/toggle_simulated_wifi_disconnect", "/toggle_simulated_bad_download",
    "/toggle_simulated_bad_upload", "/toggle_simulated_timeout_failure", "/set_simulated_good_quality", "/set_simulated_med_quality", "/set_simulated_low_quality",
    "/set_simulated_unknown_status", "/set_simulated_no_fix", "/set_simulated_no_rtk", "/toggle_simulated_unknown_service", "/toggle_simulated_infeasible_lat_lng",
    "/toggle_simulated_variance_history_failure", "/toggle_simulated_high_deviation", "/toggle_rain_sim", "/toggle_snow_sim", "/toggle_wind_sim", "/toggle_low_temp_sim",
    "/toggle_thuderstorm_sim", "/toggle_sunset_sim", "/toggle_simulated_scan_logging_failure", "/wheel_movement_without_pos_change", "/pos_change_without_wheel_movement",
    "/yaw_divergence", "/moving_although_standing_still_imu", "/moving_although_standing_still_odom"]

# random fail every 60s
RANDOM_FAIL_FREQUENCY = 60
SEED = 42

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
        self.sim_fail_time = datetime.now()

        rospy.Subscriber("/contingency_preemption", String, self.contingency_callback)
        rospy.Subscriber("/catastrophe_preemption", String, self.catastrophe_callback)
        rospy.Subscriber("/arox/ongoing_operation", arox_operational_param, self.operation_callback)
        rospy.Subscriber("/arox/battery_param", arox_battery_params, self.battery_callback)

        self.run_experiment()

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

    def simulate_random_failure(self):
        global SEED, FAILURE_TOPICS
        self.sim_fail_time = datetime.now()
        rand = random.randint(0, len(FAILURE_TOPICS) - 1)
        random_topic = FAILURE_TOPICS[rand]
        rospy.loginfo("init random failure: %s", random_topic)
        pub = rospy.Publisher(random_topic, String, queue_size=1)
        # needs a moment to init the publisher
        rospy.sleep(3)
        pub.publish("sim fail")

    def run_experiment(self):
        while not rospy.is_shutdown():

            if (datetime.now() - self.sim_fail_time).total_seconds() >= RANDOM_FAIL_FREQUENCY:
                self.simulate_random_failure()

            rospy.loginfo("time since last fail sim: %s", (datetime.now() - self.sim_fail_time).total_seconds())
            #self.log_info()
            rospy.sleep(1)

    def log_info(self):
        rospy.loginfo("###########################################################")
        rospy.loginfo("contingency cnt: %s", self.contingency_cnt)
        rospy.loginfo("catastrophe cnt: %s", self.catastrophe_cnt)
        rospy.loginfo("operation mode: %s", self.operation_mode)
        rospy.loginfo("operation time: %s", self.operation_time)
        rospy.loginfo("total completed goals: %s", self.total_completed_goals + self.completed_goals_current_mission)
        rospy.loginfo("goals curr mission: %s / %s", self.completed_goals_current_mission, self.total_goals_current_mission)
        rospy.loginfo("battery charge: %s, cycle: %s", self.battery_charge, self.battery_charging_cycle)
        rospy.loginfo("###########################################################")

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
