#!/usr/bin/env python
import rospy
from datetime import datetime
import random
from std_msgs.msg import String
from execution_monitoring import config
from arox_performance_parameters.msg import arox_operational_param
from arox_performance_parameters.msg import arox_battery_params


TOPIC_MSG_MAPPING = {
    "/toggle_simulated_total_sensor_failure": config.SENSOR_FAILURE_ONE,
    "/toggle_simulated_empty_ranges": config.SENSOR_FAILURE_TWO,
    "/toggle_simulated_impermissible_ranges": config.SENSOR_FAILURE_THREE,
    "/toggle_simulated_scan_repetition": config.SENSOR_FAILURE_FOUR,
    "/toggle_simulated_bad_wifi_link":  config.CONNECTION_FAILURE_ONE,
    "/toggle_simulated_bad_wifi_signal": config.CONNECTION_FAILURE_TWO,
    "/toggle_simulated_bad_wifi_bit_rate": config.CONNECTION_FAILURE_THREE,
    "/toggle_simulated_wifi_disconnect": config.CONNECTION_FAILURE_FOUR,
    "/sim_internet_connection_failure": config.CONNECTION_FAILURE_FIVE,
    "/toggle_simulated_bad_download": config.CONNECTION_FAILURE_SIX,
    "/toggle_simulated_bad_upload": config.CONNECTION_FAILURE_SEVEN,
    "/toggle_simulated_timeout_failure": config.CONNECTION_FAILURE_EIGHT,
    "/set_simulated_unknown_status": config.CONNECTION_FAILURE_ELEVEN,
    "/set_simulated_no_fix": config.CONNECTION_FAILURE_TWELVE,
    "/set_simulated_no_rtk": config.CONNECTION_FAILURE_THIRTEEN,
    "/toggle_simulated_unknown_service": config.CONNECTION_FAILURE_FOURTEEN,
    "/toggle_simulated_infeasible_lat_lng": [config.CONNECTION_FAILURE_FIFTEEN, config.CONNECTION_FAILURE_SIXTEEN],
    "/toggle_simulated_variance_history_failure": config.CONNECTION_FAILURE_TWENTY,
    "/toggle_simulated_high_deviation": config.CONNECTION_FAILURE_EIGHTEEN,
    "/toggle_rain_sim": config.WEATHER_FAILURE_TWO,
    "/toggle_snow_sim": config.WEATHER_FAILURE_FIVE,
    "/toggle_wind_sim": config.WEATHER_FAILURE_EIGHT,
    "/toggle_low_temp_sim": config.WEATHER_FAILURE_TWELVE,
    "/toggle_thuderstorm_sim": config.WEATHER_FAILURE_THIRTEEN,
    "/toggle_sunset_sim": config.WEATHER_FAILURE_SEVENTEEN,
    "/sim_full_disk_failure": config.DATA_MANAGEMENT_FAILURE_ONE,
    "/toggle_simulated_scan_logging_failure": config.DATA_MANAGEMENT_FAILURE_TWO,
    "/wheel_movement_without_pos_change": [config.LOCALIZATION_FAILURE_ONE, config.LOCALIZATION_FAILURE_TWO],
    "/pos_change_without_wheel_movement": [config.LOCALIZATION_FAILURE_ONE, config.LOCALIZATION_FAILURE_TWO],
    "/yaw_divergence": [config.LOCALIZATION_FAILURE_FOUR, config.LOCALIZATION_FAILURE_FIVE],
    "/moving_although_standing_still_imu": [config.LOCALIZATION_FAILURE_SEVEN, config.LOCALIZATION_FAILURE_EIGHT],
    "/moving_although_standing_still_odom": [config.LOCALIZATION_FAILURE_ONE, config.LOCALIZATION_FAILURE_TWO],
    "sim_extended_idle_time": config.PLAN_DEPLOYMENT_FAILURE_ONE,
    "toggle_unavailable_plan_service": config.PLAN_DEPLOYMENT_FAILURE_TWO,
    "sim_empty_plan": config.PLAN_DEPLOYMENT_FAILURE_THREE,
    "sim_infeasible_plan": config.PLAN_DEPLOYMENT_FAILURE_FOUR,
    "spawn_static_obstacles": [],
    "spawn_robot_prison": [config.NAV_FAILURE_ONE, config.NAV_FAILURE_THREE],
    "trigger_nav_fail": config.NAV_FAILURE_ONE,
    "sim_undocking_failure": config.CHARGING_FAILURE_TWO,
    "sim_docking_failure_raised_ramp": config.CHARGING_FAILURE_ONE,
    "sim_docking_failure_base_pose": config.CHARGING_FAILURE_ONE,
    "sim_charging_failure": config.CHARGING_FAILURE_THREE,
    "sim_power_management_contingency": config.POWER_MANAGEMENT_FAILURE_ONE
}


# random fail every 15 minutes
RANDOM_FAIL_FREQUENCY = 300 #900
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
        self.expected_contingency = None

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
        rospy.loginfo("CONTINIGENCY: %s", msg.data)
        rospy.loginfo("EXPECTED: %s", self.expected_contingency)
        if isinstance(self.expected_contingency, list):
            if msg.data in self.expected_contingency:
                self.expected_contingency = None
        else:
            if msg.data == self.expected_contingency:
                self.expected_contingency = None
        self.contingency_cnt += 1

    def catastrophe_callback(self, msg):
        self.catastrophe_cnt += 1

    def simulate_random_failure(self):
        global SEED, TOPIC_MSG_MAPPING
        self.sim_fail_time = datetime.now()
        rand = random.randint(0, len(TOPIC_MSG_MAPPING.keys()) - 1)
        random_topic = TOPIC_MSG_MAPPING.keys()[rand]
        rospy.loginfo("###################################################################")
        rospy.loginfo("###################################################################")
        rospy.loginfo("init random failure: %s", random_topic)
        self.expected_contingency = TOPIC_MSG_MAPPING[random_topic]
        rospy.loginfo("expected contingency: %s", self.expected_contingency)
        rospy.loginfo("###################################################################")
        rospy.loginfo("###################################################################")
        pub = rospy.Publisher(random_topic, String, queue_size=1)
        # needs a moment to init the publisher
        rospy.sleep(3)
        pub.publish("sim fail")

    def run_experiment(self):
        while not rospy.is_shutdown():

            if (datetime.now() - self.sim_fail_time).total_seconds() >= RANDOM_FAIL_FREQUENCY and self.expected_contingency is None:
                self.simulate_random_failure()

            rospy.loginfo("time since last fail sim: %s", (datetime.now() - self.sim_fail_time).total_seconds())
            #self.log_info()
            rospy.sleep(120)

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
