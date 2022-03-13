#!/usr/bin/env python
import rospy
from datetime import datetime
import random
from std_msgs.msg import String
from execution_monitoring import config, util
from arox_performance_parameters.msg import arox_operational_param
from arox_performance_parameters.msg import arox_battery_params
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf2_ros
import math

# would break the mission -- it's a cata in 100% of the cases
CATA_TOPIC_MSG_MAPPING = {
    # prerequisite: prepare full USB flash drive + mount it under config.FULL_DRIVE, e.g. "/mnt/usb"
    "/sim_full_disk_failure": config.DATA_MANAGEMENT_FAILURE_ONE,
    "/spawn_robot_prison": [config.NAV_FAILURE_ONE, config.NAV_FAILURE_THREE],
    "/sim_docking_failure_raised_ramp": config.CHARGING_FAILURE_ONE,
    "/sim_undocking_failure": config.CHARGING_FAILURE_TWO,
    "/sim_power_management_catastrophe": config.POWER_MANAGEMENT_CATA
}

CONT_TOPIC_MSG_MAPPING = {
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
    "/set_simulated_no_rtk": [],
    "/toggle_simulated_infeasible_lat_lng": [config.CONNECTION_FAILURE_FIFTEEN, config.CONNECTION_FAILURE_SIXTEEN],
    "/toggle_simulated_variance_history_failure": config.CONNECTION_FAILURE_TWENTY,
    "/toggle_simulated_high_deviation": config.CONNECTION_FAILURE_EIGHTEEN,
    "/toggle_rain_sim": config.WEATHER_FAILURE_TWO,
    "/toggle_snow_sim": config.WEATHER_FAILURE_FIVE,
    "/toggle_wind_sim": config.WEATHER_FAILURE_EIGHT,
    "/toggle_low_temp_sim": config.WEATHER_FAILURE_TWELVE,
    "/toggle_thuderstorm_sim": config.WEATHER_FAILURE_THIRTEEN,
    "/toggle_sunset_sim": config.WEATHER_FAILURE_SEVENTEEN,
    "/toggle_simulated_scan_logging_failure": config.DATA_MANAGEMENT_FAILURE_TWO,
    "/wheel_movement_without_pos_change": [config.LOCALIZATION_FAILURE_ONE, config.LOCALIZATION_FAILURE_TWO],
    "/pos_change_without_wheel_movement": [config.LOCALIZATION_FAILURE_ONE, config.LOCALIZATION_FAILURE_TWO],
    "/yaw_divergence": [config.LOCALIZATION_FAILURE_FOUR, config.LOCALIZATION_FAILURE_FIVE],
    "/moving_although_standing_still_imu": [config.LOCALIZATION_FAILURE_SEVEN, config.LOCALIZATION_FAILURE_EIGHT],
    "/moving_although_standing_still_odom": [config.LOCALIZATION_FAILURE_ONE, config.LOCALIZATION_FAILURE_TWO],
    "/sim_extended_idle_time": config.PLAN_DEPLOYMENT_FAILURE_ONE,
    "/toggle_unavailable_plan_service": config.PLAN_DEPLOYMENT_FAILURE_TWO,
    "/sim_empty_plan": config.PLAN_DEPLOYMENT_FAILURE_THREE,
    "/sim_infeasible_plan": config.PLAN_DEPLOYMENT_FAILURE_FOUR,
    "/spawn_static_obstacles": [],
    "/spawn_static_obstacles": [],
    "/toggle_simulated_unknown_service": [],
    "/trigger_nav_fail": config.NAV_FAILURE_ONE,
    "/sim_docking_failure_base_pose": config.CHARGING_FAILURE_ONE,
    "/sim_charging_failure": config.CHARGING_FAILURE_THREE,
    "/sim_power_management_contingency": config.POWER_MANAGEMENT_FAILURE_ONE
}

RANDOM_FAIL_FREQUENCY = 250 # random fail every 250s
SEED = 42
EXPERIMENT_DURATION = 18000 # 14400 # 4 hours
IDX = 0
SIM_FAILURES = True
PLAN_LENGTH = 34
TF_BUFFER = None

class Experiment:

    def __init__(self):
        global SEED
        random.seed(SEED)
        self.expected_contingency_cnt = 0
        self.false_positive_contingency = 0
        self.issue_expected_without_contingy_and_fulfilled = 0
        self.false_negative_contingency = 0
        self.unexpected_contingency_cnt = 0
        self.catastrophe_cnt = 0
        self.completed_goals = 0
        self.total_goals_current_mission = 0
        self.operation_mode = None
        self.operation_time = 0
        self.battery_charge = 0
        self.battery_charging_cycle = 1
        self.mission_cycle = 1
        self.sim_fail_time = datetime.now()
        self.expected_contingency = None
        self.sim_launched = True
        self.open_goals_prev = None
        self.mode_times = {'traversing': 0, 'scanning': 0, 'waiting': 0, 'catastrophe': 0, 'contingency': 0, 'charging': 0, 'docking': 0, 'undocking': 0}
        self.prev_mode = None
        self.prev_start = None
        self.prev_pose = None
        self.arox_total_dist = 0
        self.simulated_problems = 0

        rospy.Subscriber("/contingency_preemption", String, self.contingency_callback)
        rospy.Subscriber("/catastrophe_preemption", String, self.catastrophe_callback)
        rospy.Subscriber("/arox/ongoing_operation", arox_operational_param, self.operation_callback)
        rospy.Subscriber("/arox/battery_param", arox_battery_params, self.battery_callback)
        rospy.Subscriber('/sim_info', String, self.sim_info_callback, queue_size=1)
        rospy.Subscriber('/odometry/filtered_odom', Odometry, self.filtered_odom_callback, queue_size=1)

        self.run_experiment()

    def filtered_odom_callback(self, msg):
        curr_pose = PoseStamped()
        curr_pose.header.frame_id = msg.header.frame_id
        curr_pose.pose.position.x = msg.pose.pose.position.x
        curr_pose.pose.position.y = msg.pose.pose.position.y
        curr_pose.pose.orientation.z = msg.pose.pose.orientation.z
        curr_pose.pose.orientation.w = msg.pose.pose.orientation.w
            
        try:
            curr_pose_map = util.transform_pose(TF_BUFFER, curr_pose, 'map')

            if not self.prev_pose:
                self.prev_pose = curr_pose_map
            else:
                dist = math.sqrt((self.prev_pose.pose.position.x - curr_pose_map.pose.position.x) ** 2 
                    + (self.prev_pose.pose.position.y - curr_pose_map.pose.position.y) ** 2)

                self.arox_total_dist += dist
                self.prev_pose = curr_pose_map
                
        except Exception as e:
            rospy.loginfo("ERROR: %s", e)

    def sim_info_callback(self, msg):
        # by this, we know when there is a msg on a fail sim topic and thus when a contingency is expected
        rospy.loginfo("SIM LAUNCHED --------------------------")
        self.simulated_problems += 1
        self.sim_launched = True
        # reset timer -- sim only launched now -- next shouldn't follow immediately, e.g. docking
        self.sim_fail_time = datetime.now()

    def operation_callback(self, msg):
        self.operation_mode = msg.operation_mode

        if self.operation_mode != self.prev_mode:
            if self.prev_mode:
                self.mode_times[self.prev_mode] += (datetime.now() - self.prev_start).total_seconds()
            if not self.operation_mode in self.mode_times.keys():
                self.mode_times[self.operation_mode] = 0
            self.prev_mode = self.operation_mode
            self.prev_start = datetime.now()
        self.completed_goals = msg.rewards_gained

        # new mission
        if msg.total_tasks == PLAN_LENGTH and self.open_goals_prev == 1:
            self.mission_cycle += 1

        self.open_goals_prev = msg.total_tasks

    def battery_callback(self, msg):
        self.battery_charge = msg.charge
        self.battery_charging_cycle = msg.charging_cycle
        self.operation_time = msg.operation_time

    def contingency_callback(self, msg):
        rospy.loginfo("------------------------------------------------")
        rospy.loginfo("CONTINIGENCY: %s", msg.data)
        rospy.loginfo("EXPECTED: %s", self.expected_contingency)
        rospy.loginfo("------------------------------------------------")

        # expected contingency -- answer to simulated failure
        if isinstance(self.expected_contingency, list):
            if msg.data in self.expected_contingency:
                self.expected_contingency = None
                self.expected_contingency_cnt += 1
            # expected a contingency, but not the present one
            else:
                # keep the expected one - could still come
                self.unexpected_contingency_cnt += 1
        elif self.expected_contingency is not None:
            if msg.data == self.expected_contingency:
                self.expected_contingency = None
                self.expected_contingency_cnt += 1
            # expected a contingency, but not the present one
            else:
                # keep the expected one - could still come
                self.unexpected_contingency_cnt += 1
        elif self.expected_contingency is None:
            # false positive -- contingency although no failure sim
            self.false_positive_contingency += 1

        self.log_info()

    def catastrophe_callback(self, msg):
        self.catastrophe_cnt += 1

    def simulate_random_failure(self):
        global CONT_TOPIC_MSG_MAPPING
        # shouldn't simulate any new failures during docking or when last sim has not even started
        if self.operation_mode == "docking" or not self.sim_launched:
            rospy.loginfo("NOT SIMULATING DUE TO DOCKING / NOT SIM LAUNCHED ------------------")
            return

        rand = random.randint(0, len(CONT_TOPIC_MSG_MAPPING.keys()) - 1)
        random_topic = CONT_TOPIC_MSG_MAPPING.keys()[rand]

        # doesn't make any sense to simulate IDLE time during active mission
        if random_topic in ["/sim_extended_idle_time", "/toggle_unavailable_plan_service", "/sim_empty_plan", "/sim_infeasible_plan"] and self.operation_mode != "waiting":
            rospy.loginfo("topic: %s currently not feasible -- selecting another one..", random_topic)
            self.simulate_random_failure()
        else:
            self.sim_fail_time = datetime.now()
            self.sim_launched = False
            rospy.loginfo("###################################################################")
            rospy.loginfo("###################################################################")
            rospy.loginfo("init random failure: %s", random_topic)

            if self.expected_contingency == []:
                self.issue_expected_without_contingy_and_fulfilled += 1
                self.expected_contingency = None

            # previously expected contingency was not detected -> false negative
            if self.expected_contingency is not None:
                self.false_negative_contingency += 1

            self.expected_contingency = CONT_TOPIC_MSG_MAPPING[random_topic]
            rospy.loginfo("expected contingency: %s", self.expected_contingency)
            rospy.loginfo("###################################################################")
            rospy.loginfo("###################################################################")
            pub = rospy.Publisher(random_topic, String, queue_size=1)
            # needs a moment to init the publisher
            rospy.sleep(3)
            pub.publish("sim fail")

    def run_experiment(self):

        start_time = datetime.now()

        while not rospy.is_shutdown() and (datetime.now() - start_time).total_seconds() < EXPERIMENT_DURATION:

            # assumption -- there is enough time to complete all the simulated failures, e.g. docking fail
            #       - docking does not occur every 2 minutes, it can take a while to get in this situation
            #       - thus, docking shouldn't be entirely random, but only when the last docking is 
            # elegant way to avoid these issues: wait until sim is actually launched, not until sim launch is "prepared"
            if SIM_FAILURES and (datetime.now() - self.sim_fail_time).total_seconds() >= RANDOM_FAIL_FREQUENCY and self.sim_launched: # and self.expected_contingency is None:
                self.simulate_random_failure()

            if SIM_FAILURES:
                rospy.loginfo("time since last fail sim: %s", (datetime.now() - self.sim_fail_time).total_seconds())

            rospy.sleep(120)

        self.save_results((datetime.now() - start_time).total_seconds() / 60 / 60)

    def save_results(self, duration):
        name = str(RANDOM_FAIL_FREQUENCY) + "_" + str(SEED) + "_" + str(IDX)
        completed = duration >= 4 and self.catastrophe_cnt == 0
        try:
            with open(config.EXP_PATH + "results.csv", 'a') as out_file:
                if IDX == 0:
                    out_file.write("experiment,duration,correct_contingencies,false_positives,false_negatives,correct_no_contingency,unexpected_contingencies,completed,completed_tasks,charge_cycles,mission_cycles,total_dist,simulated_problems,traverse_time,scan_time,wait_time,cata_time,cont_time,charge_time,dock_time,undock_time\n")
                out_file.write(name + "," + str(duration) + "," + str(self.expected_contingency_cnt) + "," + str(self.false_positive_contingency) + "," + str(self.false_negative_contingency)
                + "," + str(self.issue_expected_without_contingy_and_fulfilled) + "," + str(self.unexpected_contingency_cnt) + "," + str(completed) + "," + str(self.completed_goals)
                + "," + str(self.battery_charging_cycle + 1) + "," + str(self.mission_cycle) + "," + str(self.arox_total_dist) + "," + str(self.simulated_problems) + ","
                + str(self.mode_times['traversing']) + "," + str(self.mode_times['scanning']) + "," + str(self.mode_times['waiting']) + "," + str(self.mode_times['catastrophe']) + ","
                + str(self.mode_times['contingency']) + "," + str(self.mode_times['charging']) + "," + str(self.mode_times['docking']) + "," + str(self.mode_times['undocking']) + "\n")
        except Exception as e:
            rospy.loginfo("EXCEPTION during storage of experiment results: %s", e)

    def log_info(self):
        rospy.loginfo("###########################################################")
        rospy.loginfo("contingency cnt: %s", self.expected_contingency_cnt)
        rospy.loginfo("false positives: %s", self.false_positive_contingency)
        rospy.loginfo("false negatives: %s", self.false_negative_contingency)
        rospy.loginfo("postively handled issue without CONT: %s", self.issue_expected_without_contingy_and_fulfilled)
        rospy.loginfo("unexpected contingency: %s", self.unexpected_contingency_cnt)
        rospy.loginfo("catastrophe cnt: %s", self.catastrophe_cnt)
        rospy.loginfo("operation mode: %s", self.operation_mode)
        rospy.loginfo("operation time: %s", self.operation_time)
        rospy.loginfo("completed goals: %s", self.completed_goals)
        rospy.loginfo("battery charge: %s, cycle: %s", self.battery_charge, self.battery_charging_cycle + 1)
        rospy.loginfo("mission cycle: %s", self.mission_cycle)
        rospy.loginfo("total distance: %sm", self.arox_total_dist)
        rospy.loginfo("simulated problems: %s", self.simulated_problems)
        for i in self.mode_times.keys():
            rospy.loginfo("time in '%s' mode: %ss", i, self.mode_times[i])
        rospy.loginfo("###########################################################")

def node():
    rospy.init_node('experiments')
    
    global TF_BUFFER
    TF_BUFFER = tf2_ros.Buffer()
    tf2_ros.TransformListener(TF_BUFFER)

    Experiment()
    rospy.loginfo("launch experiments node..")
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
