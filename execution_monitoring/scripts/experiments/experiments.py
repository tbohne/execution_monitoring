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

CATA_TOPIC_MSG_MAPPING = {
    # prerequisite: prepare full USB flash drive + mount it under config.FULL_DRIVE, e.g., "/mnt/usb"
    "/sim_full_disk_failure": config.DATA_MANAGEMENT_FAILURES[0],
    "/spawn_robot_prison": [config.NAVIGATION_FAILURES[0], config.NAVIGATION_FAILURES[2]],
    "/sim_docking_failure_raised_ramp": config.CHARGING_FAILURES[0],
    "/sim_undocking_failure": config.CHARGING_FAILURES[1],
    "/sim_power_management_catastrophe": config.POWER_MANAGEMENT_CATA
}

CONT_TOPIC_MSG_MAPPING = {
    "/toggle_simulated_total_sensor_failure": config.SENSOR_FAILURES[0],
    "/toggle_simulated_empty_ranges": config.SENSOR_FAILURES[1],
    "/toggle_simulated_impermissible_ranges": config.SENSOR_FAILURES[2],
    "/toggle_simulated_scan_repetition": config.SENSOR_FAILURES[3],
    "/toggle_simulated_bad_wifi_link": config.WIFI_FAILURES[0],
    "/toggle_simulated_bad_wifi_signal": config.WIFI_FAILURES[1],
    "/toggle_simulated_bad_wifi_bit_rate": config.WIFI_FAILURES[2],
    "/toggle_simulated_wifi_disconnect": config.WIFI_FAILURES[3],
    "/sim_internet_connection_failure": config.INTERNET_FAILURES[0],
    "/toggle_simulated_bad_download": config.INTERNET_FAILURES[1],
    "/toggle_simulated_bad_upload": config.INTERNET_FAILURES[2],
    "/toggle_simulated_timeout_failure": config.GNSS_FAILURES[0],
    "/set_simulated_unknown_status": config.GNSS_FAILURES[1],
    "/set_simulated_no_fix": config.GNSS_FAILURES[2],
    "/set_simulated_no_rtk": [],
    "/toggle_simulated_infeasible_lat_lng": [config.GNSS_FAILURES[5], config.GNSS_FAILURES[6]],
    "/toggle_simulated_variance_history_failure": config.GNSS_FAILURES[10],
    "/toggle_simulated_high_deviation": config.GNSS_FAILURES[8],
    "/toggle_rain_sim": config.WEATHER_FAILURES[1],
    "/toggle_snow_sim": config.WEATHER_FAILURES[3],
    "/toggle_wind_sim": config.WEATHER_FAILURES[6],
    "/toggle_low_temp_sim": config.WEATHER_FAILURES[10],
    "/toggle_thunderstorm_sim": config.WEATHER_FAILURES[11],
    "/toggle_sunset_sim": config.WEATHER_FAILURES[15],
    "/toggle_simulated_scan_logging_failure": config.DATA_MANAGEMENT_FAILURES[1],
    "/wheel_movement_without_pos_change": [config.LOCALIZATION_FAILURES[0], config.LOCALIZATION_FAILURES[1]],
    "/pos_change_without_wheel_movement": [config.LOCALIZATION_FAILURES[0], config.LOCALIZATION_FAILURES[1]],
    "/yaw_divergence": [config.LOCALIZATION_FAILURES[3], config.LOCALIZATION_FAILURES[4]],
    "/moving_although_standing_still_imu": [config.LOCALIZATION_FAILURES[6], config.LOCALIZATION_FAILURES[7]],
    "/moving_although_standing_still_odom": [config.LOCALIZATION_FAILURES[0], config.LOCALIZATION_FAILURES[1]],
    "/sim_extended_idle_time": config.PLAN_DEPLOYMENT_FAILURES[0],
    "/toggle_unavailable_plan_service": config.PLAN_DEPLOYMENT_FAILURES[1],
    "/sim_empty_plan": config.PLAN_DEPLOYMENT_FAILURES[2],
    "/sim_infeasible_plan": config.PLAN_DEPLOYMENT_FAILURES[3],
    "/spawn_static_obstacles": [],
    "/toggle_simulated_unknown_service": [],
    "/trigger_nav_fail": config.NAVIGATION_FAILURES[0],
    "/sim_docking_failure_base_pose": config.CHARGING_FAILURES[0],
    "/sim_charging_failure": config.CHARGING_FAILURES[2],
    "/sim_power_management_contingency": config.POWER_MANAGEMENT_FAILURES[0]
}

RANDOM_FAIL_FREQUENCY = 250  # random fail every 250s (lower bound + depends on RTF)
SEED = 42  # for random failure selection
EXPERIMENT_DURATION = 18000  # 5 hours
IDX = 0  # to identify the experiment run (e.g. in a set of runs)
PLAN_LENGTH = 34  # length of the plan that defines one mission (number of tasks)
SIM_FAILURES = True  # whether there should be random failure simulation every n seconds
TF_BUFFER = None  # transformation buffer


class Experiment:

    def __init__(self):
        global SEED
        random.seed(SEED)
        self.correct_contingency_cnt = 0
        self.correct_no_contingency_cnt = 0
        self.false_positive_cnt = 0
        self.false_negative_cnt = 0
        self.unexpected_contingency_cnt = 0
        self.catastrophe_cnt = 0
        self.simulated_problems = 0

        self.expected_contingency = None
        self.sim_launched = True

        self.completed_tasks = 0
        self.operation_time = 0
        self.battery_charge = 0
        self.battery_charging_cycle = 1
        self.mission_cycle = 0
        self.sim_fail_time = datetime.now()
        self.mode_times = {'traversing': 0, 'scanning': 0, 'waiting': 0, 'catastrophe': 0, 'contingency': 0,
                           'charging': 0, 'docking': 0, 'undocking': 0}
        self.operation_mode = None
        self.prev_mode = None
        self.prev_start = None
        self.prev_pose = None
        self.arox_total_dist = 0

        rospy.Subscriber("/contingency_preemption", String, self.contingency_callback)
        rospy.Subscriber("/catastrophe_preemption", String, self.catastrophe_callback)
        rospy.Subscriber("/arox/ongoing_operation", arox_operational_param, self.operation_callback)
        rospy.Subscriber("/arox/battery_param", arox_battery_params, self.battery_callback)
        rospy.Subscriber('/sim_info', String, self.sim_info_callback, queue_size=1)
        rospy.Subscriber('/odometry/filtered_odom', Odometry, self.filtered_odom_callback, queue_size=1)
        rospy.Subscriber('/robot_info', String, self.robot_info_callback, queue_size=1)

        self.run_experiment()

    def robot_info_callback(self, msg):
        """
        Called whenever new robot status updates arrive.
        Used to recognize SHUTDOWN situations.

        @param msg: robot info message
        """
        if msg.data == "catastrophe processed, shutting down":
            self.catastrophe_cnt += 1

    def filtered_odom_callback(self, msg):
        """
        Called whenever new odometry data arrives. Used to track the total distance traveled by the AROX.

        @param msg: odometry data
        """
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
        """
        Called when failure situation is simulated.

        @param msg: failure simulation message
        """
        rospy.loginfo("failure simulation launched..")
        self.simulated_problems += 1
        self.sim_launched = True
        # reset timer -- sim only launched now -- next sim shouldn't follow immediately, e.g., docking
        self.sim_fail_time = datetime.now()

    def operation_callback(self, msg):
        """
        Called when new information about operational parameters arrive.
        Used to track the robot's mode times, i.e., the time it spends in each mode, and the mission cycles.

        @param msg: operational parameters
        """
        self.operation_mode = msg.operation_mode

        if self.operation_mode != self.prev_mode:
            if self.prev_mode:
                self.mode_times[self.prev_mode] += (datetime.now() - self.prev_start).total_seconds()
            if self.operation_mode not in self.mode_times.keys():
                self.mode_times[self.operation_mode] = 0
            self.prev_mode = self.operation_mode
            self.prev_start = datetime.now()
        self.completed_tasks = msg.rewards_gained

        # new mission
        if msg.total_tasks == PLAN_LENGTH:
            self.mission_cycle += 1

    def battery_callback(self, msg):
        """
        Called whenever new information about the robot's battery parameters arrive.

        @param msg: battery parameters
        """
        self.battery_charge = msg.charge
        self.battery_charging_cycle = msg.charging_cycle
        self.operation_time = msg.operation_time

    def contingency_callback(self, msg):
        """
        Called in contingency situations.

        @param msg: reason for contingency
        """
        rospy.loginfo("------------------------------------------------")
        rospy.loginfo("CONTINGENCY: %s", msg.data)
        rospy.loginfo("EXPECTED: %s", self.expected_contingency)
        rospy.loginfo("------------------------------------------------")

        # expected contingency -- answer to simulated failure
        if isinstance(self.expected_contingency, list):
            if msg.data in self.expected_contingency:
                self.expected_contingency = None
                self.correct_contingency_cnt += 1
            # expected a contingency, but not the present one
            else:
                # keep the expected one - could still come
                self.unexpected_contingency_cnt += 1
        elif self.expected_contingency is not None:
            if msg.data == self.expected_contingency:
                self.expected_contingency = None
                self.correct_contingency_cnt += 1
            # expected a contingency, but not the present one
            else:
                # keep the expected one - could still come
                self.unexpected_contingency_cnt += 1
        elif self.expected_contingency is None:
            # false positive -- contingency, although no failure sim
            self.false_positive_cnt += 1

        self.log_info()

    def catastrophe_callback(self, msg):
        """
        Called in catastrophe situations.

        @param msg: reason for contingency
        @return:
        """
        self.catastrophe_cnt += 1

    def simulate_random_failure(self):
        """
        Initiates the simulation of a random LTA failure.
        """
        global CONT_TOPIC_MSG_MAPPING
        # shouldn't simulate any new failures during docking or when the last sim has not even started
        if self.operation_mode == "docking" or not self.sim_launched:
            rospy.loginfo("not initiating new random failure simulation (docking / previous sim not completed)")
            return

        rand = random.randint(0, len(CONT_TOPIC_MSG_MAPPING.keys()) - 1)
        random_topic = CONT_TOPIC_MSG_MAPPING.keys()[rand]

        # doesn't make any sense to simulate IDLE time during active mission
        if random_topic in ["/sim_extended_idle_time", "/toggle_unavailable_plan_service", "/sim_empty_plan",
                            "/sim_infeasible_plan"] and self.operation_mode != "waiting":
            rospy.loginfo("topic: %s currently not feasible -- selecting another one..", random_topic)
            self.simulate_random_failure()
        else:
            self.sim_fail_time = datetime.now()
            self.sim_launched = False
            rospy.loginfo("###################################################################")
            rospy.loginfo("init random failure: %s", random_topic)

            if not self.expected_contingency:
                self.correct_no_contingency_cnt += 1
                self.expected_contingency = None

            # previously expected contingency was not detected -> false negative
            if self.expected_contingency is not None:
                self.false_negative_cnt += 1

            self.expected_contingency = CONT_TOPIC_MSG_MAPPING[random_topic]
            rospy.loginfo("expected contingency: %s", self.expected_contingency)
            rospy.loginfo("###################################################################")
            pub = rospy.Publisher(random_topic, String, queue_size=1)
            # takes a moment to init the publisher
            rospy.sleep(3)
            pub.publish("sim fail")

    def run_experiment(self):
        """
        Runs an LTA experiment.
        """
        global SIM_FAILURES, EXPERIMENT_DURATION, RANDOM_FAIL_FREQUENCY
        start_time = datetime.now()

        while not rospy.is_shutdown() and self.catastrophe_cnt == 0 and (
                datetime.now() - start_time).total_seconds() < EXPERIMENT_DURATION:

            cooldown_satisfied = (datetime.now() - self.sim_fail_time).total_seconds() >= RANDOM_FAIL_FREQUENCY
            # only initiate new failure simulation after a certain time has passed + the previous one is completed
            if SIM_FAILURES and cooldown_satisfied and self.sim_launched:
                self.simulate_random_failure()

            if SIM_FAILURES and config.VERBOSE_LOGGING:
                rospy.loginfo("time since last fail sim: %s", (datetime.now() - self.sim_fail_time).total_seconds())
            rospy.sleep(config.EXPERIMENTS_CHECK_FREQ)

        self.save_results((datetime.now() - start_time).total_seconds() / 60 / 60)

    def save_results(self, duration):
        """
        Saves the results of the LTA experiment.

        @param duration: experiment duration
        """
        global RANDOM_FAIL_FREQUENCY, SEED, IDX, EXPERIMENT_DURATION
        name = str(RANDOM_FAIL_FREQUENCY) + "_" + str(SEED) + "_" + str(IDX)  # notation (frequency_seed_idx)
        completed = duration >= (EXPERIMENT_DURATION / 60 / 60) and self.catastrophe_cnt == 0
        try:
            with open(config.EXP_PATH + "results.csv", 'a') as out_file:
                if IDX == 0:  # first entry --> write CSV header
                    out_file.write("experiment,duration,correct_contingencies,false_positives,false_negatives,"
                                   + "correct_no_contingency,unexpected_contingencies,completed,completed_tasks,"
                                   + "charge_cycles,mission_cycles,total_dist,simulated_problems,traverse_time,"
                                   + "scan_time,wait_time,cata_time,cont_time,charge_time,dock_time,undock_time\n")

                out_file.write(name + "," + str(duration) + "," + str(self.correct_contingency_cnt) + ","
                               + str(self.false_positive_cnt) + "," + str(self.false_negative_cnt) + ","
                               + str(self.correct_no_contingency_cnt) + "," + str(self.unexpected_contingency_cnt) + ","
                               + str(completed) + "," + str(self.completed_tasks) + ","
                               + str(self.battery_charging_cycle + 1) + "," + str(self.mission_cycle) + ","
                               + str(self.arox_total_dist) + "," + str(self.simulated_problems) + ","
                               + str(self.mode_times['traversing']) + "," + str(self.mode_times['scanning']) + ","
                               + str(self.mode_times['waiting']) + "," + str(self.mode_times['catastrophe']) + ","
                               + str(self.mode_times['contingency']) + "," + str(self.mode_times['charging']) + ","
                               + str(self.mode_times['docking']) + "," + str(self.mode_times['undocking']) + "\n")
        except Exception as e:
            rospy.loginfo("EXCEPTION during storage of experiment results: %s", e)

    def log_info(self):
        """
        Logs information about the current state of the LTA experiment to the console.
        """
        rospy.loginfo("###########################################################")
        rospy.loginfo("corr. contingency cnt: %s", self.correct_contingency_cnt)
        rospy.loginfo("false positives: %s", self.false_positive_cnt)
        rospy.loginfo("false negatives: %s", self.false_negative_cnt)
        rospy.loginfo("corr. NO-contingency cnt: %s", self.correct_no_contingency_cnt)
        rospy.loginfo("unexpected contingency cnt: %s", self.unexpected_contingency_cnt)
        rospy.loginfo("simulated problems: %s", self.simulated_problems)
        rospy.loginfo("catastrophe cnt: %s", self.catastrophe_cnt)
        rospy.loginfo("operation mode: %s", self.operation_mode)
        rospy.loginfo("operation time: %s", self.operation_time)
        rospy.loginfo("completed tasks: %s", self.completed_tasks)
        rospy.loginfo("battery charge: %s, cycle: %s", self.battery_charge, self.battery_charging_cycle + 1)
        rospy.loginfo("mission cycle: %s", self.mission_cycle)
        rospy.loginfo("total distance: %sm", self.arox_total_dist)

        for i in self.mode_times.keys():
            rospy.loginfo("time in '%s' mode: %ss", i, self.mode_times[i])
        rospy.loginfo("###########################################################")


def node():
    """
    LTA experiments node.
    """
    global TF_BUFFER
    rospy.init_node('experiments')
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
