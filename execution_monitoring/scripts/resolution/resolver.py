#!/usr/bin/env python
from datetime import datetime

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from arox_navigation_flex.msg import drive_to_goalAction
from arox_performance_parameters.msg import arox_battery_params
from arox_performance_parameters.msg import arox_operational_param
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Float64

from execution_monitoring import config, util


class FallbackResolver:
    """
    The fallback resolver is launched in cases where no other solution is successful / available. The fallback solution
    is to call the human operator who then takes care of the problem. A message on "/request_fallback" indicates that
    the robot is requesting the help of a human operator, i.e., is unable to solve a problem by itself.
    """

    def __init__(self):
        self.problem_resolved = False
        self.human_operator_pub = rospy.Publisher("/request_help", String, queue_size=1)
        self.fallback_pub = rospy.Publisher("/fallback_success", Bool, queue_size=1)
        self.resolution_pub = rospy.Publisher('/resolution', String, queue_size=1)
        rospy.Subscriber('/request_fallback', String, self.request_fallback, queue_size=1)

    def request_fallback(self, msg):
        """
        Callback that communicates the problem to the human operator who should take care of it.

        @param msg: callback message - problem description
        """
        rospy.loginfo("fallback requested.. communicating problem to human operator..")
        rospy.loginfo("problem: %s", msg.data)
        self.resolution_pub.publish("fallback requested -- communicating problem to human operator: " + msg.data)
        self.problem_resolved = True
        self.human_operator_pub.publish(msg.data)
        rospy.loginfo("failure communicated..")
        self.fallback_pub.publish(True)


class GeneralFailureResolver(object):
    """
    Encapsulates certain functionalities common to all resolution classes.
    """

    def __init__(self):
        self.problem_resolved = False
        self.fallback_pub = rospy.Publisher('/request_fallback', String, queue_size=1)
        self.resolution_pub = rospy.Publisher('/resolution', String, queue_size=1)
        self.robot_info_pub = rospy.Publisher('/robot_info', String, queue_size=1)
        rospy.Subscriber("/fallback_success", Bool, self.fallback_callback, queue_size=1)

    def fallback_callback(self, msg):
        """
        Communicates whether the fallback resolver has successfully solved the problem.

        @param msg: callback message - whether the problem is solved
        """
        if msg.data:
            rospy.loginfo("solved by fallback resolver: %s", msg.data)
            self.resolution_pub.publish("solved by fallback resolver: " + str(msg.data))
            self.problem_resolved = True
        else:
            rospy.loginfo("fallback solution was not successful..")
            self.resolution_pub.publish("fallback solution was not successful..")


class WeatherFailureResolver(GeneralFailureResolver):
    """
    Specific resolver class for weather related issues.
    """

    def __init__(self):
        super(WeatherFailureResolver, self).__init__()
        self.success_pub = rospy.Publisher('/resolve_weather_failure_success', Bool, queue_size=1)
        self.insert_goal_pub = rospy.Publisher('introduce_intermediate_shelter_goal', String, queue_size=1)
        rospy.Subscriber('/resolve_weather_failure', String, self.resolve_callback, queue_size=1)

    def resolve_callback(self, msg):
        """
        Initiates weather failure resolution.

        @param msg: callback message - type of weather failure
        """
        rospy.loginfo("launch weather failure resolver..")
        rospy.loginfo("type of weather failure: %s", msg.data)
        rospy.sleep(config.SHORT_DELAY)
        self.resolution_pub.publish("launch weather failure resolver -- type of weather failure: " + msg.data)
        self.problem_resolved = False

        # all weather related failures are resolved by seeking shelter and waiting
        if msg.data in config.WEATHER_FAILURES.values():
            self.resolve_weather_failure(msg.data)
        elif msg.data == config.WEATHER_CATA:
            self.resolve_catastrophe(config.WEATHER_CATA)

        if self.problem_resolved:
            self.success_pub.publish(True)

    def resolve_catastrophe(self, msg):
        """
        Initiates fallback resolver in case of failure.

        @param msg: catastrophe message
        """
        rospy.loginfo("resolve weather change catastrophe -- requesting help from human operator")
        self.resolution_pub.publish("resolve weather change catastrophe -- requesting help from human operator")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(config.RESOLUTION_CHECK_FREQ)

    def resolve_weather_failure(self, msg):
        """
        Resolves weather related failure by seeking shelter and waiting.

        @param msg: callback message
        """
        self.resolution_pub.publish("resolve drastic weather change -- seeking shelter -- driving back to base")
        rospy.loginfo(
            "resolve drastic weather change -- introduce intermediate goals into plan - [return_to_base, charge, wait]")
        self.resolution_pub.publish(
            "resolve drastic weather change -- introduce intermediate goals into plan - [return_to_base, charge, wait]")
        # insert intermediate recharge goals [return_to_base, charge, wait]
        self.insert_goal_pub.publish("")
        self.problem_resolved = True


class DataManagementFailureResolver(GeneralFailureResolver):
    """
    Specific resolver class for data management issues.
    """

    def __init__(self):
        super(DataManagementFailureResolver, self).__init__()
        self.fail_cnt = 0
        self.success_pub = rospy.Publisher('/resolve_data_management_failure_success', Bool, queue_size=1)
        rospy.Subscriber('/resolve_data_management_failure', String, self.resolve_callback, queue_size=1)
        rospy.Subscriber('/scan_successfully_logged', String, self.logging_success_callback, queue_size=1)

    def logging_success_callback(self, msg):
        """
        Callback that informs about successful scan logging - resets fail counter.

        @param msg: callback message
        """
        self.fail_cnt = 0

    def resolve_callback(self, msg):
        """
        Initiates data management failure resolution.

        @param msg: callback message - type of data management failure
        """
        rospy.loginfo("launch data management failure resolver..")
        rospy.sleep(config.SHORT_DELAY)
        self.resolution_pub.publish("launch data management failure resolver -- type of failure: " + msg.data)
        rospy.loginfo("type of data management failure: %s", msg.data)
        self.problem_resolved = False

        # different types of resolution are required based on the type of issue
        if msg.data == config.DATA_MANAGEMENT_FAILURES[0]:
            self.resolve_full_memory_failure(msg.data)
        elif msg.data == config.DATA_MANAGEMENT_FAILURES[1]:
            self.resolve_scan_logging_failure(msg.data)
        elif msg.data == config.DATA_MANAGEMENT_CATA:
            self.resolve_catastrophe(msg.data)

        if self.problem_resolved:
            self.resolution_pub.publish("problem resolved")
            self.success_pub.publish(True)

    def resolve_catastrophe(self, msg):
        """
        Initiates fallback resolver in case of failure.

        @param msg: catastrophe message
        """
        rospy.loginfo("resolve data management catastrophe -- requesting help from human operator")
        self.resolution_pub.publish("resolve data management catastrophe -- requesting help from human operator")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(config.RESOLUTION_CHECK_FREQ)

    def resolve_full_memory_failure(self, msg):
        """
        Prepares resolution of failures based on a full memory.

        @param msg: callback message
        """
        rospy.loginfo("resolve full memory failure.. a full memory can't be dealt with by the robot,"
                      + " but it can drive back to the base in preparation..")
        self.resolution_pub.publish("resolve full memory failure")
        # return to base + launch catastrophe
        # TODO: implement docking for config.DOCKING cases
        action_goal = util.create_nav_goal(config.BASE_POSE, None)
        self.drive_to_goal_client.wait_for_server()
        self.drive_to_goal_client.send_goal(action_goal)
        rospy.loginfo("goal sent, waiting for completion..")
        self.drive_to_goal_client.wait_for_result()
        # initiate catastrophe
        self.success_pub.publish(False)

    def resolve_scan_logging_failure(self, msg):
        """
        Tackles a scan logging failure by simply repeating the scan operation.

        @param msg: callback message
        """
        rospy.loginfo("resolve scan logging failure..")
        if self.fail_cnt < config.REPEAT_SCAN_THRESH:
            rospy.loginfo("just trying again..")
            self.resolution_pub.publish("resolve scan logging failure - trying again..")
            self.problem_resolved = True
        else:
            # initiate catastrophe
            self.success_pub.publish(False)
        self.fail_cnt += 1


class ConnectionResolver(GeneralFailureResolver):
    """
    Specific resolver class for connection issues.
    """

    def __init__(self):
        super(ConnectionResolver, self).__init__()
        self.fail_cnt = 0
        self.fail_cnt_update = None
        self.success_pub = rospy.Publisher('/resolve_connection_failure_success', Bool, queue_size=1)
        self.re_init_pub = rospy.Publisher('/re_init_internet_monitoring', String, queue_size=1)
        rospy.Subscriber('/resolve_wifi_failure', String, self.resolve_callback, queue_size=1)
        rospy.Subscriber('/resolve_internet_failure', String, self.resolve_callback, queue_size=1)
        rospy.Subscriber('/resolve_gnss_failure', String, self.resolve_callback, queue_size=1)

    def resolve_callback(self, msg):
        """
        Initiates connection failure resolution.

        @param msg: callback message - type of connection failure
        """
        rospy.loginfo("launch connection failure resolver..")
        rospy.loginfo("type of connection failure: %s", msg.data)
        rospy.sleep(config.SHORT_DELAY)
        self.resolution_pub.publish("launch connection failure resolver -- type of connection failure: " + msg.data)
        self.problem_resolved = False

        # fail count outdated -> reset
        time_since_fail = (datetime.now() - self.fail_cnt_update).total_seconds() if self.fail_cnt_update else None
        if time_since_fail and time_since_fail > config.FAIL_OUTDATED_THRESH:
            self.fail_cnt = 0

        # wifi failures
        if msg.data in config.WIFI_FAILURES.values():
            self.resolve_wifi_failure(msg.data)
        # internet failures
        elif msg.data in config.INTERNET_FAILURES.values():
            self.resolve_internet_failure(msg.data)
        # gnss failures
        elif msg.data in config.GNSS_FAILURES.values():
            self.resolve_gnss_failure(msg.data)
        elif msg.data == config.CONNECTION_CATA:
            self.resolve_catastrophe(config.CONNECTION_CATA)

        if self.problem_resolved:
            self.resolution_pub.publish("problem resolved")
            self.success_pub.publish(True)

    def resolve_catastrophe(self, msg):
        """
        Initiates fallback resolver in case of failure.

        @param msg: catastrophe message
        """
        rospy.loginfo("resolve connection catastrophe -- requesting help from human operator")
        self.resolution_pub.publish("resolve connection catastrophe -- requesting help from human operator")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(config.RESOLUTION_CHECK_FREQ)

    def resolve_wifi_failure(self, msg):
        """
        Tackles the specified WiFi connection failure.

        @param msg: type of WiFi connection failure
        """
        rospy.loginfo("resolve wifi failure: %s", msg)
        self.resolution_pub.publish("resolve wifi failure: " + str(msg))

        if self.fail_cnt < config.REPEAT_CONNECTION_CHECK_THRESH:
            rospy.loginfo("just trying again..")
            self.resolution_pub.publish("just trying again..")
            self.problem_resolved = True
        else:
            # initiate catastrophe
            self.success_pub.publish(False)
        self.fail_cnt += 1
        self.fail_cnt_update = datetime.now()

    def resolve_internet_failure(self, msg):
        """
        Tackles the specified internet connection failure.

        @param msg: type of internet connection failure
        """
        rospy.loginfo("resolve internet failure: %s", msg)
        self.resolution_pub.publish("resolve internet failure: " + str(msg))

        if self.fail_cnt < config.REPEAT_CONNECTION_CHECK_THRESH:
            # try to re-initialize the internet monitor (requires connection)
            rospy.loginfo("reinitializing internet connection..")
            self.resolution_pub.publish("reinitializing internet connection..")
            self.re_init_pub.publish("")
            self.problem_resolved = True
        else:
            # initiate catastrophe
            self.success_pub.publish(False)
        self.fail_cnt += 1
        self.fail_cnt_update = datetime.now()

    def resolve_gnss_failure(self, msg):
        """
        Tackles the specified GNSS connection failure.

        @param msg: type of GNSS connection failure
        """
        rospy.loginfo("resolve GNSS failure: %s", msg)
        self.resolution_pub.publish("resolve GNSS failure: " + str(msg))

        if self.fail_cnt < config.REPEAT_CONNECTION_CHECK_THRESH:
            rospy.loginfo("just trying again..")
            self.resolution_pub.publish("just trying again..")
            self.problem_resolved = True
        else:
            # initiate catastrophe
            self.success_pub.publish(False)
        self.fail_cnt += 1
        self.fail_cnt_update = datetime.now()


class PowerManagementFailureResolver(GeneralFailureResolver):
    """
    Specific resolver class for power management issues.
    """

    def __init__(self):
        super(PowerManagementFailureResolver, self).__init__()
        self.success_pub = rospy.Publisher('/resolve_power_management_failure_success', Bool, queue_size=1)
        self.insert_goal_pub = rospy.Publisher('introduce_intermediate_recharge_goal', String, queue_size=1)
        self.reset_discharge_rate_pub = rospy.Publisher('/reset_discharge_rate', String, queue_size=1)
        self.cata_launched_pub = rospy.Publisher('/catastrophe_launched', String, queue_size=1)
        rospy.Subscriber('/resolve_power_management_failure', String, self.resolve_callback, queue_size=1)

    def resolve_callback(self, msg):
        """
        Initiates power management failure resolution.

        @param msg: callback message - type of power management failure
        """
        rospy.loginfo("launch power management failure resolver..")
        rospy.loginfo("type of power management failure: %s", msg.data)
        self.reset_discharge_rate_pub.publish(msg)
        rospy.sleep(config.SHORT_DELAY)
        self.resolution_pub.publish("launch power management failure resolver -- type of issue: " + msg.data)
        self.problem_resolved = False

        if msg.data in config.POWER_MANAGEMENT_FAILURES.values():
            self.resolve_contingency(msg.data)
        elif msg.data == config.POWER_MANAGEMENT_CATA:
            self.resolve_catastrophe(config.POWER_MANAGEMENT_CATA)

        if self.problem_resolved:
            self.resolution_pub.publish("problem resolved")
            self.success_pub.publish(True)

    def resolve_contingency(self, msg):
        """
        Resolves power management contingency cases by returning to the base and recharging.

        @param msg: type of power management issue
        """
        rospy.loginfo(
            "resolve power management contingency -- introduce intermediate goals into plan - [return_to_base, charge]")
        self.resolution_pub.publish(
            "resolve power management contingency -- introduce intermediate goals into plan - [return_to_base, charge]")
        # insert intermediate recharge goals [return_to_base, charge]
        self.insert_goal_pub.publish("")
        self.problem_resolved = True

    def resolve_catastrophe(self, msg):
        """
        Initiates fallback resolver in case of failure.

        @param msg: catastrophe message
        """
        rospy.loginfo("resolve power management catastrophe -- requesting help from human operator")
        self.resolution_pub.publish("resolve power management catastrophe -- requesting help from human operator")
        self.fallback_pub.publish(msg)
        self.cata_launched_pub.publish("")
        while not self.problem_resolved:
            rospy.sleep(config.RESOLUTION_CHECK_FREQ)


class SensorFailureResolver(GeneralFailureResolver):
    """
    Specific resolver class for sensor failures.
    """

    def __init__(self):
        super(SensorFailureResolver, self).__init__()
        self.fail_cnt = 0
        self.fail_cnt_update = None
        self.success_pub = rospy.Publisher('/resolve_sensor_failure_success', Bool, queue_size=1)
        rospy.Subscriber('/resolve_sensor_failure', String, self.resolve_callback, queue_size=1)

    def resolve_callback(self, msg):
        """
        Initiates sensor failure resolution.

        @param msg: callback message - type of sensor failure
        """
        rospy.loginfo("launch sensor failure resolver..")
        rospy.loginfo("type of sensor failure: %s", msg.data)
        rospy.sleep(config.SHORT_DELAY)
        self.resolution_pub.publish("launch sensor failure resolver -- type of sensor failure: " + msg.data)
        self.problem_resolved = False

        # reset outdated fail count
        time_since_fail = (datetime.now() - self.fail_cnt_update).total_seconds() if self.fail_cnt_update else None
        if time_since_fail and time_since_fail > config.FAIL_OUTDATED_THRESH:
            self.fail_cnt = 0

        if msg.data in config.SENSOR_FAILURES.values():
            self.resolve_sensor_failure(msg.data)
        elif msg.data == config.SENSOR_CATA:
            self.resolve_catastrophe(config.SENSOR_CATA)

        if self.problem_resolved:
            self.resolution_pub.publish("problem resolved")
            self.success_pub.publish(True)

    def resolve_catastrophe(self, msg):
        """
        Initiates fallback resolver in case of failure.

        @param msg: catastrophe message
        """
        rospy.loginfo("resolve sensor catastrophe -- requesting help from human operator")
        self.resolution_pub.publish("resolve sensor catastrophe -- requesting help from human operator")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(config.RESOLUTION_CHECK_FREQ)

    def resolve_sensor_failure(self, msg):
        """
        Tackles the specified sensor failure.

        @param msg: type of sensor failure
        """
        if self.fail_cnt < config.REPEAT_SCAN_THRESH:
            rospy.loginfo("resolve sensor failure..")
            self.resolution_pub.publish("resolve sensor failure - simply trying again")
            rospy.loginfo("simply trying again..")
            self.problem_resolved = True
        else:
            # initiate catastrophe
            self.success_pub.publish(False)
        self.fail_cnt += 1
        self.fail_cnt_update = datetime.now()


class LocalizationFailureResolver(GeneralFailureResolver):
    """
    Specific resolver class for localization failures.
    """

    def __init__(self):
        super(LocalizationFailureResolver, self).__init__()
        self.success_pub = rospy.Publisher('/resolve_localization_failure_success', Bool, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber('/resolve_localization_failure', String, self.resolve_callback, queue_size=1)

    def resolve_callback(self, msg):
        """
        Initiates localization failure resolution.

        @param msg: callback message - type of localization failure
        """
        rospy.loginfo("launch localization failure resolver..")
        rospy.loginfo("type of localization failure: %s", msg.data)
        self.resolution_pub.publish("launch localization failure resolver -- type of localization failure: " + msg.data)
        self.problem_resolved = False

        if msg.data in config.LOCALIZATION_FAILURES.values():
            self.resolve_localization_failure()

        if self.problem_resolved:
            self.resolution_pub.publish("problem resolved")
            self.success_pub.publish(True)

    def resolve_localization_failure(self):
        """
        Attempts to resolve the localization failure at hand.
        """
        rospy.loginfo("resolve localization failure..")
        # sleeping a moment to wait for the robot to stand still
        rospy.sleep(config.RESOLUTION_CHECK_FREQ)
        rospy.loginfo("driving the robot a few meters back and forth to recalibrate the localization"
                      + " using different GNSS positions..")
        self.resolution_pub.publish("driving the robot a few meters back and forth to recalibrate the localization"
                                    + " using different GNSS positions")

        # TODO: should pay attention to obstacles etc.
        twist = Twist()
        twist.linear.x = -3.0
        rate = rospy.Rate(4)
        for _ in range(2):
            for _ in range(4):
                self.cmd_vel_pub.publish(twist)
                rate.sleep()
            twist.linear.x = 3.0

        rospy.loginfo("clearing costmaps..")
        self.robot_info_pub.publish("clearing costmaps..")
        util.clear_costmaps()
        self.problem_resolved = True


class PlanDeploymentFailureResolver(GeneralFailureResolver):
    """
    Specific resolver class for plan deployment failures.
    """

    def __init__(self):
        super(PlanDeploymentFailureResolver, self).__init__()
        self.infeasible_plan_cnt = 0
        self.empty_plan_cnt = 0
        self.unavailable_service_cnt = 0
        self.success_pub = rospy.Publisher('/resolve_plan_deployment_failure_success', Bool, queue_size=1)
        self.activate_plan_service_pub = rospy.Publisher('/activate_plan_service', String, queue_size=1)
        rospy.Subscriber('/resolve_plan_deployment_failure', String, self.resolve_callback, queue_size=1)
        rospy.Subscriber('arox/ongoing_operation', arox_operational_param, self.operation_callback, queue_size=1)

    def operation_callback(self, msg):
        """
        Callback receiving operational parameter updates - used to reset failure counters.

        @param msg: callback message
        """
        # the repetition of the plan retrieval attempt was obviously successful
        self.infeasible_plan_cnt = self.empty_plan_cnt = 0

    def resolve_callback(self, msg):
        """
        Initiates plan deployment failure resolution.

        @param msg: callback message - type of plan deployment failure
        """
        rospy.loginfo("launch plan deployment failure resolver..")
        rospy.loginfo("type of plan deployment failure: %s", msg.data)
        rospy.sleep(config.SHORT_DELAY)
        self.resolution_pub.publish("launch plan deployment failure resolver -- type of failure: " + msg.data)
        self.problem_resolved = False

        # different types of resolution are required based on the type of issue
        if msg.data == config.PLAN_DEPLOYMENT_FAILURES[1]:
            self.resolve_unavailable_service_failure(msg.data)
        elif msg.data in [config.PLAN_DEPLOYMENT_FAILURES[2], config.PLAN_DEPLOYMENT_FAILURES[3],
                          config.PLAN_DEPLOYMENT_FAILURES[4]]:
            self.resolve_invalid_plan_failure(msg.data)
        elif msg.data == config.PLAN_DEPLOYMENT_CATA:
            self.resolve_catastrophe(msg.data)

        if self.problem_resolved:
            self.resolution_pub.publish("problem resolved")
            self.success_pub.publish(True)

    def resolve_catastrophe(self, msg):
        """
        Initiates fallback resolver in case of failure.

        @param msg: catastrophe message
        """
        rospy.loginfo("resolve plan deployment catastrophe -- requesting help from human operator")
        self.resolution_pub.publish("resolve plan deployment catastrophe -- requesting help from human operator")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_unavailable_service_failure(self, msg):
        """
        Attempts to resolve failures related to an unavailable plan service by reinitializing the plan provision node.

        @param msg: failure message
        """
        rospy.loginfo("resolve unavailable plan failure..")
        self.resolution_pub.publish("resolve unavailable plan failure by trying to bring up the service")
        if self.unavailable_service_cnt < config.UNAVAIL_PLAN_SERVICE_THRESH:
            rospy.loginfo("resolve by trying to bring up the service..")
            self.activate_plan_service_pub.publish("bring up service")
            self.unavailable_service_cnt += 1
            self.problem_resolved = True
        else:
            rospy.loginfo("plan generation service repeatedly unavailable..")
            self.resolution_pub.publish("plan generation service repeatedly unavailable")
            # initiate catastrophe
            self.success_pub.publish(False)

    def resolve_invalid_plan_failure(self, msg):
        """
        Attempts to resolve failures related to an invalid plan by repeating the plan request.

        @param msg: failure message
        """
        rospy.loginfo("resolve invalid plan failure..")
        self.resolution_pub.publish("resolve invalid plan failure -- resolve by trying again")
        if self.empty_plan_cnt < config.INVALID_PLAN_THRESH:
            rospy.loginfo("resolve by trying again..")
            self.empty_plan_cnt += 1
            self.problem_resolved = True
        else:
            rospy.loginfo("plan retrieval failed repeatedly..")
            self.resolution_pub.publish("plan retrieval failed repeatedly")
            # initiate catastrophe
            self.success_pub.publish(False)


class NavigationFailureResolver(GeneralFailureResolver):
    """
    Specific resolver class for navigation failures.
    """

    def __init__(self):
        super(NavigationFailureResolver, self).__init__()
        self.success_pub = rospy.Publisher('/resolve_navigation_failure_success', Bool, queue_size=1)
        self.remove_obstacles_pub = rospy.Publisher('/clear_spawned_obstacles', String, queue_size=1)
        self.drive_to_goal_client = actionlib.SimpleActionClient('drive_to_goal', drive_to_goalAction)
        rospy.Subscriber('/resolve_navigation_failure', String, self.resolve_callback, queue_size=1)
        rospy.Subscriber('/resolution_failure', String, self.resolution_failure_callback, queue_size=1)

    def resolution_failure_callback(self, msg):
        """
        Callback that handles situations of failed resolution attempts.

        @param msg: callback message
        """
        rospy.loginfo("nav fail resolution failed.. cancelling resolution attempt..")
        self.resolution_pub.publish("nav fail resolution failed.. cancelling resolution attempt..")
        self.drive_to_goal_client.cancel_all_goals()
        # initiate catastrophe
        self.success_pub.publish(False)
        # solved -- obstacles removed
        self.remove_obstacles_pub.publish("")
        # wait for obstacle removal before costmap clearance
        rospy.sleep(3)
        self.robot_info_pub.publish("clearing costmaps..")
        util.clear_costmaps()

    def resolve_callback(self, msg):
        """
        Initiates navigation failure resolution.

        @param msg: callback message - type of navigation failure
        """
        rospy.loginfo("launch navigation failure resolver..")
        rospy.loginfo("type of navigation failure: %s", msg.data)
        rospy.sleep(config.SHORT_DELAY)
        self.resolution_pub.publish("launch navigation failure resolver -- type of navigation failure: " + msg.data)
        self.problem_resolved = False

        if msg.data in config.NAVIGATION_FAILURES.values():
            self.resolve_nav_failure(msg.data)
        elif msg.data == config.NAV_CATA:
            self.resolve_catastrophe(config.NAV_CATA)
        else:
            rospy.loginfo("cannot resolve unknown nav failure: %s", msg.data)
            self.resolution_pub.publish("cannot resolve unknown nav failure: " + msg.data)

        if self.problem_resolved:
            self.resolution_pub.publish("problem resolved")
            self.success_pub.publish(True)

    def resolve_catastrophe(self, msg):
        """
        Initiates fallback resolver in case of failure.

        @param msg: catastrophe message
        """
        rospy.loginfo("resolve nav catastrophe -- requesting help from human operator")
        self.resolution_pub.publish("resolve nav catastrophe -- requesting help from human operator")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(config.RESOLUTION_CHECK_FREQ)

    def resolve_nav_failure(self, msg):
        """
        Attempts to resolve navigation failures.

        @param msg: type of navigation failure
        """
        rospy.loginfo("resolve navigation failure.. driving to recovery point..")
        self.resolution_pub.publish("resolve navigation failure -- driving to recovery point")
        self.robot_info_pub.publish("clearing costmaps..")
        util.clear_costmaps()

        action_goal = util.create_nav_goal(config.RECOVERY_POINT_ONE, None)
        self.drive_to_goal_client.wait_for_server()
        self.drive_to_goal_client.send_goal(action_goal)
        rospy.loginfo("goal sent, waiting for completion..")
        self.drive_to_goal_client.wait_for_result()
        self.drive_to_goal_client.get_result()

        # navigation failure during resolution -- goal not reached
        if self.drive_to_goal_client.get_state() == GoalStatus.ABORTED:
            rospy.loginfo("nav failure during resolution -- notifying operator..")
            self.resolution_pub.publish("nav failure during resolution -- notifying operator")
            # initiate catastrophe
            self.success_pub.publish(False)
            # solved -- obstacles removed
            self.remove_obstacles_pub.publish("")
            # wait for obstacle removal before costmap clearance
            rospy.sleep(3)
            self.robot_info_pub.publish("clearing costmaps..")
            util.clear_costmaps()
        elif self.drive_to_goal_client.get_state() == GoalStatus.SUCCEEDED:
            self.problem_resolved = True


class ChargingFailureResolver(GeneralFailureResolver):
    """
    Specific resolver class for charging failures.
    """

    def __init__(self):
        super(ChargingFailureResolver, self).__init__()
        self.charge_level_at_resolution = 0.0
        self.latest_charge_level = 0.0
        self.docking_fail_cnt = 0
        self.undocking_fail_cnt = 0
        self.charge_fail_cnt = 0
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.success_pub = rospy.Publisher('/resolve_charging_failure_success', Bool, queue_size=1)
        self.insert_goal_pub = rospy.Publisher('introduce_intermediate_nav_goal', String, queue_size=1)
        rospy.Subscriber('/resolve_charging_failure', String, self.resolve_callback, queue_size=1)
        rospy.Subscriber('/arox/battery_param', arox_battery_params, self.battery_callback, queue_size=1)
        rospy.Subscriber('/move_base_flex/exe_path/status', GoalStatusArray, self.nav_status_callback, queue_size=1)

    def nav_status_callback(self, nav_status):
        """
        Callback that provides information about the status of navigation.

        @param nav_status: navigation status
        """
        if len(nav_status.status_list) > 0 and nav_status.status_list[-1].status == GoalStatus.SUCCEEDED:
            # undocking was obviously successful - reset fail cnt
            self.undocking_fail_cnt = 0

    def battery_callback(self, msg):
        """
        Callback that provides battery parameter information.

        @param msg: battery parameters
        """
        self.latest_charge_level = msg.charge
        if self.latest_charge_level > self.charge_level_at_resolution:
            # resolution successful - reset fail cnt
            self.docking_fail_cnt = 0
            self.charge_fail_cnt = 0

    def resolve_callback(self, msg):
        """
        Initiates charging failure resolution.

        @param msg: callback message - type of charging failure
        """
        rospy.loginfo("launch charging failure resolver..")
        rospy.loginfo("type of charging failure: %s", msg.data)
        rospy.sleep(config.SHORT_DELAY)
        self.resolution_pub.publish("launch charging failure resolver -- type of failure: " + msg.data)
        self.problem_resolved = False

        if msg.data == config.CHARGING_FAILURES[0]:
            self.resolve_docking_failure()
        elif msg.data == config.CHARGING_FAILURES[1]:
            self.resolve_undocking_failure()
        elif msg.data == config.CHARGING_FAILURES[2]:
            self.resolve_charging_failure()
        elif msg.data == config.CHARGING_CATA:
            self.resolve_catastrophe(config.CHARGING_CATA)
        else:
            rospy.loginfo("cannot resolve unknown nav failure: %s", msg.data)

        if self.problem_resolved:
            self.resolution_pub.publish("problem resolved")
            self.success_pub.publish(True)

    def resolve_catastrophe(self, msg):
        """
        Initiates fallback resolver in case of failure.

        @param msg: catastrophe message
        """
        rospy.loginfo("resolve charging catastrophe -- requesting help from human operator")
        self.resolution_pub.publish("resolve charging catastrophe -- requesting help from human operator")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(config.RESOLUTION_CHECK_FREQ)

    @staticmethod
    def open_container():
        """
        Sends a command to open the container front (lower the ramp).
        """
        rospy.loginfo("sending command to open container front..")
        container_pub = rospy.Publisher('/container/rampB_position_controller/command', Float64, queue_size=1)
        for _ in range(3):
            container_pub.publish(2.0)
            rospy.sleep(0.5)

    def resolve_docking_failure(self):
        """
        Attempts to resolve docking failures.
        """
        rospy.loginfo("resolving docking failure..")
        self.resolution_pub.publish("resolving docking failure")
        self.charge_level_at_resolution = self.latest_charge_level
        # already tried resolution before - call human
        if self.docking_fail_cnt == config.DOCKING_FAIL_THRESH:
            rospy.loginfo("already tried autonomous resolution before -- calling human operator for help..")
            self.resolution_pub.publish("already tried autonomous resolution before -- calling human operator for help")
            self.success_pub.publish(False)  # initiate catastrophe
            self.open_container()  # human would have opened the container -- in case it was closed
            rospy.sleep(5)  # clear costmap to perceive that the door is open now
            self.robot_info_pub.publish("clearing costmaps..")
            util.clear_costmaps()
        else:
            rospy.loginfo("just trying again..")
            self.resolution_pub.publish("just trying again")
            self.problem_resolved = True
            self.docking_fail_cnt += 1

    def perform_minor_relocation(self):
        """
        Performs a minor relocation that often alleviates the problem.
        """
        self.resolution_pub.publish("driving robot back and forth -- minor relocation")
        twist = Twist()
        twist.linear.x = 3.0
        rate = rospy.Rate(2)
        for _ in range(2):
            for _ in range(2):
                self.cmd_vel_pub.publish(twist)
                rate.sleep()
            twist.linear.x = -3.0

    def resolve_undocking_failure(self):
        """
        Attempts to resolve undocking failures.
        """
        # already tried resolution before - call human
        if self.undocking_fail_cnt == config.UNDOCKING_FAIL_THRESH:
            rospy.loginfo("already tried autonomous resolution before -- calling human operator for help..")
            self.resolution_pub.publish("already tried autonomous resolution before -- calling human operator for help")
            self.success_pub.publish(False)  # initiate catastrophe
            # human would have opened the container -- in case it was closed
            rospy.loginfo("sending command to open container front..")
            self.open_container()
            # clear costmap to perceive that the door is open now
            rospy.sleep(5)
            self.robot_info_pub.publish("clearing costmaps..")
            util.clear_costmaps()
        else:
            self.perform_minor_relocation()
            self.problem_resolved = True
            self.undocking_fail_cnt += 1

    def resolve_charging_failure(self):
        """
        Attempts to resolve the charging failure.
        """
        rospy.loginfo("resolving charging failure..")
        self.resolution_pub.publish("resolving charging failure")
        self.charge_level_at_resolution = self.latest_charge_level

        if self.charge_fail_cnt == 1:
            rospy.loginfo("already tried autonomous resolution before -- calling human operator for help..")
            self.resolution_pub.publish("already tried autonomous resolution before -- calling human operator for help")
            # initiate catastrophe
            self.success_pub.publish(False)
        else:
            self.perform_minor_relocation()
            self.problem_resolved = True
            self.charge_fail_cnt += 1


def node():
    """
    Resolution node that provides solving functionalities for numerous LTA related challenges.
    """
    rospy.init_node('failure_resolver')
    rospy.wait_for_message('SMACH_running', String)
    SensorFailureResolver()
    ConnectionResolver()
    WeatherFailureResolver()
    DataManagementFailureResolver()
    FallbackResolver()
    LocalizationFailureResolver()
    PlanDeploymentFailureResolver()
    NavigationFailureResolver()
    ChargingFailureResolver()
    PowerManagementFailureResolver()
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
