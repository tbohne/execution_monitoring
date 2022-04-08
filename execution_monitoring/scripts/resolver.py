#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Bool, Float64
import actionlib
from datetime import datetime
from arox_navigation_flex.msg import drive_to_goalAction
from execution_monitoring import config, util
from geometry_msgs.msg import Twist
from arox_performance_parameters.msg import arox_operational_param
from mbf_msgs.msg import RecoveryAction, RecoveryGoal
from std_srvs.srv import Empty
from arox_performance_parameters.msg import arox_battery_params
from actionlib_msgs.msg import GoalStatusArray, GoalStatus


class FallbackResolver:
    """
    The fallback resolver works in cases where no other solution is successful / available.
    The fallback solution is to call the human operator who then takes care of the problem.
    """

    def __init__(self):
        rospy.Subscriber('/request_fallback', String, self.request_fallback, queue_size=1)
        self.human_operator_pub = rospy.Publisher("/request_help", String, queue_size=1)
        self.fallback_pub = rospy.Publisher("/fallback_success", Bool, queue_size=1)
        self.resolution_pub = rospy.Publisher('/resolution', String, queue_size=1)
        self.problem_resolved = False

    def request_fallback(self, msg):
        rospy.loginfo("fallback requested.. communicating problem to human operator..")
        rospy.loginfo("problem: %s", msg.data)
        self.resolution_pub.publish("fallback requested -- communicating problem to human operator -- problem: " + msg.data)
        self.problem_resolved = True
        self.human_operator_pub.publish(msg.data)
        rospy.loginfo("failure resolved..")
        self.fallback_pub.publish(True)


class GeneralFailureResolver(object):

    def __init__(self):
        rospy.Subscriber("/fallback_success", Bool, self.fallback_callback, queue_size=1)
        self.fallback_pub = rospy.Publisher('/request_fallback', String, queue_size=1)
        self.resolution_pub = rospy.Publisher('/resolution', String, queue_size=1)
        self.robot_info_pub = rospy.Publisher('/robot_info', String, queue_size=1)
        self.problem_resolved = False

    def fallback_callback(self, msg):
        if msg.data:
            rospy.loginfo("solved by fallback resolver: %s", msg.data)
            self.resolution_pub.publish("solved by fallback resolver: " + str(msg.data))
            self.problem_resolved = True
        else:
            rospy.loginfo("fallback solution was not successful..")
            self.resolution_pub.publish("fallback solution was not successful..")


class WeatherFailureResolver(GeneralFailureResolver):

    def __init__(self):
        super(WeatherFailureResolver, self).__init__()
        rospy.Subscriber('/resolve_weather_failure', String, self.resolve_callback, queue_size=1)
        self.success_pub = rospy.Publisher('/resolve_weather_failure_success', Bool, queue_size=1)
        self.insert_goal_pub = rospy.Publisher('introduce_intermediate_shelter_goal', String, queue_size=1)

    def resolve_callback(self, msg):
        rospy.loginfo("launch weather failure resolver..")
        rospy.loginfo("type of weather failure: %s", msg.data)
        rospy.sleep(2)
        self.resolution_pub.publish("launch weather failure resolver -- type of weather failure: " + msg.data)
        self.problem_resolved = False

        # all these failures are resolved by seeking shelter and waiting
        if msg.data in config.WEATHER_FAILURES.values():
            self.resolve_weather_failure(msg.data)

        elif msg.data == config.WEATHER_CATA:
            self.resolve_catastrophe(config.WEATHER_CATA)

        if self.problem_resolved:
            self.success_pub.publish(True)

    def resolve_catastrophe(self, msg):
        rospy.loginfo("resolve weather change catastrophe -- requesting help from human operator")
        self.resolution_pub.publish("resolve weather change catastrophe -- requesting help from human operator")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_weather_failure(self, msg):
        self.resolution_pub.publish("resolve drastic weather change -- seeking shelter -- driving back to base")
        rospy.loginfo("resolve drastic weather change -- introduce intermediate goals in plan - [return_to_base, charge, wait]")
        self.resolution_pub.publish("resolve drastic weather change -- introduce intermediate goals in plan - [return_to_base, charge, wait]")
        # insert intermediate recharge goals [return_to_base, charge, wait]
        self.insert_goal_pub.publish("")
        self.problem_resolved = True


class DataManagementFailureResolver(GeneralFailureResolver):

    def __init__(self):
        super(DataManagementFailureResolver, self).__init__()
        self.fail_cnt = 0
        rospy.Subscriber('/resolve_data_management_failure', String, self.resolve_callback, queue_size=1)
        rospy.Subscriber('/scan_successfully_logged', String, self.logging_success_callback, queue_size=1)
        self.success_pub = rospy.Publisher('/resolve_data_management_failure_success', Bool, queue_size=1)

    def logging_success_callback(self, msg):
        self.fail_cnt = 0

    def resolve_callback(self, msg):
        rospy.loginfo("launch data management failure resolver..")
        rospy.sleep(2)
        self.resolution_pub.publish("launch data management failure resolver -- type of failure: " + msg.data)
        rospy.loginfo("type of data management failure: %s", msg.data)
        self.problem_resolved = False

        # different types of resolution are required based on the type of issue
        if msg.data == config.DATA_MANAGEMENT_FAILURES[0]:
            self.resolve_type_one_failure(msg.data)
        elif msg.data == config.DATA_MANAGEMENT_FAILURES[1]:
            self.resolve_type_two_failure(msg.data)
        elif msg.data == config.DATA_MANAGEMENT_CATA:
            self.resolve_catastrophe(msg.data)

        if self.problem_resolved:
            self.resolution_pub.publish("problem resolved")
            self.success_pub.publish(True)

    def resolve_catastrophe(self, msg):
        rospy.loginfo("resolve data management catastrophe -- requesting help from human operator")
        self.resolution_pub.publish("resolve data management catastrophe -- requesting help from human operator")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_one_failure(self, msg):
        rospy.loginfo("resolve type one failure.. a full memory can't be dealt with by the robot, but it can drive back to the base..")
        self.resolution_pub.publish("resolve type one failure")
        # return to base + launch catastrohe
        # TODO: implement docking for config.DOCKING cases
        action_goal = util.create_dtg_goal(config.BASE_POSE, None)
        self.drive_to_goal_client.wait_for_server()
        self.drive_to_goal_client.send_goal(action_goal)
        rospy.loginfo("goal sent, wait for accomplishment..")
        self.drive_to_goal_client.wait_for_result()
        # initiate catastrophe
        self.success_pub.publish(False)

    def resolve_type_two_failure(self, msg):
        rospy.loginfo("resolve type two failure..")
        if self.fail_cnt == 0:
            # fail count 0 -> just try again
            rospy.loginfo("just trying again..")
            self.resolution_pub.publish("resolve type two failure - trying again..")
            self.problem_resolved = True
        else:
            # initiate catastrophe
            self.success_pub.publish(False)
        self.fail_cnt += 1


class ConnectionResolver(GeneralFailureResolver):

    def __init__(self):
        super(ConnectionResolver, self).__init__()
        self.fail_cnt = 0
        self.fail_cnt_update = None
        rospy.Subscriber('/resolve_wifi_failure', String, self.resolve_callback, queue_size=1)
        rospy.Subscriber('/resolve_internet_failure', String, self.resolve_callback, queue_size=1)
        rospy.Subscriber('/resolve_gnss_failure', String, self.resolve_callback, queue_size=1)
        self.success_pub = rospy.Publisher('/resolve_connection_failure_success', Bool, queue_size=1)
        self.re_init_pub = rospy.Publisher('/re_init_internet_monitoring', String, queue_size=1)

    def resolve_callback(self, msg):
        rospy.loginfo("launch connection failure resolver..")
        rospy.loginfo("type of connection failure: %s", msg.data)
        rospy.sleep(2)
        self.resolution_pub.publish("launch connection failure resolver -- type of connection failure: " + msg.data)
        self.problem_resolved = False

        # fail count outdated -> reset
        if self.fail_cnt_update and (datetime.now() - self.fail_cnt_update).total_seconds() > 500:
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
        rospy.loginfo("resolve connection catastrophe -- requesting help from human operator")
        self.resolution_pub.publish("resolve connection catastrophe -- requesting help from human operator")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_wifi_failure(self, msg):
        rospy.loginfo("resolve wifi failure: %s", msg)
        self.resolution_pub.publish("resolve wifi failure: " + str(msg))
        
        if self.fail_cnt == 0:
            # simply repeat it
            rospy.loginfo("just trying again..")
            self.resolution_pub.publish("just trying again..")
            self.problem_resolved = True
        else:
            # initiate catastrophe
            self.success_pub.publish(False)
        self.fail_cnt += 1
        self.fail_cnt_update = datetime.now()

    def resolve_internet_failure(self, msg):
        rospy.loginfo("resolve internet failure: %s", msg)
        self.resolution_pub.publish("resolve internet failure: " + str(msg))

        if self.fail_cnt == 0:
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
        rospy.loginfo("resolve GNSS failure: %s", msg)
        self.resolution_pub.publish("resolve GNSS failure: " + str(msg))

        if self.fail_cnt == 0:
            # simply repeat it
            rospy.loginfo("just trying again..")
            self.resolution_pub.publish("just trying again..")
            self.problem_resolved = True
        else:
            # initiate catastrophe
            self.success_pub.publish(False)
        self.fail_cnt += 1
        self.fail_cnt_update = datetime.now()


class PowerManagementFailureResolver(GeneralFailureResolver):

    def __init__(self):
        super(PowerManagementFailureResolver, self).__init__()
        rospy.Subscriber('/resolve_power_management_failure', String, self.resolve_callback, queue_size=1)
        self.success_pub = rospy.Publisher('/resolve_power_management_failure_success', Bool, queue_size=1)
        self.insert_goal_pub = rospy.Publisher('introduce_intermediate_recharge_goal', String, queue_size=1)
        self.reset_discharge_rate_pub = rospy.Publisher('/reset_discharge_rate', String, queue_size=1)
        self.cata_launched_pub = rospy.Publisher('/catastrophe_launched', String, queue_size=1)

    def resolve_callback(self, msg):
        rospy.loginfo("launch power management failure resolver..")
        rospy.loginfo("type of power management failure: %s", msg.data)
        self.reset_discharge_rate_pub.publish(msg)
        rospy.sleep(2)
        self.resolution_pub.publish("launch power management failure resolver -- type of power management failure: " + msg.data)
        self.problem_resolved = False

        # different types of resolution are required based on the type of issue
        if msg.data == config.POWER_MANAGEMENT_FAILURE_ONE:
            self.resolve_contingency(config.POWER_MANAGEMENT_FAILURE_ONE)
        elif msg.data == config.POWER_MANAGEMENT_CATA:
            self.resolve_catastrophe(config.POWER_MANAGEMENT_CATA)

        if self.problem_resolved:
            self.resolution_pub.publish("problem resolved")
            self.success_pub.publish(True)

    def resolve_contingency(self, msg):
        rospy.loginfo("resolve power management contingency -- introduce intermediate goals in plan - [return_to_base, charge]")
        self.resolution_pub.publish("resolve power management contingency -- introduce intermediate goals in plan - [return_to_base, charge]")
        # insert intermediate recharge goals [return_to_base, charge]
        self.insert_goal_pub.publish("")
        self.problem_resolved = True

    def resolve_catastrophe(self, msg):
        rospy.loginfo("resolve power management catastrophe -- requesting help from human operator")
        self.resolution_pub.publish("resolve power management catastrophe -- requesting help from human operator")
        self.fallback_pub.publish(msg)
        self.cata_launched_pub.publish("")
        while not self.problem_resolved:
            rospy.sleep(5)


class SensorFailureResolver(GeneralFailureResolver):

    def __init__(self):
        super(SensorFailureResolver, self).__init__()
        self.fail_cnt = 0
        self.fail_cnt_update = None
        rospy.Subscriber('/resolve_sensor_failure', String, self.resolve_callback, queue_size=1)
        self.success_pub = rospy.Publisher('/resolve_sensor_failure_success', Bool, queue_size=1)

    def resolve_callback(self, msg):
        rospy.loginfo("launch sensor failure resolver..")
        rospy.loginfo("type of sensor failure: %s", msg.data)
        rospy.sleep(2)
        self.resolution_pub.publish("launch sensor failure resolver -- type of sensor failure: " + msg.data)
        self.problem_resolved = False

        # reset outdated fail count
        if self.fail_cnt_update and (datetime.now() - self.fail_cnt_update).total_seconds() > 300:
            self.fail_cnt = 0

        if msg.data in config.SENSOR_FAILURES.values():
            self.resolve_sensor_failure(msg.data)
        elif msg.data == config.SENSOR_CATA:
            self.resolve_catastrophe(config.SENSOR_CATA)

        if self.problem_resolved:
            self.resolution_pub.publish("problem resolved")
            self.success_pub.publish(True)

    def resolve_catastrophe(self, msg):
        rospy.loginfo("resolve sensor catastrophe -- requesting help from human operator")
        self.resolution_pub.publish("resolve sensor catastrophe -- requesting help from human operator")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_sensor_failure(self, msg):
        if self.fail_cnt == 0:
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

    def __init__(self):
        super(LocalizationFailureResolver, self).__init__()
        rospy.Subscriber('/resolve_localization_failure', String, self.resolve_callback, queue_size=1)
        self.success_pub = rospy.Publisher('/resolve_localization_failure_success', Bool, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def resolve_callback(self, msg):
        rospy.loginfo("launch localization failure resolver..")
        rospy.loginfo("type of localization failure: %s", msg.data)
        self.resolution_pub.publish("launch localization failure resolver -- type of localization failure: " + msg.data)
        self.problem_resolved = False

        if msg.data in config.LOCALIZATION_FAILURES.values():
            self.resolve_localization_failure()

        if self.problem_resolved:
            self.resolution_pub.publish("problem resolved")
            self.success_pub.publish(True)

    def clear_costmaps(self):
        rospy.wait_for_service('/move_base_flex/clear_costmaps')
        clear_costmaps_service = rospy.ServiceProxy('/move_base_flex/clear_costmaps', Empty)
        rec_client = actionlib.SimpleActionClient("move_base_flex/recovery", RecoveryAction)
        rec_client.wait_for_server()

        # order matters -> first global, then local
        rospy.loginfo("clearing global costmap..")
        self.resolution_pub.publish("clearing global costmap")
        try:
            clear_costmaps_service()
        except rospy.ServiceException as e:
            rospy.loginfo("error: %s", e)

        rospy.loginfo("clearing local costmap..")
        self.resolution_pub.publish("clearing local costmap")
        # concurrency_slot 3
        clear_local_costmap_goal = RecoveryGoal('clear_costmap', 3)
        rec_client.send_goal(clear_local_costmap_goal)
        res = rec_client.wait_for_result()
        if res:
            rospy.loginfo("cleared costmap..")

    def resolve_localization_failure(self):
        rospy.loginfo("resolve localization failure..")
        # sleeping a moment to wait for the robot to stand still
        rospy.sleep(5)
        # TODO: should pay attention to obstacles etc.
        rospy.loginfo("driving the robot a few meters back and forth to recalibrate the localization using different GNSS positions..")
        self.resolution_pub.publish("driving the robot a few meters back and forth to recalibrate the localization using different GNSS positions")
        twist = Twist()
        twist.linear.x = -3.0
        rate = rospy.Rate(4)
        for _ in range(2):
            for _ in range(4):
                self.cmd_vel_pub.publish(twist)
                rate.sleep()
            twist.linear.x = 3.0

        rospy.loginfo("clearing costmaps..")
        self.clear_costmaps()
        self.problem_resolved = True


class PlanDeploymentFailureResolver(GeneralFailureResolver):

    def __init__(self):
        super(PlanDeploymentFailureResolver, self).__init__()
        rospy.Subscriber('/resolve_plan_deployment_failure', String, self.resolve_callback, queue_size=1)
        rospy.Subscriber('arox/ongoing_operation', arox_operational_param, self.operation_callback, queue_size=1)
        self.success_pub = rospy.Publisher('/resolve_plan_deployment_failure_success', Bool, queue_size=1)
        self.activate_plan_service_pub = rospy.Publisher('/activate_plan_service', String, queue_size=1)

        self.infeasible_plan_cnt = 0
        self.empty_plan_cnt = 0
        self.unavailable_service_cnt = 0

    def operation_callback(self, msg):
        # the repetition of the plan retrieval attempt was obviously successful
        self.infeasible_plan_cnt = self.empty_plan_cnt = 0

    def resolve_callback(self, msg):
        rospy.loginfo("launch plan deployment failure resolver..")
        rospy.loginfo("type of plan deployment failure: %s", msg.data)
        rospy.sleep(2)
        self.resolution_pub.publish("launch plan deployment failure resolver -- type of plan deployment failure: " + msg.data)
        self.problem_resolved = False

        # different types of resolution are required based on the type of issue
        if msg.data == config.PLAN_DEPLOYMENT_FAILURES[1]:
            self.resolve_type_two_failure(msg.data)
        elif msg.data in [config.PLAN_DEPLOYMENT_FAILURES[2], config.PLAN_DEPLOYMENT_FAILURES[4]]:
            self.resolve_type_three_failure(msg.data)
        elif msg.data == config.PLAN_DEPLOYMENT_FAILURES[3]:
            self.resolve_type_four_failure(msg.data)
        elif msg.data == config.PLAN_DEPLOYMENT_CATA:
            self.resolve_catastrophe(msg.data)

        if self.problem_resolved:
            self.resolution_pub.publish("problem resolved")
            self.success_pub.publish(True)

    def resolve_catastrophe(self, msg):
        rospy.loginfo("resolve plan deployment catastrophe -- requesting help from human operator")
        self.resolution_pub.publish("resolve plan deployment catastrophe -- requesting help from human operator")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_two_failure(self, msg):
        rospy.loginfo("resolve type two failure..")
        self.resolution_pub.publish("resolve type two failure -- resolve by trying to bring up the service")
        if self.unavailable_service_cnt == 0:
            rospy.loginfo("resolve by trying to bring up the service..")
            self.activate_plan_service_pub.publish("bring up service")
            self.unavailable_service_cnt += 1
            self.problem_resolved = True
        else:
            rospy.loginfo("plan generation service repeatedly unavailable..")
            self.resolution_pub.publish("plan generation service repeatedly unavailable")
            # initiate catastrophe
            self.success_pub.publish(False)

    def resolve_type_three_failure(self, msg):
        rospy.loginfo("resolve type three failure..")
        self.resolution_pub.publish("resolve type three failure -- resolve by trying again")
        if self.empty_plan_cnt == 0:
            rospy.loginfo("resolve by trying again..")
            self.empty_plan_cnt += 1
            self.problem_resolved = True
        else:
            rospy.loginfo("plan retrieval failed repeatedly..")
            self.resolution_pub.publish("plan retrieval failed repeatedly")
            # initiate catastrophe
            self.success_pub.publish(False)

    def resolve_type_four_failure(self, msg):
        rospy.loginfo("resolve type four failure..")
        self.resolution_pub.publish("resolve type four failure -- resolve by trying again")
        if self.infeasible_plan_cnt == 0:
            rospy.loginfo("resolve by trying again..")
            self.infeasible_plan_cnt += 1
            self.problem_resolved = True
        else:
            rospy.loginfo("plan retrieval failed repeatedly..")
            self.resolution_pub.publish("plan retrieval failed repeatedly")
            # initiate catastrophe
            self.success_pub.publish(False)


class NavigationFailureResolver(GeneralFailureResolver):

    def __init__(self):
        super(NavigationFailureResolver, self).__init__()
        rospy.Subscriber('/resolve_navigation_failure', String, self.resolve_callback, queue_size=1)
        rospy.Subscriber('/resolution_failure', String, self.resolution_failure_callback, queue_size=1)
        self.success_pub = rospy.Publisher('/resolve_navigation_failure_success', Bool, queue_size=1)
        self.remove_obstacles_pub = rospy.Publisher('/clear_spawned_obstacles', String, queue_size=1)
        self.drive_to_goal_client = actionlib.SimpleActionClient('drive_to_goal', drive_to_goalAction)

    def resolution_failure_callback(self, msg):
        rospy.loginfo("nav fail resolution failed.. cancelling resolution attempt..")
        self.resolution_pub.publish("nav fail resolution failed.. cancelling resolution attempt..")
        self.drive_to_goal_client.cancel_all_goals()
        # initiate catastrophe
        self.success_pub.publish(False)
        # solved -- obstacles removed
        self.remove_obstacles_pub.publish("")
        # wait for obstacle removal before costmap clearance
        rospy.sleep(3)
        self.clear_costmaps()

    def resolve_callback(self, msg):
        rospy.loginfo("launch navigation failure resolver..")
        rospy.loginfo("type of navigation failure: %s", msg.data)
        rospy.sleep(2)
        self.resolution_pub.publish("launch navigation failure resolver -- type of navigation failure: " + msg.data)
        self.problem_resolved = False

        if msg.data == config.NAV_FAILURE_ONE or msg.data == config.NAV_FAILURE_THREE:
            self.resolve_nav_failure(config.NAV_FAILURE_ONE)
        elif msg.data == config.NAV_CATA:
            self.resolve_catastrophe(config.NAV_CATA)
        else:
            rospy.loginfo("cannot resolve unknown nav failure: %s", msg.data)
            self.resolution_pub.publish("cannot resolve unknown nav failure: " + msg.data)

        if self.problem_resolved:
            self.resolution_pub.publish("problem resolved")
            self.success_pub.publish(True)

    def resolve_catastrophe(self, msg):
        rospy.loginfo("resolve nav catastrophe -- requesting help from human operator")
        self.resolution_pub.publish("resolve nav catastrophe -- requesting help from human operator")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def clear_costmaps(self):
        rospy.wait_for_service('/move_base_flex/clear_costmaps')
        clear_costmaps_service = rospy.ServiceProxy('/move_base_flex/clear_costmaps', Empty)
        rec_client = actionlib.SimpleActionClient("move_base_flex/recovery", RecoveryAction)
        rec_client.wait_for_server()

        # order matters -> first global, then local
        rospy.loginfo("clearing global costmap..")
        self.resolution_pub.publish("clearing global costmap")
        try:
            clear_costmaps_service()
        except rospy.ServiceException as e:
            rospy.loginfo("error: %s", e)

        rospy.loginfo("clearing local costmap..")
        self.resolution_pub.publish("clearing local costmap")
        # concurrency_slot 3
        clear_local_costmap_goal = RecoveryGoal('clear_costmap', 3)
        rec_client.send_goal(clear_local_costmap_goal)
        res = rec_client.wait_for_result()
        if res:
            rospy.loginfo("cleared costmap..")

    def resolve_nav_failure(self, msg):
        rospy.loginfo("resolve navigation failure.. driving to recovery point..")
        self.resolution_pub.publish("resolve navigation failure -- driving to recovery point")
        self.clear_costmaps()

        # TODO: implement and try more potential recovery points
        action_goal = util.create_dtg_goal(config.RECOVERY_POINT_ONE, None)
        self.drive_to_goal_client.wait_for_server()
        self.drive_to_goal_client.send_goal(action_goal)
        rospy.loginfo("goal sent, wait for accomplishment..")
        self.drive_to_goal_client.wait_for_result()
        out = self.drive_to_goal_client.get_result()

        # navigation failure during resolution -- goal not reached
        if self.drive_to_goal_client.get_state() == config.GOAL_STATUS_ABORTED:
            rospy.loginfo("nav failure during resolution -- notifying operator..")
            self.resolution_pub.publish("nav failure during resolution -- notifying operator")
            # initiate catastrophe
            self.success_pub.publish(False)
            # solved -- obstacles removed
            self.remove_obstacles_pub.publish("")
            # wait for obstacle removal before costmap clearance
            rospy.sleep(3)
            self.clear_costmaps()

        elif self.drive_to_goal_client.get_state() == config.GOAL_STATUS_SUCCEEDED:
            self.problem_resolved = True


class ChargingFailureResolver(GeneralFailureResolver):

    def __init__(self):
        super(ChargingFailureResolver, self).__init__()
        rospy.Subscriber('/resolve_charging_failure', String, self.resolve_callback, queue_size=1)
        rospy.Subscriber('/arox/battery_param', arox_battery_params, self.battery_callback, queue_size=1)
        rospy.Subscriber('/move_base_flex/exe_path/status', GoalStatusArray, self.mbf_status_callback, queue_size=1)

        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.success_pub = rospy.Publisher('/resolve_charging_failure_success', Bool, queue_size=1)
        self.insert_goal_pub = rospy.Publisher('introduce_intermediate_nav_goal', String, queue_size=1)

        self.charge_level_at_resolution = 0.0
        self.latest_charge_level = 0.0
        self.docking_fail_cnt = 0
        self.undocking_fail_cnt = 0
        self.charge_fail_cnt = 0

    def mbf_status_callback(self, mbf_status):
        if len(mbf_status.status_list) > 0 and mbf_status.status_list[-1].status == config.GOAL_STATUS_SUCCEEDED:
            # undocking was obviously successful - reset fail cnt
            self.undocking_fail_cnt = 0

    def battery_callback(self, msg):
        self.latest_charge_level = msg.charge
        if self.latest_charge_level > self.charge_level_at_resolution:
            # resolution successful - reset fail cnt
            self.docking_fail_cnt = 0
            self.charge_fail_cnt = 0

    def resolve_callback(self, msg):
        rospy.loginfo("launch charging failure resolver..")
        rospy.loginfo("type of charging failure: %s", msg.data)
        rospy.sleep(2)
        self.resolution_pub.publish("launch charging failure resolver -- type of charging failure: " + msg.data)
        self.problem_resolved = False

        if msg.data == config.CHARGING_FAILURE_ONE:
            self.resolve_docking_failure()
        elif msg.data == config.CHARGING_FAILURE_TWO:
            self.resolve_undocking_failure()
        elif msg.data == config.CHARGING_FAILURE_THREE:
            self.resolve_charging_failure()
        elif msg.data == config.CHARGING_CATA:
            self.resolve_catastrophe(config.CHARGING_CATA)
        else:
            rospy.loginfo("cannot resolve unknown nav failure: %s", msg.data)

        if self.problem_resolved:
            self.resolution_pub.publish("problem resolved")
            self.success_pub.publish(True)
    
    def resolve_catastrophe(self, msg):
        rospy.loginfo("resolve charging catastrophe -- requesting help from human operator")
        self.resolution_pub.publish("resolve charging catastrophe -- requesting help from human operator")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_docking_failure(self):
        rospy.loginfo("resolving docking failure..")
        self.resolution_pub.publish("resolving docking failure")
        self.charge_level_at_resolution = self.latest_charge_level

        # already tried resolution before - call human
        if self.docking_fail_cnt == 1:
            rospy.loginfo("already tried autonomous resolution before -- calling human operator for help..")
            self.resolution_pub.publish("already tried autonomous resolution before -- calling human operator for help")
            # initiate catastrophe
            self.success_pub.publish(False)
            # human would have opened the container -- in case it was closed
            rospy.loginfo("sending command to open container front..")
            container_pub = rospy.Publisher('/container/rampB_position_controller/command', Float64, queue_size=1)
            for _ in range(3):
                container_pub.publish(2.0)
                rospy.sleep(0.5)
            # clear costmap to perceive that the door is open now
            rospy.sleep(5)
            self.clear_costmaps()
        else:
            rospy.loginfo("just trying again..")
            self.resolution_pub.publish("just trying again")
            self.problem_resolved = True
            self.docking_fail_cnt += 1

    def clear_costmaps(self):
        rospy.wait_for_service('/move_base_flex/clear_costmaps')
        clear_costmaps_service = rospy.ServiceProxy('/move_base_flex/clear_costmaps', Empty)
        rec_client = actionlib.SimpleActionClient("move_base_flex/recovery", RecoveryAction)
        rec_client.wait_for_server()

        # order matters -> first global, then local
        rospy.loginfo("clearing global costmap..")
        self.resolution_pub.publish("clearing global costmap")
        try:
            clear_costmaps_service()
        except rospy.ServiceException as e:
            rospy.loginfo("error: %s", e)

        rospy.loginfo("clearing local costmap..")
        self.resolution_pub.publish("clearing local costmap")
        # concurrency_slot 3
        clear_local_costmap_goal = RecoveryGoal('clear_costmap', 3)
        rec_client.send_goal(clear_local_costmap_goal)
        res = rec_client.wait_for_result()
        if res:
            rospy.loginfo("cleared costmap..")

    def resolve_undocking_failure(self):
         # already tried resolution before - call human
        if self.undocking_fail_cnt == 1:
            rospy.loginfo("already tried autonomous resolution before -- calling human operator for help..")
            self.resolution_pub.publish("already tried autonomous resolution before -- calling human operator for help")

            # initiate catastrophe
            self.success_pub.publish(False)

            # human would have opened the container -- in case it was closed
            rospy.loginfo("sending command to open container front..")
            container_pub = rospy.Publisher('/container/rampB_position_controller/command', Float64, queue_size=1)
            for _ in range(3):
                container_pub.publish(2.0)
                rospy.sleep(0.5)
            # clear costmap to perceive that the door is open now
            rospy.sleep(5)
            self.clear_costmaps()
        else:
            self.resolution_pub.publish("driving robot back and forth -- minor relocation")
            twist = Twist()
            twist.linear.x = 3.0
            rate = rospy.Rate(2)
            for _ in range(2):
                for _ in range(2):
                    self.cmd_vel_pub.publish(twist)
                    rate.sleep()
                twist.linear.x = -3.0

            self.problem_resolved = True
            self.undocking_fail_cnt += 1

    def resolve_charging_failure(self):
        rospy.loginfo("resolving charging failure..")
        self.resolution_pub.publish("resolving charging failure")
        self.charge_level_at_resolution =  self.latest_charge_level

        if self.charge_fail_cnt == 1:
            rospy.loginfo("already tried autonomous resolution before -- calling human operator for help..")
            self.resolution_pub.publish("already tried autonomous resolution before -- calling human operator for help")
            # initiate catastrophe
            self.success_pub.publish(False)
        else:
            self.resolution_pub.publish("driving robot back and forth -- minor relocation")
            twist = Twist()
            twist.linear.x = 3.0
            rate = rospy.Rate(2)
            for _ in range(2):
                for _ in range(2):
                    self.cmd_vel_pub.publish(twist)
                    rate.sleep()
                twist.linear.x = -3.0

            self.problem_resolved = True
            self.charge_fail_cnt += 1

def node():
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
