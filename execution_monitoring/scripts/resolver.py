#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Bool, Float64
import actionlib
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
        rospy.Subscriber('/problem_solved', String, self.solved_by_human_callback, queue_size=1)
        self.human_operator_contingency_pub = rospy.Publisher("/request_help_contingency", String, queue_size=1)
        self.human_operator_catastrophe_pub = rospy.Publisher("/request_help_catastrophe", String, queue_size=1)
        self.fallback_pub = rospy.Publisher("/fallback_success", Bool, queue_size=1)
        self.resolution_pub = rospy.Publisher('/resolution', String, queue_size=1)
        self.problem_resolved = False

    def request_fallback(self, msg):
        rospy.loginfo("fallback requested.. communicating problem to human operator..")
        rospy.loginfo("problem: %s", msg.data)
        self.resolution_pub.publish("fallback requested -- communicating problem to human operator -- problem: " + msg.data)
        self.problem_resolved = False
        self.human_operator_contingency_pub.publish(msg.data)

        # TODO: when it takes too long it should go to CATASTROPHE
        while not self.problem_resolved:
            rospy.loginfo("waiting for human operator to solve the problem..")
            rospy.sleep(5)
        
        rospy.loginfo("failure resolved..")
        self.fallback_pub.publish(True)

    def solved_by_human_callback(self, msg):
        rospy.loginfo("solved by human operator: %s", msg.data)
        self.resolution_pub.publish("solved by human operator: %" + msg.data)
        self.problem_resolved = True


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
        rospy.Subscriber('/moderate_weather', String, self.moderate_weather_callback, queue_size=1)
        self.success_pub = rospy.Publisher('/resolve_weather_failure_success', Bool, queue_size=1)
        self.drive_to_goal_client = actionlib.SimpleActionClient('drive_to_goal', drive_to_goalAction)
        self.moderate_weather = True

    def resolve_callback(self, msg):
        rospy.loginfo("launch weather failure resolver..")
        rospy.loginfo("type of weather failure: %s", msg.data)
        rospy.sleep(2)
        self.resolution_pub.publish("launch weather failure resolver -- type of weather failure: " + msg.data)
        self.problem_resolved = False
        self.moderate_weather = False

        # all these failures are resolved by seeking shelter and waiting
        if msg.data in [config.WEATHER_FAILURE_TWO, config.WEATHER_FAILURE_FIVE, config.WEATHER_FAILURE_EIGHT, config.WEATHER_FAILURE_NINE, config.WEATHER_FAILURE_TEN,
            config.WEATHER_FAILURE_ELEVEN, config.WEATHER_FAILURE_TWELVE, config.WEATHER_FAILURE_THIRTEEN, config.WEATHER_FAILURE_FOURTEEN,
            config.WEATHER_FAILURE_FIFTEEN, config.WEATHER_FAILURE_SIXTEEN, config.WEATHER_FAILURE_SEVENTEEN, config.WEATHER_FAILURE_EIGHTEEN]:

            self.resolve_weather_failure(msg.data)

        if self.problem_resolved:
            self.success_pub.publish(True)

    def moderate_weather_callback(self, msg):
        self.moderate_weather = True

    def resolve_weather_failure(self, msg):
        self.resolution_pub.publish("resolve drastic weather change -- seeking shelter -- driving back to base")
        rospy.loginfo("resolve drastic weather change..")
        rospy.loginfo("seeking shelter - driving back to base..")

        # TODO: implement docking for config.DOCKING cases
        action_goal = util.create_dtg_goal(config.BASE_POSE, None)
        self.drive_to_goal_client.wait_for_server()
        self.drive_to_goal_client.send_goal(action_goal)
        rospy.loginfo("goal sent, wait for accomplishment..")
        self.drive_to_goal_client.wait_for_result()

        out = self.drive_to_goal_client.get_result()
        if out.progress > 0:
            self.fallback_pub.publish(msg)
            while not self.problem_resolved:
                rospy.sleep(5)

        rospy.loginfo("start docking procedure..")
        rospy.loginfo("waiting until weather is moderate again - charging battery in the meantime..")
        self.robot_info_pub.publish("waiting until weather is moderate again -- charging battery in the meantime")
        self.resolution_pub.publish("waiting until weather is moderate again -- charging battery in the meantime")

        rospy.set_param("charging_mode", True)
        charge_mode = rospy.get_param("charging_mode")

        while charge_mode:
            charge_mode = rospy.get_param("charging_mode")
            rospy.loginfo("charging battery..")
            rospy.sleep(2)

        if not charge_mode:
            rospy.loginfo("battery charged..")
            self.robot_info_pub.publish("battery charged")

        rospy.loginfo("waiting until weather is moderate again")
        while not self.moderate_weather:
            rospy.sleep(5)
        
        self.resolution_pub.publish("weather moderate again -- problem solved")
        self.problem_resolved = True


class DataManagementFailureResolver(GeneralFailureResolver):

    def __init__(self):
        super(DataManagementFailureResolver, self).__init__()
        rospy.Subscriber('/resolve_data_management_failure', String, self.resolve_callback, queue_size=1)
        self.success_pub = rospy.Publisher('/resolve_data_management_failure_success', Bool, queue_size=1)

    def resolve_callback(self, msg):
        rospy.loginfo("launch data management failure resolver..")
        rospy.sleep(2)
        self.resolution_pub.publish("launch data management failure resolver -- type of failure: " + msg.data)
        rospy.loginfo("type of data management failure: %s", msg.data)
        self.problem_resolved = False

        # different types of resolution are required based on the type of issue
        if msg.data == config.DATA_MANAGEMENT_FAILURE_ONE:
            self.resolve_type_one_failure(config.DATA_MANAGEMENT_FAILURE_ONE)
        elif msg.data == config.DATA_MANAGEMENT_FAILURE_TWO:
            self.resolve_type_two_failure(config.DATA_MANAGEMENT_FAILURE_TWO)

        if self.problem_resolved:
            self.resolution_pub.publish("problem resolved")
            self.success_pub.publish(True)

    def resolve_type_one_failure(self, msg):
        rospy.loginfo("resolve type one failure..")
        self.resolution_pub.publish("resolve type one failure")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_two_failure(self, msg):
        rospy.loginfo("resolve type two failure..")
        self.resolution_pub.publish("resolve type two failure")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)


class ConnectionResolver(GeneralFailureResolver):

    def __init__(self):
        super(ConnectionResolver, self).__init__()
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

        # different types of resolution are required based on the type of issue
        if msg.data == config.CONNECTION_FAILURE_ONE:
            self.resolve_type_one_failure(config.CONNECTION_FAILURE_ONE)
        elif msg.data == config.CONNECTION_FAILURE_TWO:
            self.resolve_type_two_failure(config.CONNECTION_FAILURE_TWO)
        elif msg.data == config.CONNECTION_FAILURE_THREE:
            self.resolve_type_three_failure(config.CONNECTION_FAILURE_THREE)
        elif msg.data == config.CONNECTION_FAILURE_FOUR:
            self.resolve_type_four_failure(config.CONNECTION_FAILURE_FOUR)
        elif msg.data == config.CONNECTION_FAILURE_FIVE:
            self.resolve_type_five_failure(config.CONNECTION_FAILURE_FIVE)
        elif msg.data == config.CONNECTION_FAILURE_SIX:
            self.resolve_type_six_failure(config.CONNECTION_FAILURE_SIX)
        elif msg.data == config.CONNECTION_FAILURE_SEVEN:
            self.resolve_type_seven_failure(config.CONNECTION_FAILURE_SEVEN)
        elif msg.data == config.CONNECTION_FAILURE_EIGHT:
            self.resolve_type_eight_failure(config.CONNECTION_FAILURE_EIGHT)
        elif msg.data == config.CONNECTION_FAILURE_NINE:
            self.resolve_type_ninie_failure(config.CONNECTION_FAILURE_NINE)
        elif msg.data == config.CONNECTION_FAILURE_TEN:
            self.resolve_type_ten_failure(config.CONNECTION_FAILURE_TEN)
        elif msg.data == config.CONNECTION_FAILURE_ELEVEN:
            self.resolve_type_eleven_failure(config.CONNECTION_FAILURE_ELEVEN)
        elif msg.data == config.CONNECTION_FAILURE_TWELVE:
            self.resolve_type_twelve_failure(config.CONNECTION_FAILURE_TWELVE)
        elif msg.data == config.CONNECTION_FAILURE_THIRTEEN:
            self.resolve_type_thirteen_failure(config.CONNECTION_FAILURE_THIRTEEN)
        elif msg.data == config.CONNECTION_FAILURE_FOURTEEN:
            self.resolve_type_fourteen_failure(config.CONNECTION_FAILURE_FOURTEEN)
        elif msg.data == config.CONNECTION_FAILURE_FIFTEEN:
            self.resolve_type_fifteen_failure(config.CONNECTION_FAILURE_FIFTEEN)
        elif msg.data == config.CONNECTION_FAILURE_SIXTEEN:
            self.resolve_type_sixteen_failure(config.CONNECTION_FAILURE_SIXTEEN)
        elif msg.data == config.CONNECTION_FAILURE_SEVENTEEN:
            self.resolve_type_seventeen_failure(config.CONNECTION_FAILURE_SEVENTEEN)
        elif msg.data == config.CONNECTION_FAILURE_EIGHTEEN:
            self.resolve_type_eighteen_failure(config.CONNECTION_FAILURE_EIGHTEEN)
        elif msg.data == config.CONNECTION_FAILURE_NINETEEN:
            self.resolve_type_nineteen_failure(config.CONNECTION_FAILURE_NINETEEN)
        elif msg.data == config.CONNECTION_FAILURE_TWENTY:
            self.resolve_type_twenty_failure(config.CONNECTION_FAILURE_TWENTY)

        if self.problem_resolved:
            self.resolution_pub.publish("problem resolved")
            self.success_pub.publish(True)

    def resolve_type_one_failure(self, msg):
        rospy.loginfo("resolve type one failure..")
        self.resolution_pub.publish("resolve type one failure")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_two_failure(self, msg):
        rospy.loginfo("resolve type two failure..")
        self.resolution_pub.publish("resolve type two failure")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_three_failure(self, msg):
        rospy.loginfo("resolve type three failure..")
        self.resolution_pub.publish("resolve type three failure")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_four_failure(self, msg):
        rospy.loginfo("resolve type four failure..")
        self.resolution_pub.publish("resolve type four failure")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_five_failure(self, msg):
        rospy.loginfo("resolve type five failure..")
        self.resolution_pub.publish("resolve type five failure")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)
        # try to re-initialize the internet monitor (requires connection)
        self.re_init_pub.publish("")

    def resolve_type_six_failure(self, msg):
        rospy.loginfo("resolve type six failure..")
        self.resolution_pub.publish("resolve type six failure")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_seven_failure(self, msg):
        rospy.loginfo("resolve type seven failure..")
        self.resolution_pub.publish("resolve type seven failure")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_eight_failure(self, msg):
        rospy.loginfo("resolve type eight failure..")
        self.resolution_pub.publish("resolve type eight failure")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_nine_failure(self, msg):
        rospy.loginfo("resolve type nine failure..")
        self.resolution_pub.publish("resolve type nine failure")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_ten_failure(self, msg):
        rospy.loginfo("resolve type ten failure..")
        self.resolution_pub.publish("resolve type ten failure")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_eleven_failure(self, msg):
        rospy.loginfo("resolve type eleven failure..")
        self.resolution_pub.publish("resolve type eleven failure")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_twelve_failure(self, msg):
        rospy.loginfo("resolve type twelve failure..")
        self.resolution_pub.publish("resolve type twelve failure")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_thirteen_failure(self, msg):
        rospy.loginfo("resolve type thirteen failure..")
        self.resolution_pub.publish("resolve type thirteen failure")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_fourteen_failure(self, msg):
        rospy.loginfo("resolve type fourteen failure..")
        self.resolution_pub.publish("resolve type fourteen failure")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)
        
    def resolve_type_fifteen_failure(self, msg):
        rospy.loginfo("resolve type fifteen failure..")
        self.resolution_pub.publish("resolve type fifteen failure")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_sixteen_failure(self, msg):
        rospy.loginfo("resolve type sixteen failure..")
        self.resolution_pub.publish("resolve type sixteen failure")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_seventeen_failure(self, msg):
        rospy.loginfo("resolve type seventeen failure..")
        self.resolution_pub.publish("resolve type seventeen failure")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_eighteen_failure(self, msg):
        rospy.loginfo("resolve type eighteen failure..")
        self.resolution_pub.publish("resolve type eighteen failure")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_nineteen_failure(self, msg):
        rospy.loginfo("resolve type nineteen failure..")
        self.resolution_pub.publish("resolve type nineteen failure")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_twenty_failure(self, msg):
        rospy.loginfo("resolve type twenty failure..")
        self.resolution_pub.publish("resolve type twenty failure")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

class PowerManagementFailureResolver(GeneralFailureResolver):

    def __init__(self):
        super(PowerManagementFailureResolver, self).__init__()
        rospy.Subscriber('/resolve_power_management_failure', String, self.resolve_callback, queue_size=1)
        self.success_pub = rospy.Publisher('/resolve_power_management_failure_success', Bool, queue_size=1)

    def resolve_callback(self, msg):
        rospy.loginfo("launch power management failure resolver..")
        rospy.loginfo("type of power management failure: %s", msg.data)
        rospy.sleep(2)
        self.resolution_pub.publish("launch power management failure resolver -- type of power management failure: " + msg.data)
        self.problem_resolved = False

        # different types of resolution are required based on the type of issue
        if msg.data == config.POWER_MANAGEMENT_FAILURE_ONE:
            self.resolve_type_one_failure(config.POWER_MANAGEMENT_FAILURE_ONE)
        elif msg.data == config.POWER_MANAGEMENT_FAILURE_TWO:
            self.resolve_type_two_failure(config.POWER_MANAGEMENT_FAILURE_TWO)

        if self.problem_resolved:
            self.resolution_pub.publish("problem resolved")
            self.success_pub.publish(True)

    def resolve_type_one_failure(self, msg):
        rospy.loginfo("resolve type one failure..")
        self.resolution_pub.publish("resolve type one failure")

        # TODO: drive back to base + recharge

        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_two_failure(self, msg):
        rospy.loginfo("resolve type two failure..")
        self.resolution_pub.publish("resolve type two failure")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

class SensorFailureResolver(GeneralFailureResolver):

    def __init__(self):
        super(SensorFailureResolver, self).__init__()
        rospy.Subscriber('/resolve_sensor_failure', String, self.resolve_callback, queue_size=1)
        self.success_pub = rospy.Publisher('/resolve_sensor_failure_success', Bool, queue_size=1)

    def resolve_callback(self, msg):
        rospy.loginfo("launch sensor failure resolver..")
        rospy.loginfo("type of sensor failure: %s", msg.data)
        rospy.sleep(2)
        self.resolution_pub.publish("launch sensor failure resolver -- type of sensor failure: " + msg.data)
        self.problem_resolved = False

        # different types of resolution are required based on the type of issue
        if msg.data == config.SENSOR_FAILURE_ONE:
            self.resolve_type_one_failure(config.SENSOR_FAILURE_ONE)
        elif msg.data == config.SENSOR_FAILURE_TWO:
            self.resolve_type_two_failure(config.SENSOR_FAILURE_TWO)
        elif msg.data == config.SENSOR_FAILURE_THREE:
            self.resolve_type_three_failure(config.SENSOR_FAILURE_THREE)
        elif msg.data == config.SENSOR_FAILURE_FOUR:
            self.resolve_type_four_failure(config.SENSOR_FAILURE_FOUR)

        if self.problem_resolved:
            self.resolution_pub.publish("problem resolved")
            self.success_pub.publish(True)

    def resolve_type_one_failure(self, msg):
        rospy.loginfo("resolve type one failure..")
        self.resolution_pub.publish("resolve type one failure")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_two_failure(self, msg):
        rospy.loginfo("resolve type two failure..")
        self.resolution_pub.publish("resolve type two failure")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_three_failure(self, msg):
        rospy.loginfo("resolve type three failure..")
        self.resolution_pub.publish("resolve type three failure")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_four_failure(self, msg):
        rospy.loginfo("resolve type four failure..")
        self.resolution_pub.publish("resolve type four failure")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)


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

        # different types of resolution are required based on the type of issue

        if msg.data in [config.LOCALIZATION_FAILURE_ONE, config.LOCALIZATION_FAILURE_TWO, config.LOCALIZATION_FAILURE_THREE, config.LOCALIZATION_FAILURE_FOUR,
            config.LOCALIZATION_FAILURE_FIVE, config.LOCALIZATION_FAILURE_SIX, config.LOCALIZATION_FAILURE_SEVEN, config.LOCALIZATION_FAILURE_EIGHT, config.LOCALIZATION_FAILURE_NINE,
            config.LOCALIZATION_FAILURE_TEN, config.LOCALIZATION_FAILURE_ELEVEN, config.LOCALIZATION_FAILURE_TWELVE, config.LOCALIZATION_FAILURE_THIRTEEN]:

            self.resolve_localization_failure()

        if self.problem_resolved:
            self.resolution_pub.publish("problem resolved")
            self.success_pub.publish(True)

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

        rospy.loginfo("clearing local costmap..")
        self.resolution_pub.publish("clearing local costmap")
        rec_client = actionlib.SimpleActionClient("move_base_flex/recovery", RecoveryAction)
        rec_client.wait_for_server()

        # concurrency_slot 3
        clear_local_costmap_goal = RecoveryGoal('clear_costmap', 3)
        rec_client.send_goal(clear_local_costmap_goal)
        res = rec_client.wait_for_result()
        if res:
            rospy.loginfo("cleared costmap..")

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
        if msg.data == config.PLAN_DEPLOYMENT_FAILURE_ONE:
            self.resolve_type_one_failure(config.PLAN_DEPLOYMENT_FAILURE_ONE)
        elif msg.data == config.PLAN_DEPLOYMENT_FAILURE_TWO:
            self.resolve_type_two_failure(config.PLAN_DEPLOYMENT_FAILURE_TWO)
        elif msg.data == config.PLAN_DEPLOYMENT_FAILURE_THREE:
            self.resolve_type_three_failure(config.PLAN_DEPLOYMENT_FAILURE_THREE)
        elif msg.data == config.PLAN_DEPLOYMENT_FAILURE_FOUR:
            self.resolve_type_four_failure(config.PLAN_DEPLOYMENT_FAILURE_FOUR)
        elif msg.data == config.PLAN_DEPLOYMENT_FAILURE_FIVE:
            self.resolve_type_five_failure(config.PLAN_DEPLOYMENT_FAILURE_FIVE)

        if self.problem_resolved:
            self.resolution_pub.publish("problem resolved")
            self.success_pub.publish(True)

    def resolve_type_one_failure(self, msg):
        rospy.loginfo("resolve type one failure..")
        self.resolution_pub.publish("resolve type one failure")
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
            self.unavailable_service_cnt = 0
            self.fallback_pub.publish(msg)
            while not self.problem_resolved:
                rospy.sleep(5)

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
            self.empty_plan_cnt = 0
            self.fallback_pub.publish(msg)
            while not self.problem_resolved:
                rospy.sleep(5)

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
            self.infeasible_plan_cnt = 0
            self.fallback_pub.publish(msg)
            while not self.problem_resolved:
                rospy.sleep(5)

    def resolve_type_five_failure(self, msg):
        rospy.loginfo("resolve type five failure..")
        self.resolution_pub.publish("resolve type five failure")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

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
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)
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
        else:
            rospy.loginfo("cannot resolve unknown nav failure: %s", msg.data)
            self.resolution_pub.publish("cannot resolve unknown nav failure: " + msg.data)

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

            self.fallback_pub.publish(msg)
            while not self.problem_resolved:
                rospy.sleep(5)
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
        else:
            rospy.loginfo("cannot resolve unknown nav failure: %s", msg.data)

        if self.problem_resolved:
            self.resolution_pub.publish("problem resolved")
            self.success_pub.publish(True)

    def resolve_docking_failure(self):
        rospy.loginfo("resolving docking failure..")
        self.resolution_pub.publish("resolving docking failure")
        self.charge_level_at_resolution = self.latest_charge_level

        # already tried resolution before - call human
        if self.docking_fail_cnt == 1:
            rospy.loginfo("already tried autonomous resolution before -- calling human operator for help..")
            self.resolution_pub.publish("already tried autonomous resolution before -- calling human operator for help")
            self.fallback_pub.publish(config.CHARGING_FAILURE_ONE)
            while not self.problem_resolved:
                rospy.sleep(5)
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
            self.fallback_pub.publish(config.CHARGING_FAILURE_TWO)
            while not self.problem_resolved:
                rospy.sleep(5)
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
            self.fallback_pub.publish(config.CHARGING_FAILURE_THREE)
            while not self.problem_resolved:
                rospy.sleep(5)
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
    rospy.wait_for_message('SMACH_runnning', String)
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
