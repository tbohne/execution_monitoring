#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Bool
import actionlib
from arox_navigation_flex.msg import drive_to_goalAction
from execution_monitoring import config, util
from geometry_msgs.msg import Twist
from arox_performance_parameters.msg import arox_operational_param
from mbf_msgs.msg import RecoveryAction, RecoveryGoal


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
        self.problem_resolved = False

    def request_fallback(self, msg):
        rospy.loginfo("fallback requested.. communicating problem to human operator..")
        rospy.loginfo("problem: %s", msg.data)
        self.problem_resolved = False
        self.human_operator_contingency_pub.publish(msg.data)

        # TODO: when it takes too long it should go to CATASTROPHE
        while not self.problem_resolved:
            rospy.loginfo("waiting for human operator to solve the problem..")
            rospy.sleep(5)
        
        rospy.loginfo("sensor failure resolved..")
        self.fallback_pub.publish(True)

    def solved_by_human_callback(self, msg):
        rospy.loginfo("solved by human operator: %s", msg.data)
        self.problem_resolved = True


class GeneralFailureResolver(object):

    def __init__(self):
        rospy.Subscriber("/fallback_success", Bool, self.fallback_callback, queue_size=1)
        self.fallback_pub = rospy.Publisher('/request_fallback', String, queue_size=1)
        self.problem_resolved = False

    def fallback_callback(self, msg):
        if msg.data:
            rospy.loginfo("solved by fallback resolver: %s", msg.data)
            self.problem_resolved = True
        else:
            rospy.loginfo("fallback solution was not successful..")

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
        rospy.loginfo("resolve drastic weather change..")
        rospy.loginfo("seeking shelter - driving back to base..")

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
        rospy.set_param("charging_mode", True)
        charge_mode = rospy.get_param("charging_mode")

        while charge_mode:
            charge_mode = rospy.get_param("charging_mode")
            rospy.loginfo("charging battery..")
            rospy.sleep(2)

        if not charge_mode:
            rospy.loginfo("battery charged..")

        rospy.loginfo("waiting until weather is moderate again")
        while not self.moderate_weather:
            rospy.sleep(5)

        self.problem_resolved = True


class DataManagementFailureResolver(GeneralFailureResolver):

    def __init__(self):
        super(DataManagementFailureResolver, self).__init__()
        rospy.Subscriber('/resolve_data_management_failure', String, self.resolve_callback, queue_size=1)
        self.success_pub = rospy.Publisher('/resolve_data_management_failure_success', Bool, queue_size=1)

    def resolve_callback(self, msg):
        rospy.loginfo("launch data management failure resolver..")
        rospy.loginfo("type of data management failure: %s", msg.data)
        self.problem_resolved = False

        # different types of resolution are required based on the type of issue
        if msg.data == config.DATA_MANAGEMENT_FAILURE_ONE:
            self.resolve_type_one_failure(config.DATA_MANAGEMENT_FAILURE_ONE)
        elif msg.data == config.DATA_MANAGEMENT_FAILURE_TWO:
            self.resolve_type_two_failure(config.DATA_MANAGEMENT_FAILURE_TWO)

        if self.problem_resolved:
            self.success_pub.publish(True)

    def resolve_type_one_failure(self, msg):
        rospy.loginfo("resolve type one failure..")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_two_failure(self, msg):
        rospy.loginfo("resolve type two failure..")
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
            self.success_pub.publish(True)

    def resolve_type_one_failure(self, msg):
        rospy.loginfo("resolve type one failure..")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_two_failure(self, msg):
        rospy.loginfo("resolve type two failure..")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_three_failure(self, msg):
        rospy.loginfo("resolve type three failure..")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_four_failure(self, msg):
        rospy.loginfo("resolve type four failure..")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_five_failure(self, msg):
        rospy.loginfo("resolve type five failure..")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)
        # try to re-initialize the internet monitor (requires connection)
        self.re_init_pub.publish("")

    def resolve_type_six_failure(self, msg):
        rospy.loginfo("resolve type six failure..")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_seven_failure(self, msg):
        rospy.loginfo("resolve type seven failure..")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_eight_failure(self, msg):
        rospy.loginfo("resolve type eight failure..")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_nine_failure(self, msg):
        rospy.loginfo("resolve type nine failure..")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_ten_failure(self, msg):
        rospy.loginfo("resolve type ten failure..")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_eleven_failure(self, msg):
        rospy.loginfo("resolve type eleven failure..")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_twelve_failure(self, msg):
        rospy.loginfo("resolve type twelve failure..")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_thirteen_failure(self, msg):
        rospy.loginfo("resolve type thirteen failure..")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_fourteen_failure(self, msg):
        rospy.loginfo("resolve type fourteen failure..")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)
        
    def resolve_type_fifteen_failure(self, msg):
        rospy.loginfo("resolve type fifteen failure..")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_sixteen_failure(self, msg):
        rospy.loginfo("resolve type sixteen failure..")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_seventeen_failure(self, msg):
        rospy.loginfo("resolve type seventeen failure..")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_eighteen_failure(self, msg):
        rospy.loginfo("resolve type eighteen failure..")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_nineteen_failure(self, msg):
        rospy.loginfo("resolve type nineteen failure..")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_twenty_failure(self, msg):
        rospy.loginfo("resolve type twenty failure..")
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
            self.success_pub.publish(True)

    def resolve_type_one_failure(self, msg):
        rospy.loginfo("resolve type one failure..")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_two_failure(self, msg):
        rospy.loginfo("resolve type two failure..")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_three_failure(self, msg):
        rospy.loginfo("resolve type three failure..")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_four_failure(self, msg):
        rospy.loginfo("resolve type four failure..")
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
        self.problem_resolved = False

        # different types of resolution are required based on the type of issue

        if msg.data in [config.LOCALIZATION_FAILURE_ONE, config.LOCALIZATION_FAILURE_TWO, config.LOCALIZATION_FAILURE_THREE, config.LOCALIZATION_FAILURE_FOUR,
            config.LOCALIZATION_FAILURE_FIVE, config.LOCALIZATION_FAILURE_SIX, config.LOCALIZATION_FAILURE_SEVEN, config.LOCALIZATION_FAILURE_EIGHT, config.LOCALIZATION_FAILURE_NINE,
            config.LOCALIZATION_FAILURE_TEN, config.LOCALIZATION_FAILURE_ELEVEN, config.LOCALIZATION_FAILURE_TWELVE, config.LOCALIZATION_FAILURE_THIRTEEN]:

            self.resolve_localization_failure()

        if self.problem_resolved:
            self.success_pub.publish(True)

    def resolve_localization_failure(self):
        rospy.loginfo("resolve localization failure..")
        # sleeping a moment to wait for the robot to stand still
        rospy.sleep(5)
        # TODO: should pay attention to obstacles etc.
        rospy.loginfo("driving the robot a few meters back and forth to recalibrate the localization using different GNSS positions..")
        twist = Twist()
        twist.linear.x = -3.0
        rate = rospy.Rate(4)
        for _ in range(2):
            for _ in range(4):
                self.cmd_vel_pub.publish(twist)
                rate.sleep()
            twist.linear.x = 3.0

        rospy.loginfo("clearing local costmap..")
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
            self.success_pub.publish(True)

    def resolve_type_one_failure(self, msg):
        rospy.loginfo("resolve type one failure..")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

    def resolve_type_two_failure(self, msg):
        rospy.loginfo("resolve type two failure..")
        if self.unavailable_service_cnt == 0:
            rospy.loginfo("resolve by trying to bring up the service..")
            self.activate_plan_service_pub.publish("bring up service")
            self.unavailable_service_cnt += 1
            self.problem_resolved = True
        else:
            rospy.loginfo("plan generation service repeatedly unavailable..")
            self.unavailable_service_cnt = 0
            self.fallback_pub.publish(msg)
            while not self.problem_resolved:
                rospy.sleep(5)

    def resolve_type_three_failure(self, msg):
        rospy.loginfo("resolve type three failure..")
        if self.empty_plan_cnt == 0:
            rospy.loginfo("resolve by trying again..")
            self.empty_plan_cnt += 1
            self.problem_resolved = True
        else:
            rospy.loginfo("plan retrieval failed repeatedly..")
            self.empty_plan_cnt = 0
            self.fallback_pub.publish(msg)
            while not self.problem_resolved:
                rospy.sleep(5)

    def resolve_type_four_failure(self, msg):
        rospy.loginfo("resolve type four failure..")
        if self.infeasible_plan_cnt == 0:
            rospy.loginfo("resolve by trying again..")
            self.infeasible_plan_cnt += 1
            self.problem_resolved = True
        else:
            rospy.loginfo("plan retrieval failed repeatedly..")
            self.infeasible_plan_cnt = 0
            self.fallback_pub.publish(msg)
            while not self.problem_resolved:
                rospy.sleep(5)

    def resolve_type_five_failure(self, msg):
        rospy.loginfo("resolve type five failure..")
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

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
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
