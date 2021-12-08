#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Bool
from execution_monitoring import config


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
        self.success_pub = rospy.Publisher('/resolve_weather_failure_success', Bool, queue_size=1)

    def resolve_callback(self, msg):
        rospy.loginfo("launch weather failure resolver..")
        rospy.loginfo("type of weather failure: %s", msg.data)
        self.problem_resolved = False

        # all these failures are resolved by seeking shelter and waiting
        if msg.data in [config.WEATHER_FAILURE_TWO, config.WEATHER_FAILURE_THREE, config.WEATHER_FAILURE_FIVE, config.WEATHER_FAILURE_EIGHT, config.WEATHER_FAILURE_NINE,
            config.WEATHER_FAILURE_TEN, config.WEATHER_FAILURE_ELEVEN, config.WEATHER_FAILURE_TWELVE, config.WEATHER_FAILURE_THIRTEEN, config.WEATHER_FAILURE_FOURTEEN,
            config.WEATHER_FAILURE_FIFTEEN, config.WEATHER_FAILURE_SIXTEEN, config.WEATHER_FAILURE_SEVENTEEN, config.WEATHER_FAILURE_EIGHTEEN]:

            self.resolve_weather_failure(msg.data)

        if self.problem_resolved:
            self.success_pub.publish(True)

    def resolve_weather_failure(self, msg):
        rospy.loginfo("resolve drastic weather change..")
        # TODO: drive back to base
        self.fallback_pub.publish(msg)
        while not self.problem_resolved:
            rospy.sleep(5)

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


def node():
    rospy.init_node('failure_resolver')
    rospy.wait_for_message('SMACH_runnning', String)
    SensorFailureResolver()
    ConnectionResolver()
    WeatherFailureResolver()
    DataManagementFailureResolver()
    FallbackResolver()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
