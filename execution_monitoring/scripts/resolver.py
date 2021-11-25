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
        self.fallback_sub = rospy.Subscriber('/request_fallback', String, self.request_fallback, queue_size=1)
        self.human_operator_sub = rospy.Subscriber('/problem_solved', String, self.solved_by_human_callback, queue_size=1)
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
        self.fallback_sub = rospy.Subscriber("/fallback_success", Bool, self.fallback_callback, queue_size=1)
        self.fallback_pub = rospy.Publisher('/request_fallback', String, queue_size=1)
        self.problem_resolved = False

    def fallback_callback(self, msg):
        if msg.data:
            rospy.loginfo("solved by fallback resolver: %s", msg.data)
            self.problem_resolved = True
        else:
            rospy.loginfo("fallback solution was not successful..")


class DataManagementFailureResolver(GeneralFailureResolver):

    def __init__(self):
        super(DataManagementFailureResolver, self).__init__()
        self.resolve_sub = rospy.Subscriber('/resolve_data_management_failure', String, self.resolve_callback, queue_size=1)
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


class WiFiFailureResolver(GeneralFailureResolver):

    def __init__(self):
        super(WiFiFailureResolver, self).__init__()
        self.resolve_sub = rospy.Subscriber('/resolve_wifi_failure', String, self.resolve_callback, queue_size=1)
        self.success_pub = rospy.Publisher('/resolve_wifi_failure_success', Bool, queue_size=1)

    def resolve_callback(self, msg):
        rospy.loginfo("launch wifi failure resolver..")
        rospy.loginfo("type of wifi failure: %s", msg.data)
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


class SensorFailureResolver(GeneralFailureResolver):

    def __init__(self):
        super(SensorFailureResolver, self).__init__()
        self.resolve_sub = rospy.Subscriber('/resolve_sensor_failure', String, self.resolve_callback, queue_size=1)
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
    WiFiFailureResolver()
    DataManagementFailureResolver()
    FallbackResolver()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
