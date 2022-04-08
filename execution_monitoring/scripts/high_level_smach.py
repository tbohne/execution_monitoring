#!/usr/bin/env python
import rospy
import smach
import smach_ros
from std_msgs.msg import String, Bool, Float64
from operation import OperationStateMachine
from execution_monitoring import config


class Contingency(smach.State):
    """
    State in the high-level SMACH that represents situations in which the robot recognizes a problem and is able to
    solve it by itself.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['solved', 'aggravated'])
        self.interrupt_reason = ""
        self.successfully_resolved = False
        self.aggravate = False

        self.sensor_failure_resolver_pub = rospy.Publisher('/resolve_sensor_failure', String, queue_size=1)
        self.wifi_failure_resolver_pub = rospy.Publisher('/resolve_wifi_failure', String, queue_size=1)
        self.internet_failure_resolver_pub = rospy.Publisher('/resolve_internet_failure', String, queue_size=1)
        self.gnss_failure_resolver_pub = rospy.Publisher('/resolve_gnss_failure', String, queue_size=1)
        self.data_failure_resolver_pub = rospy.Publisher('/resolve_data_management_failure', String, queue_size=1)
        self.weather_failure_resolver_pub = rospy.Publisher('/resolve_weather_failure', String, queue_size=1)
        self.localization_failure_resolver_pub = rospy.Publisher('/resolve_localization_failure', String, queue_size=1)
        self.plan_failure_resolver_pub = rospy.Publisher('/resolve_plan_deployment_failure', String, queue_size=1)
        self.navigation_failure_resolver_pub = rospy.Publisher('/resolve_navigation_failure', String, queue_size=1)
        self.charging_failure_resolver_pub = rospy.Publisher('/resolve_charging_failure', String, queue_size=1)
        self.power_failure_resolver_pub = rospy.Publisher('/resolve_power_management_failure', String, queue_size=1)
        self.robot_info_pub = rospy.Publisher('/robot_info', String, queue_size=1)

        rospy.Subscriber('/interrupt_reason', String, self.interrupt_reason_callback, queue_size=1)
        rospy.Subscriber('/aggravate', String, self.aggravate_callback, queue_size=1)
        rospy.Subscriber('/resolve_sensor_failure_success', Bool, self.resolve_callback, queue_size=1)
        rospy.Subscriber('/resolve_connection_failure_success', Bool, self.resolve_callback, queue_size=1)
        rospy.Subscriber('/resolve_data_management_failure_success', Bool, self.resolve_callback, queue_size=1)
        rospy.Subscriber('/resolve_weather_failure_success', Bool, self.resolve_callback, queue_size=1)
        rospy.Subscriber('/resolve_localization_failure_success', Bool, self.resolve_callback, queue_size=1)
        rospy.Subscriber('/resolve_plan_deployment_failure_success', Bool, self.resolve_callback, queue_size=1)
        rospy.Subscriber('/resolve_navigation_failure_success', Bool, self.resolve_callback, queue_size=1)
        rospy.Subscriber('/resolve_charging_failure_success', Bool, self.resolve_callback, queue_size=1)
        rospy.Subscriber('/resolve_power_management_failure_success', Bool, self.resolve_callback, queue_size=1)

    def aggravate_callback(self, msg):
        """
        Aggravation callback - problem couldn't be resolved.

        @param msg: callback message
        """
        self.aggravate = True

    def interrupt_reason_callback(self, reason):
        """
        Communicates the reason of an interruption.

        @param reason: callback message - reason for interruption
        """
        self.interrupt_reason = reason.data

    def resolve_callback(self, res):
        """
        Resolution method communicates result of resolution, i.e., whether it was successful.

        @param res: callback message - whether resolution was successful
        """
        self.successfully_resolved = res.data

    def execute(self, userdata):
        """
        Execution of 'CONTINGENCY' state - initiation of problem resolution based on reason for interruption.

        @param userdata: input of state
        """
        rospy.loginfo("executing CONTINGENCY state..")
        rospy.loginfo("reason for contingency: %s", self.interrupt_reason)
        self.robot_info_pub.publish("executing CONTINGENCY state -- reason for contingency: " + self.interrupt_reason)
        rospy.sleep(1)
        self.successfully_resolved = False

        if self.interrupt_reason in config.SENSOR_FAILURES.values():
            self.sensor_failure_resolver_pub.publish(self.interrupt_reason)
        if self.interrupt_reason in config.WIFI_FAILURES.values():
            self.wifi_failure_resolver_pub.publish(self.interrupt_reason)
        if self.interrupt_reason in config.INTERNET_FAILURES.values():
            self.internet_failure_resolver_pub.publish(self.interrupt_reason)
        if self.interrupt_reason in config.GNSS_FAILURES.values():
            self.gnss_failure_resolver_pub.publish(self.interrupt_reason)
        if self.interrupt_reason in config.DATA_MANAGEMENT_FAILURES.values():
            self.data_failure_resolver_pub.publish(self.interrupt_reason)
        if self.interrupt_reason in config.WEATHER_FAILURES.values():
            self.weather_failure_resolver_pub.publish(self.interrupt_reason)
        if self.interrupt_reason in config.LOCALIZATION_FAILURES.values():
            self.localization_failure_resolver_pub.publish(self.interrupt_reason)
        if self.interrupt_reason in config.PLAN_DEPLOYMENT_FAILURES.values():
            self.plan_failure_resolver_pub.publish(self.interrupt_reason)


        elif self.interrupt_reason == config.NAV_FAILURE_ONE:
            self.navigation_failure_resolver_pub.publish(config.NAV_FAILURE_ONE)
        elif self.interrupt_reason == config.NAV_FAILURE_THREE:
            self.navigation_failure_resolver_pub.publish(config.NAV_FAILURE_THREE)
        elif self.interrupt_reason == config.CHARGING_FAILURE_ONE:
            self.charging_failure_resolver_pub.publish(config.CHARGING_FAILURE_ONE)
        elif self.interrupt_reason == config.CHARGING_FAILURE_TWO:
            self.charging_failure_resolver_pub.publish(config.CHARGING_FAILURE_TWO)
        elif self.interrupt_reason == config.CHARGING_FAILURE_THREE:
            self.charging_failure_resolver_pub.publish(config.CHARGING_FAILURE_THREE)
        elif self.interrupt_reason == config.POWER_MANAGEMENT_FAILURE_ONE:
            self.power_failure_resolver_pub.publish(config.POWER_MANAGEMENT_FAILURE_ONE)
        else:
            rospy.loginfo("unkonwn interrupt reason: %s", self.interrupt_reason)
            self.robot_info_pub.publish("unkonwn interrupt reason: " + self.interrupt_reason)

        while not self.successfully_resolved and not self.aggravate:
            rospy.sleep(5)

        if self.successfully_resolved:
            rospy.loginfo("contingency solved, continuing normal operation..")
            self.robot_info_pub.publish("contingency solved, continuing normal operation")
            return "solved"

        if self.aggravate:
            rospy.loginfo("issue couldn't be handled / something went wrong")
            self.robot_info_pub.publish("issue couldn't be handled / something went wrong")
            return "aggravated"


class Catastrophe(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['damage_control_performed'])
        self.interrupt_reason = ""
        self.successfully_resolved = False
        rospy.Subscriber('/interrupt_reason', String, self.interrupt_reason_callback, queue_size=1)
        rospy.Subscriber('/resolve_sensor_failure_success', Bool, self.resolve_failure_success_callback, queue_size=1)
        rospy.Subscriber('/resolve_connection_failure_success', Bool, self.resolve_failure_success_callback,
                         queue_size=1)
        rospy.Subscriber('/resolve_data_management_failure_success', Bool, self.resolve_failure_success_callback,
                         queue_size=1)
        rospy.Subscriber('/resolve_weather_failure_success', Bool, self.resolve_failure_success_callback, queue_size=1)
        rospy.Subscriber('/resolve_localization_failure_success', Bool, self.resolve_failure_success_callback,
                         queue_size=1)
        rospy.Subscriber('/resolve_plan_deployment_failure_success', Bool, self.resolve_failure_success_callback,
                         queue_size=1)
        rospy.Subscriber('/resolve_navigation_failure_success', Bool, self.resolve_failure_success_callback,
                         queue_size=1)
        rospy.Subscriber('/resolve_charging_failure_success', Bool, self.resolve_failure_success_callback, queue_size=1)
        rospy.Subscriber('/resolve_power_management_failure_success', Bool, self.resolve_failure_success_callback,
                         queue_size=1)
        self.robot_info_pub = rospy.Publisher('/robot_info', String, queue_size=1)
        self.power_management_failure_resolver_pub = rospy.Publisher('/resolve_power_management_failure', String,
                                                                     queue_size=1)
        self.sensor_failure_resolver_pub = rospy.Publisher('/resolve_sensor_failure', String, queue_size=1)
        self.internet_failure_resolver_pub = rospy.Publisher('/resolve_internet_failure', String, queue_size=1)
        self.data_management_failure_resolver_pub = rospy.Publisher('/resolve_data_management_failure', String,
                                                                    queue_size=1)
        self.weather_failure_resolver_pub = rospy.Publisher('/resolve_weather_failure', String, queue_size=1)
        self.localization_failure_resolver_pub = rospy.Publisher('/resolve_localization_failure', String, queue_size=1)
        self.plan_deployment_failure_resolver_pub = rospy.Publisher('/resolve_plan_deployment_failure', String,
                                                                    queue_size=1)
        self.navigation_failure_resolver_pub = rospy.Publisher('/resolve_navigation_failure', String, queue_size=1)
        self.charging_failure_resolver_pub = rospy.Publisher('/resolve_charging_failure', String, queue_size=1)

    def interrupt_reason_callback(self, reason):
        self.interrupt_reason = reason.data

    def resolve_failure_success_callback(self, res):
        self.successfully_resolved = res.data

    def execute(self, userdata):
        rospy.loginfo("executing CATASTROPHE state..")
        rospy.loginfo("reason for catastrophe: %s", self.interrupt_reason)
        self.robot_info_pub.publish("executing CATASTROPHE state -- reason for catastrophe: " + self.interrupt_reason)

        # do everything that is still possible:
        #   - communicate problem
        #   - save state
        #   - ...
        if self.interrupt_reason == config.POWER_MANAGEMENT_CATA:
            self.power_management_failure_resolver_pub.publish(config.POWER_MANAGEMENT_CATA)
        elif self.interrupt_reason == config.NAV_CATA:
            self.navigation_failure_resolver_pub.publish(config.NAV_CATA)
        elif self.interrupt_reason == config.CHARGING_CATA:
            self.charging_failure_resolver_pub.publish(config.CHARGING_CATA)
        elif self.interrupt_reason == config.PLAN_DEPLOYMENT_CATA:
            self.plan_deployment_failure_resolver_pub.publish(config.PLAN_DEPLOYMENT_CATA)
        elif self.interrupt_reason == config.DATA_MANAGEMENT_CATA:
            self.data_management_failure_resolver_pub.publish(config.DATA_MANAGEMENT_CATA)
        elif self.interrupt_reason == config.SENSOR_CATA:
            self.sensor_failure_resolver_pub.publish(config.SENSOR_CATA)
        elif self.interrupt_reason == config.LOCALIZATION_CATA:
            self.localization_failure_resolver_pub.publish(config.LOCALIZATION_CATA)
        elif self.interrupt_reason == config.WEATHER_CATA:
            self.weather_failure_resolver_pub.publish(config.WEATHER_CATA)
        elif self.interrupt_reason == config.CONNECTION_CATA:
            self.internet_failure_resolver_pub.publish(config.CONNECTION_CATA)

        while not self.successfully_resolved:
            rospy.sleep(5)

        rospy.loginfo("catastrophe processed, shutting down..")
        self.robot_info_pub.publish("catastrophe processed, shutting down")
        return "damage_control_performed"


class Shutdown(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['data_management_completed'])
        self.pub = rospy.Publisher('/arox/shutdown_trigger', String, queue_size=1)
        self.robot_info_pub = rospy.Publisher('/robot_info', String, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo("executing SHUTDOWN state..")
        rospy.loginfo("system shuts down..")
        self.robot_info_pub.publish("executing SHUTDOWN state -- system shuts down")
        self.pub.publish("shutdown")
        return "data_management_completed"


class ExecutionMonitoringStateMachine(smach.StateMachine):

    def __init__(self):
        super(ExecutionMonitoringStateMachine, self).__init__(
            outcomes=['system_deactivated'],
            input_keys=[],
            output_keys=[]
        )

        self.interrupt_reason_pub = rospy.Publisher('/interrupt_reason', String, queue_size=1)
        self.interrupt_active_goals_pub = rospy.Publisher('/interrupt_active_goals', String, queue_size=1)

        operation_monitoring = smach.Concurrence(
            outcomes=['minor_complication', 'critical_complication', 'end_of_episode'],
            default_outcome='end_of_episode',
            child_termination_cb=self.child_termination_callback,
            outcome_cb=self.out_callback)

        with operation_monitoring:
            smach.Concurrence.add('OPERATION', OperationStateMachine())
            smach.Concurrence.add('CONTINGENCY_MONITORING',
                                  smach_ros.MonitorState("/contingency_preemption", String, self.monitoring_callback))
            smach.Concurrence.add('CATASTROPHE_MONITORING',
                                  smach_ros.MonitorState("/catastrophe_preemption", String, self.monitoring_callback))

        with self:
            self.add('NORMAL_OPERATION', operation_monitoring,
                     transitions={'minor_complication': 'CONTINGENCY',
                                  'critical_complication': 'CATASTROPHE',
                                  'end_of_episode': 'SHUTDOWN'})

            self.add('CONTINGENCY', Contingency(),
                     transitions={'solved': 'NORMAL_OPERATION',
                                  'aggravated': 'CATASTROPHE'})

            self.add('CATASTROPHE', Catastrophe(),
                     transitions={'damage_control_performed': 'SHUTDOWN'})

            self.add('SHUTDOWN', Shutdown(),
                     transitions={'data_management_completed': 'system_deactivated'})

    def child_termination_callback(self, outcome_map):
        """
        Called every time one of the child states terminates.

        @return False: state machine should keep running
                True:  preempt all remaining running states
        """
        if outcome_map['OPERATION'] in ['minor_complication', 'critical_complication', 'end_of_episode', 'preempted']:
            return True
        elif outcome_map['CONTINGENCY_MONITORING'] == 'invalid' or outcome_map['CATASTROPHE_MONITORING'] == 'invalid':
            return True
        else:
            return False

    def out_callback(self, outcome_map):
        """
        Called once when the last child state terminates.

        @return outcome of the concurrence state machine
        """
        if outcome_map['OPERATION'] == 'minor_complication' or outcome_map['CONTINGENCY_MONITORING'] == 'invalid':
            return 'minor_complication'
        elif outcome_map['OPERATION'] == 'critical_complication' or outcome_map['CATASTROPHE_MONITORING'] == 'invalid':
            return 'critical_complication'
        elif outcome_map['OPERATION'] == 'end_of_episode':
            return 'end_of_episode'

    def monitoring_callback(self, userdata, msg):
        """
        Needs to return False when we want the monitoring state to terminate.
        """
        self.interrupt_reason_pub.publish(msg)
        self.interrupt_active_goals_pub.publish("interrupt all running goals..")
        return False


def node():
    """
    High-level execution monitoring node.
    """
    rospy.init_node('execution_monitoring')

    sm = ExecutionMonitoringStateMachine()
    sis = smach_ros.IntrospectionServer('execution_monitoring', sm, '/EXECUTION_MONITORING')
    sis.start()

    # wait to start and notify corresponding nodes
    for _ in range(3):
        pub = rospy.Publisher('SMACH_running', String, queue_size=1)
        pub.publish("execution monitoring SMACH runs")
        rospy.sleep(1)

    # # if the docking is activated - open container front
    # if config.DOCKING:
    #     rospy.loginfo("sending command to open container front..")
    #     container_pub = rospy.Publisher('/container/rampB_position_controller/command', Float64, queue_size=1)
    #     for _ in range(3):
    #         container_pub.publish(2.0)
    #         rospy.sleep(0.5)

    outcome = sm.execute()
    rospy.loginfo("outcome: %s", outcome)
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
