#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @author Tim Bohne

import rospy
import smach
import smach_ros
from std_msgs.msg import String, Bool

from execution_monitoring import config
from operation import OperationStateMachine


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
        Aggravation callback - problem couldn't be resolved by the robot.

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
        Execution of 'CONTINGENCY' state - initiation of problem resolution based on the reason for the interruption.

        @param userdata: input of state
        @return: outcome of the state ("solved" or "aggravated")
        """
        rospy.loginfo("executing CONTINGENCY state..")
        rospy.loginfo("reason for contingency: %s", self.interrupt_reason)
        self.robot_info_pub.publish("executing CONTINGENCY state -- reason for contingency: " + self.interrupt_reason)
        rospy.sleep(1)
        self.successfully_resolved = False

        if self.interrupt_reason in config.SENSOR_FAILURES.values():
            self.sensor_failure_resolver_pub.publish(self.interrupt_reason)
        elif self.interrupt_reason in config.WIFI_FAILURES.values():
            self.wifi_failure_resolver_pub.publish(self.interrupt_reason)
        elif self.interrupt_reason in config.INTERNET_FAILURES.values():
            self.internet_failure_resolver_pub.publish(self.interrupt_reason)
        elif self.interrupt_reason in config.GNSS_FAILURES.values():
            self.gnss_failure_resolver_pub.publish(self.interrupt_reason)
        elif self.interrupt_reason in config.DATA_MANAGEMENT_FAILURES.values():
            self.data_failure_resolver_pub.publish(self.interrupt_reason)
        elif self.interrupt_reason in config.WEATHER_FAILURES.values():
            self.weather_failure_resolver_pub.publish(self.interrupt_reason)
        elif self.interrupt_reason in config.LOCALIZATION_FAILURES.values():
            self.localization_failure_resolver_pub.publish(self.interrupt_reason)
        elif self.interrupt_reason in config.PLAN_DEPLOYMENT_FAILURES.values():
            self.plan_failure_resolver_pub.publish(self.interrupt_reason)
        elif self.interrupt_reason in config.NAVIGATION_FAILURES.values():
            self.navigation_failure_resolver_pub.publish(self.interrupt_reason)
        elif self.interrupt_reason in config.CHARGING_FAILURES.values():
            self.charging_failure_resolver_pub.publish(self.interrupt_reason)
        elif self.interrupt_reason in config.POWER_MANAGEMENT_FAILURES.values():
            self.power_failure_resolver_pub.publish(self.interrupt_reason)
        else:
            rospy.loginfo("unknown interrupt reason: %s", self.interrupt_reason)
            self.robot_info_pub.publish("unknown interrupt reason: " + self.interrupt_reason)

        while not self.successfully_resolved and not self.aggravate:
            # waiting for resolution / aggravation
            rospy.sleep(5)

        if self.successfully_resolved:
            rospy.loginfo("contingency solved, continuing normal operation..")
            self.robot_info_pub.publish("contingency solved, continuing normal operation")
            return "solved"
        elif self.aggravate:
            rospy.loginfo("issue couldn't be handled / something went wrong")
            self.robot_info_pub.publish("issue couldn't be handled / something went wrong")
            return "aggravated"


class Catastrophe(smach.State):
    """
    State in the high-level SMACH that represents situations in which the robot recognizes a problem, is unable to
    solve it, and calls an operator for help.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['damage_control_performed'])
        self.interrupt_reason = ""
        self.successfully_resolved = False

        self.robot_info_pub = rospy.Publisher('/robot_info', String, queue_size=1)
        self.power_failure_resolver_pub = rospy.Publisher('/resolve_power_management_failure', String, queue_size=1)
        self.sensor_failure_resolver_pub = rospy.Publisher('/resolve_sensor_failure', String, queue_size=1)
        self.internet_failure_resolver_pub = rospy.Publisher('/resolve_internet_failure', String, queue_size=1)
        self.data_failure_resolver_pub = rospy.Publisher('/resolve_data_management_failure', String, queue_size=1)
        self.weather_failure_resolver_pub = rospy.Publisher('/resolve_weather_failure', String, queue_size=1)
        self.localization_failure_resolver_pub = rospy.Publisher('/resolve_localization_failure', String, queue_size=1)
        self.plan_failure_resolver_pub = rospy.Publisher('/resolve_plan_deployment_failure', String, queue_size=1)
        self.navigation_failure_resolver_pub = rospy.Publisher('/resolve_navigation_failure', String, queue_size=1)
        self.charging_failure_resolver_pub = rospy.Publisher('/resolve_charging_failure', String, queue_size=1)

        rospy.Subscriber('/interrupt_reason', String, self.interrupt_reason_callback, queue_size=1)
        rospy.Subscriber('/resolve_sensor_failure_success', Bool, self.resolve_callback, queue_size=1)
        rospy.Subscriber('/resolve_connection_failure_success', Bool, self.resolve_callback, queue_size=1)
        rospy.Subscriber('/resolve_data_management_failure_success', Bool, self.resolve_callback, queue_size=1)
        rospy.Subscriber('/resolve_weather_failure_success', Bool, self.resolve_callback, queue_size=1)
        rospy.Subscriber('/resolve_localization_failure_success', Bool, self.resolve_callback, queue_size=1)
        rospy.Subscriber('/resolve_plan_deployment_failure_success', Bool, self.resolve_callback, queue_size=1)
        rospy.Subscriber('/resolve_navigation_failure_success', Bool, self.resolve_callback, queue_size=1)
        rospy.Subscriber('/resolve_charging_failure_success', Bool, self.resolve_callback, queue_size=1)
        rospy.Subscriber('/resolve_power_management_failure_success', Bool, self.resolve_callback, queue_size=1)

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
        Execution of 'CATASTROPHE' state - initiation of problem handling based on the reason for the interruption.

        @param userdata: input of state
        @return: outcome of the state ("damage_control_performed")
        """
        rospy.loginfo("executing CATASTROPHE state..")
        rospy.loginfo("reason for catastrophe: %s", self.interrupt_reason)
        self.robot_info_pub.publish("executing CATASTROPHE state -- reason for catastrophe: " + self.interrupt_reason)

        if self.interrupt_reason == config.POWER_MANAGEMENT_CATA:
            self.power_failure_resolver_pub.publish(self.interrupt_reason)
        elif self.interrupt_reason == config.NAV_CATA:
            self.navigation_failure_resolver_pub.publish(self.interrupt_reason)
        elif self.interrupt_reason == config.CHARGING_CATA:
            self.charging_failure_resolver_pub.publish(self.interrupt_reason)
        elif self.interrupt_reason == config.PLAN_DEPLOYMENT_CATA:
            self.plan_failure_resolver_pub.publish(self.interrupt_reason)
        elif self.interrupt_reason == config.DATA_MANAGEMENT_CATA:
            self.data_failure_resolver_pub.publish(self.interrupt_reason)
        elif self.interrupt_reason == config.SENSOR_CATA:
            self.sensor_failure_resolver_pub.publish(self.interrupt_reason)
        elif self.interrupt_reason == config.LOCALIZATION_CATA:
            self.localization_failure_resolver_pub.publish(self.interrupt_reason)
        elif self.interrupt_reason == config.WEATHER_CATA:
            self.weather_failure_resolver_pub.publish(self.interrupt_reason)
        elif self.interrupt_reason == config.CONNECTION_CATA:
            self.internet_failure_resolver_pub.publish(self.interrupt_reason)
        else:
            rospy.loginfo("unknown reason for catastrophe: %s", self.interrupt_reason)

        while not self.successfully_resolved:
            # waiting for handling of catastrophe situation
            rospy.sleep(5)

        rospy.loginfo("catastrophe processed, shutting down..")
        self.robot_info_pub.publish("catastrophe processed, shutting down")
        return "damage_control_performed"


class Shutdown(smach.State):
    """
    State in the high-level SMACH representing the end of an LTA episode in which the system is shut down.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['data_management_completed'])
        self.pub = rospy.Publisher('/arox/shutdown_trigger', String, queue_size=1)
        self.robot_info_pub = rospy.Publisher('/robot_info', String, queue_size=1)

    def execute(self, userdata):
        """
        Execution of 'SHUTDOWN' state - initiation of system deactivation.

        @param userdata: input of state
        @return: outcome of the state ("data_management_completed")
        """
        rospy.loginfo("executing SHUTDOWN state..")
        rospy.loginfo("system shuts down..")
        self.robot_info_pub.publish("executing SHUTDOWN state -- system shuts down")
        self.pub.publish("shutdown")
        return "data_management_completed"


class ExecutionMonitoringStateMachine(smach.StateMachine):
    """
    High-level hierarchically structured state machine responsible for plan execution as well as robot operation
    monitoring.
    """

    def __init__(self):
        super(ExecutionMonitoringStateMachine, self).__init__(
            outcomes=['system_deactivated'],
            input_keys=[],
            output_keys=[]
        )
        self.interrupt_reason_pub = rospy.Publisher('/interrupt_reason', String, queue_size=1)
        self.interrupt_active_goals_pub = rospy.Publisher('/interrupt_active_goals', String, queue_size=1)

        # concurrence container that enables parallel running of operation and monitoring
        operation_monitoring = smach.Concurrence(
            outcomes=['minor_complication', 'critical_complication', 'end_of_episode'],
            default_outcome='end_of_episode',
            child_termination_cb=self.child_termination_callback,
            outcome_cb=self.out_callback
        )
        # defines the three components running in parallel (low-level SMACH + two monitoring components)
        with operation_monitoring:
            smach.Concurrence.add('OPERATION', OperationStateMachine())
            smach.Concurrence.add('CONTINGENCY_MONITORING',
                                  smach_ros.MonitorState("/contingency_preemption", String, self.monitoring_callback))
            smach.Concurrence.add('CATASTROPHE_MONITORING',
                                  smach_ros.MonitorState("/catastrophe_preemption", String, self.monitoring_callback))

        # defines states and transitions of the high-level execution monitoring SMACH
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

    @staticmethod
    def child_termination_callback(outcome_map):
        """
        Called whenever one of the child states terminates.
        (policy for determination of overall outcome based on child outcomes)

        @param outcome_map: dictionary with parallel running components as keys and outcomes as values
        @return: False: state machine should keep running
                 True:  preempt all remaining running states
        """
        if outcome_map['OPERATION'] in ['minor_complication', 'critical_complication', 'end_of_episode', 'preempted']:
            return True
        elif outcome_map['CONTINGENCY_MONITORING'] == 'invalid' or outcome_map['CATASTROPHE_MONITORING'] == 'invalid':
            return True
        else:
            return False

    @staticmethod
    def out_callback(outcome_map):
        """
        Called once when the last child state terminates.

        @param outcome_map: dictionary with parallel running components as keys and outcomes as values
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
        Called in contingency and catastrophe preemption situations.
        Needs to return False when we want the monitoring state to terminate.

        @param userdata: SMACH userdata
        @param msg: reason for interruption
        @return: False such that the monitoring state terminates
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

    outcome = sm.execute()
    rospy.loginfo("outcome: %s", outcome)
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
