#!/usr/bin/env python
import rospy
import smach
import smach_ros
from std_msgs.msg import String, Bool, Float64
from operation import OperationStateMachine
from actionlib_msgs.msg import GoalID
from execution_monitoring import config


class Contingency(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['solved', 'aggravated'])
        self.interrupt_reason = ""
        self.successfully_resolved = False
        rospy.Subscriber('/interrupt_reason', String, self.interrupt_reason_callback, queue_size=1)
        rospy.Subscriber('/resolve_sensor_failure_success', Bool, self.resolve_failure_success_callback, queue_size=1)
        rospy.Subscriber('/resolve_connection_failure_success', Bool, self.resolve_failure_success_callback, queue_size=1)
        rospy.Subscriber('/resolve_data_management_failure_success', Bool, self.resolve_failure_success_callback, queue_size=1)
        rospy.Subscriber('/resolve_weather_failure_success', Bool, self.resolve_failure_success_callback, queue_size=1)
        rospy.Subscriber('/resolve_localization_failure_success', Bool, self.resolve_failure_success_callback, queue_size=1)
        rospy.Subscriber('/resolve_plan_deployment_failure_success', Bool, self.resolve_failure_success_callback, queue_size=1)
        rospy.Subscriber('/resolve_navigation_failure_success', Bool, self.resolve_failure_success_callback, queue_size=1)
        rospy.Subscriber('/resolve_charging_failure_success', Bool, self.resolve_failure_success_callback, queue_size=1)
        self.sensor_failure_resolver_pub = rospy.Publisher('/resolve_sensor_failure', String, queue_size=1)
        self.wifi_failure_resolver_pub = rospy.Publisher('/resolve_wifi_failure', String, queue_size=1)
        self.internet_failure_resolver_pub = rospy.Publisher('/resolve_internet_failure', String, queue_size=1)
        self.gnss_failure_resolver_pub = rospy.Publisher('/resolve_gnss_failure', String, queue_size=1)
        self.data_management_failure_resolver_pub = rospy.Publisher('/resolve_data_management_failure', String, queue_size=1)
        self.weather_failure_resolver_pub = rospy.Publisher('/resolve_weather_failure', String, queue_size=1)
        self.localization_failure_resolver_pub = rospy.Publisher('/resolve_localization_failure', String, queue_size=1)
        self.plan_deployment_failure_resolver_pub = rospy.Publisher('/resolve_plan_deployment_failure', String, queue_size=1)
        self.navigation_failure_resolver_pub = rospy.Publisher('/resolve_navigation_failure', String, queue_size=1)
        self.charging_failure_resolver_pub = rospy.Publisher('/resolve_charging_failure', String, queue_size=1)

    def interrupt_reason_callback(self, reason):
        self.interrupt_reason = reason.data

    def resolve_failure_success_callback(self, res):
        self.successfully_resolved = res.data

    def execute(self, userdata):
        rospy.loginfo("executing CONTINGENCY state..")
        rospy.loginfo("reason for contingency: %s", self.interrupt_reason)
        self.successfully_resolved = False

        if self.interrupt_reason == config.SENSOR_FAILURE_ONE:
            self.sensor_failure_resolver_pub.publish(config.SENSOR_FAILURE_ONE)
        elif self.interrupt_reason == config.SENSOR_FAILURE_TWO:
            self.sensor_failure_resolver_pub.publish(config.SENSOR_FAILURE_TWO)
        elif self.interrupt_reason == config.SENSOR_FAILURE_THREE:
            self.sensor_failure_resolver_pub.publish(config.SENSOR_FAILURE_THREE)
        elif self.interrupt_reason == config.SENSOR_FAILURE_FOUR:
            self.sensor_failure_resolver_pub.publish(config.SENSOR_FAILURE_FOUR)
        elif self.interrupt_reason == config.CONNECTION_FAILURE_ONE:
            self.wifi_failure_resolver_pub.publish(config.CONNECTION_FAILURE_ONE)
        elif self.interrupt_reason == config.CONNECTION_FAILURE_TWO:
            self.wifi_failure_resolver_pub.publish(config.CONNECTION_FAILURE_TWO)
        elif self.interrupt_reason == config.CONNECTION_FAILURE_THREE:
            self.wifi_failure_resolver_pub.publish(config.CONNECTION_FAILURE_THREE)
        elif self.interrupt_reason == config.CONNECTION_FAILURE_FOUR:
            self.wifi_failure_resolver_pub.publish(config.CONNECTION_FAILURE_FOUR)
        elif self.interrupt_reason == config.DATA_MANAGEMENT_FAILURE_ONE:
            self.data_management_failure_resolver_pub.publish(config.DATA_MANAGEMENT_FAILURE_ONE)
        elif self.interrupt_reason == config.DATA_MANAGEMENT_FAILURE_TWO:
            self.data_management_failure_resolver_pub.publish(config.DATA_MANAGEMENT_FAILURE_TWO)
        elif self.interrupt_reason == config.CONNECTION_FAILURE_FIVE:
            self.internet_failure_resolver_pub.publish(config.CONNECTION_FAILURE_FIVE)
        elif self.interrupt_reason == config.CONNECTION_FAILURE_SIX:
            self.internet_failure_resolver_pub.publish(config.CONNECTION_FAILURE_SIX)
        elif self.interrupt_reason == config.CONNECTION_FAILURE_SEVEN:
            self.internet_failure_resolver_pub.publish(config.CONNECTION_FAILURE_SEVEN)
        elif self.interrupt_reason == config.CONNECTION_FAILURE_EIGHT:
            self.gnss_failure_resolver_pub.publish(config.CONNECTION_FAILURE_EIGHT)
        elif self.interrupt_reason == config.CONNECTION_FAILURE_NINE:
            self.wifi_failure_resolver_pub.publish(config.CONNECTION_FAILURE_NINE)
        elif self.interrupt_reason == config.CONNECTION_FAILURE_TEN:
            self.internet_failure_resolver_pub.publish(config.CONNECTION_FAILURE_TEN)
        elif self.interrupt_reason == config.CONNECTION_FAILURE_ELEVEN:
            self.gnss_failure_resolver_pub.publish(config.CONNECTION_FAILURE_ELEVEN)
        elif self.interrupt_reason == config.CONNECTION_FAILURE_TWELVE:
            self.gnss_failure_resolver_pub.publish(config.CONNECTION_FAILURE_TWELVE)
        elif self.interrupt_reason == config.CONNECTION_FAILURE_THIRTEEN:
            self.gnss_failure_resolver_pub.publish(config.CONNECTION_FAILURE_THIRTEEN)
        elif self.interrupt_reason == config.CONNECTION_FAILURE_FOURTEEN:
            self.gnss_failure_resolver_pub.publish(config.CONNECTION_FAILURE_FOURTEEN)
        elif self.interrupt_reason == config.CONNECTION_FAILURE_FIFTEEN:
            self.gnss_failure_resolver_pub.publish(config.CONNECTION_FAILURE_FIFTEEN)
        elif self.interrupt_reason == config.CONNECTION_FAILURE_SIXTEEN:
            self.gnss_failure_resolver_pub.publish(config.CONNECTION_FAILURE_SIXTEEN)
        elif self.interrupt_reason == config.CONNECTION_FAILURE_SEVENTEEN:
            self.gnss_failure_resolver_pub.publish(config.CONNECTION_FAILURE_SEVENTEEN)
        elif self.interrupt_reason == config.CONNECTION_FAILURE_EIGHTEEN:
            self.gnss_failure_resolver_pub.publish(config.CONNECTION_FAILURE_EIGHTEEN)
        elif self.interrupt_reason == config.CONNECTION_FAILURE_NINETEEN:
            self.gnss_failure_resolver_pub.publish(config.CONNECTION_FAILURE_NINETEEN)
        elif self.interrupt_reason == config.CONNECTION_FAILURE_TWENTY:
            self.gnss_failure_resolver_pub.publish(config.CONNECTION_FAILURE_TWENTY)
        elif self.interrupt_reason == config.WEATHER_FAILURE_TWO:
            self.weather_failure_resolver_pub.publish(config.WEATHER_FAILURE_TWO)
        elif self.interrupt_reason == config.WEATHER_FAILURE_FIVE:
            self.weather_failure_resolver_pub.publish(config.WEATHER_FAILURE_FIVE)
        elif self.interrupt_reason == config.WEATHER_FAILURE_EIGHT:
            self.weather_failure_resolver_pub.publish(config.WEATHER_FAILURE_EIGHT)
        elif self.interrupt_reason == config.WEATHER_FAILURE_NINE:
            self.weather_failure_resolver_pub.publish(config.WEATHER_FAILURE_NINE)
        elif self.interrupt_reason == config.WEATHER_FAILURE_TEN:
            self.weather_failure_resolver_pub.publish(config.WEATHER_FAILURE_TEN)
        elif self.interrupt_reason == config.WEATHER_FAILURE_ELEVEN:
            self.weather_failure_resolver_pub.publish(config.WEATHER_FAILURE_ELEVEN)
        elif self.interrupt_reason == config.WEATHER_FAILURE_TWELVE:
            self.weather_failure_resolver_pub.publish(config.WEATHER_FAILURE_TWELVE)
        elif self.interrupt_reason == config.WEATHER_FAILURE_THIRTEEN:
            self.weather_failure_resolver_pub.publish(config.WEATHER_FAILURE_THIRTEEN)
        elif self.interrupt_reason == config.WEATHER_FAILURE_FOURTEEN:
            self.weather_failure_resolver_pub.publish(config.WEATHER_FAILURE_FOURTEEN)
        elif self.interrupt_reason == config.WEATHER_FAILURE_FIFTEEN:
            self.weather_failure_resolver_pub.publish(config.WEATHER_FAILURE_FIFTEEN)
        elif self.interrupt_reason == config.WEATHER_FAILURE_SIXTEEN:
            self.weather_failure_resolver_pub.publish(config.WEATHER_FAILURE_SIXTEEN)
        elif self.interrupt_reason == config.WEATHER_FAILURE_SEVENTEEN:
            self.weather_failure_resolver_pub.publish(config.WEATHER_FAILURE_SEVENTEEN)
        elif self.interrupt_reason == config.WEATHER_FAILURE_EIGHTEEN:
            self.weather_failure_resolver_pub.publish(config.WEATHER_FAILURE_EIGHTEEN)
        elif self.interrupt_reason == config.LOCALIZATION_FAILURE_ONE:
            self.localization_failure_resolver_pub.publish(config.LOCALIZATION_FAILURE_ONE)
        elif self.interrupt_reason == config.LOCALIZATION_FAILURE_TWO:
            self.localization_failure_resolver_pub.publish(config.LOCALIZATION_FAILURE_TWO)
        elif self.interrupt_reason == config.LOCALIZATION_FAILURE_THREE:
            self.localization_failure_resolver_pub.publish(config.LOCALIZATION_FAILURE_THREE)
        elif self.interrupt_reason == config.LOCALIZATION_FAILURE_FOUR:
            self.localization_failure_resolver_pub.publish(config.LOCALIZATION_FAILURE_FOUR)
        elif self.interrupt_reason == config.LOCALIZATION_FAILURE_FIVE:
            self.localization_failure_resolver_pub.publish(config.LOCALIZATION_FAILURE_FIVE)
        elif self.interrupt_reason == config.LOCALIZATION_FAILURE_SIX:
            self.localization_failure_resolver_pub.publish(config.LOCALIZATION_FAILURE_SIX)
        elif self.interrupt_reason == config.LOCALIZATION_FAILURE_SEVEN:
            self.localization_failure_resolver_pub.publish(config.LOCALIZATION_FAILURE_SEVEN)
        elif self.interrupt_reason == config.LOCALIZATION_FAILURE_EIGHT:
            self.localization_failure_resolver_pub.publish(config.LOCALIZATION_FAILURE_EIGHT)
        elif self.interrupt_reason == config.LOCALIZATION_FAILURE_NINE:
            self.localization_failure_resolver_pub.publish(config.LOCALIZATION_FAILURE_NINE)
        elif self.interrupt_reason == config.LOCALIZATION_FAILURE_TEN:
            self.localization_failure_resolver_pub.publish(config.LOCALIZATION_FAILURE_TEN)
        elif self.interrupt_reason == config.LOCALIZATION_FAILURE_ELEVEN:
            self.localization_failure_resolver_pub.publish(config.LOCALIZATION_FAILURE_ELEVEN)
        elif self.interrupt_reason == config.LOCALIZATION_FAILURE_TWELVE:
            self.localization_failure_resolver_pub.publish(config.LOCALIZATION_FAILURE_TWELVE)
        elif self.interrupt_reason == config.LOCALIZATION_FAILURE_THIRTEEN:
            self.localization_failure_resolver_pub.publish(config.LOCALIZATION_FAILURE_THIRTEEN)
        elif self.interrupt_reason == config.PLAN_DEPLOYMENT_FAILURE_ONE:
            self.plan_deployment_failure_resolver_pub.publish(config.PLAN_DEPLOYMENT_FAILURE_ONE)
        elif self.interrupt_reason == config.PLAN_DEPLOYMENT_FAILURE_TWO:
            self.plan_deployment_failure_resolver_pub.publish(config.PLAN_DEPLOYMENT_FAILURE_TWO)
        elif self.interrupt_reason == config.PLAN_DEPLOYMENT_FAILURE_THREE:
            self.plan_deployment_failure_resolver_pub.publish(config.PLAN_DEPLOYMENT_FAILURE_THREE)
        elif self.interrupt_reason == config.PLAN_DEPLOYMENT_FAILURE_FOUR:
            self.plan_deployment_failure_resolver_pub.publish(config.PLAN_DEPLOYMENT_FAILURE_FOUR)
        elif self.interrupt_reason == config.PLAN_DEPLOYMENT_FAILURE_FIVE:
            self.plan_deployment_failure_resolver_pub.publish(config.PLAN_DEPLOYMENT_FAILURE_FIVE)
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
        else:
            rospy.loginfo("unkonwn interrupt reason: %s", self.interrupt_reason)

        # TODO: implement time limit -> if exceeded, transition to catastrophe
        while not self.successfully_resolved:
            rospy.sleep(5)

        # solved case:
        #   - if problem resolved: return "solved"

        # aggravated case:
        #   - if minor issue couldn't be handled / something went wrong: return "aggravated"

        rospy.loginfo("contingency solved, continuing normal operation..")
        return "solved"


class Catastrophe(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['damage_control_performed'])
        self.interrupt_reason = ""
        rospy.Subscriber('/interrupt_reason', String, self.interrupt_reason_callback, queue_size=1)

    def interrupt_reason_callback(self, reason):
        self.interrupt_reason = reason.data

    def execute(self, userdata):
        rospy.loginfo("executing CATASTROPHE state..")
        rospy.loginfo("reason for catastrophe: %s", self.interrupt_reason)

        # do everything that is still possible:
        #   - communicate problem
        #   - save state
        #   - ...
        for _ in range(10):
            rospy.loginfo("#####################################################")
            rospy.sleep(2)
        rospy.loginfo("catastrophe processed, shutting down..")
        return "damage_control_performed"


class Shutdown(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['data_management_completed'])
        self.pub = rospy.Publisher('/arox/shutdown_trigger', String, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo("executing SHUTDOWN state..")
        rospy.loginfo("system shuts down..")
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

        operation_monitoring = smach.Concurrence(outcomes=['minor_complication', 'critical_complication', 'end_of_episode'],
                                                 default_outcome='end_of_episode',
                                                 child_termination_cb=self.child_termination_callback,
                                                 outcome_cb=self.out_callback)

        with operation_monitoring:
            smach.Concurrence.add('OPERATION', OperationStateMachine())
            smach.Concurrence.add('CONTINGENCY_MONITORING', smach_ros.MonitorState("/contingency_preemption", String, self.monitoring_callback))
            smach.Concurrence.add('CATASTROPHE_MONITORING', smach_ros.MonitorState("/catastrophe_preemption", String, self.monitoring_callback))

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
        elif outcome_map['CONTINGENCY_MONITORING'] == 'invalid' or  outcome_map['CATASTROPHE_MONITORING'] == 'invalid':
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
        pub = rospy.Publisher('SMACH_runnning', String, queue_size=1)
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
