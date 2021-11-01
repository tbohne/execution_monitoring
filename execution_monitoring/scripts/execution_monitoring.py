#!/usr/bin/env python
import rospy
import smach
import smach_ros
from std_msgs.msg import String
from operation import OperationStateMachine


class Contingency(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['solved', 'aggravated'])

    def execute(self, userdata):
        rospy.loginfo("executing CONTINGENCY state..")

        # solved case:
        #   - if problem resolved: return "solved"

        # aggravated case:
        #   - if minor issue couldn't be handled / something went wrong: return "aggravated"

        # default for now:
        for _ in range(10):
            rospy.loginfo("#####################################################")
            rospy.sleep(2)
        rospy.loginfo("contingency solved, continuing normal operation..")
        return "solved"


class Catastrophe(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['damage_control_performed'])

    def execute(self, userdata):
        rospy.loginfo("executing CATASTROPHE state..")

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
        rospy.loginfo("monitoring callback executed..")
        rospy.loginfo("userdata: %s", userdata)
        rospy.loginfo("msg: %s", msg)
        return False


def node():
    """
    High-level execution monitoring node.
    """
    rospy.init_node('execution_monitoring')

    sm = ExecutionMonitoringStateMachine()
    sis = smach_ros.IntrospectionServer('execution_monitoring', sm, '/EXECUTION_MONITORING')
    sis.start()
    outcome = sm.execute()
    rospy.loginfo("outcome: %s", outcome)
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
