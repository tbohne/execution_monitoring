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

        with self:
            self.add('NORMAL_OPERATION', OperationStateMachine(),
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


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
