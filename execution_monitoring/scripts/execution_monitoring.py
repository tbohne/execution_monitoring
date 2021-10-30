#!/usr/bin/env python
import rospy
import smach
import smach_ros
from std_msgs.msg import String
from operation import OperationStateMachine


class Contingency(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['normal_operation', 'catastrophe'])

    def execute(self, userdata):
        rospy.loginfo("executing CONTINGENCY state..")

        # normal_operation case:
        #   - if problem resolved: return "normal_operation"

        # catastrophe case:
        #   - if minor issue couldn't be handled / something went wrong: return "catastrophe"

        # default for now:
        rospy.loginfo("contingency resolved, continuing normal operation..")
        return "normal_operation"


class Catastrophe(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['shutdown'])

    def execute(self, userdata):
        rospy.loginfo("executing CATASTROPHE state..")

        # do everything that is still possible:
        #   - communicate problem
        #   - save state
        #   - ...
        rospy.loginfo("catastrophe processed, shutting down..")
        return "shutdown"


class Shutdown(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['shutdown'])
        self.pub = rospy.Publisher('/arox/shutdown_trigger', String, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo("executing SHUTDOWN state..")
        rospy.loginfo("system shuts down..")
        self.pub.publish("shutdown")
        return "shutdown"


class ExecutionMonitoringStateMachine(smach.StateMachine):

    def __init__(self):
        super(ExecutionMonitoringStateMachine, self).__init__(
            outcomes=['shutdown'],
            input_keys=[],
            output_keys=[]
        )

        with self:
            self.add('NORMAL_OPERATION', OperationStateMachine(),
                     transitions={'contingency': 'CONTINGENCY',
                                  'catastrophe': 'CATASTROPHE',
                                  'shutdown': 'SHUTDOWN'})

            self.add('CONTINGENCY', Contingency(),
                     transitions={'normal_operation': 'NORMAL_OPERATION',
                                  'catastrophe': 'CATASTROPHE'})

            self.add('CATASTROPHE', Catastrophe(),
                     transitions={'shutdown': 'SHUTDOWN'})

            self.add('SHUTDOWN', Shutdown(),
                     transitions={'shutdown': 'shutdown'})


def node():
    """
    High-level execution monitoring node.
    """
    rospy.init_node('execution_monitoring')

    sm = ExecutionMonitoringStateMachine()
    sis = smach_ros.IntrospectionServer('execution_monitoring', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    rospy.loginfo("outcome: %s", outcome)
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
