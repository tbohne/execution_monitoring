#!/usr/bin/env python
import rospy
import smach
from std_msgs.msg import String
from plan_executor import PlanExecutionStateMachine

class Contingency(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["catastrophe", "operation"])

    def execute(self, userdata):
        rospy.loginfo("contingency resolved, continuing normal operation..")
        return "operation"

class Catastrophe(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["shutdown"])

    def execute(self, userdata):
        rospy.loginfo("catastrophe processed, shutting down..")
        return "shutdown"
            
class Shutdown(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["shutdown"])
        self.pub = rospy.Publisher('/arox/shutdown_trigger',String,queue_size=1)

    def execute(self, ud):
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

            self.add('OPERATION', PlanExecutionStateMachine(),
                    transitions={'operation':'OPERATION',
                                 'contingency':'CONTINGENCY',
                                 'catastrophe':'CATASTROPHE',
                                 'shutdown':'SHUTDOWN'})

            self.add('CONTINGENCY', Contingency(),
                    transitions={'catastrophe':'CATASTROPHE',
                                 'operation':'OPERATION'})
            
            self.add('CATASTROPHE', Catastrophe(),
                    transitions={'shutdown':'SHUTDOWN'})

            self.add('SHUTDOWN', Shutdown(),
                    transitions={'shutdown':'shutdown'})


def node():
    """
    Plan execution and monitoring node.
    """
    rospy.init_node("execution_monitoring")

    sm = ExecutionMonitoringStateMachine()
    outcome = sm.execute()
    rospy.loginfo("outcome: %s", outcome)
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
