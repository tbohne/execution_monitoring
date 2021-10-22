#!/usr/bin/env python
import rospy
import smach
import threading
import smach_ros
from std_msgs.msg import String
from arox_operation import Operation_StateMachine

class Contingency(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["catastrophe", "shutdown", "charge"])

    def execute(self, userdata):
        rospy.loginfo("contingency resolved, continuing normal operation..")
        return "operation"

class Catastrophe(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["shutdown"])

    def execute(self, userdata):
        rospy.loginfo("catastrophe processed, shutting down..")
        return "shutdown"

class Dock(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["operation", "shutdown"])

    def execute(self, userdata):
        rospy.loginfo("docking - start charging...")
        rospy.set_param("charging_mode", True)
        charge_mode = rospy.get_param("charging_mode")
        
        while charge_mode:
            charge_mode = rospy.get_param("charging_mode")
            rospy.loginfo("charging in progress...")
            rospy.sleep(2)

        if not charge_mode:
            rospy.loginfo("charging finished..")
            return "operation"
            
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

            self.add('OPERATION', Operation_StateMachine(),
                    transitions={'shutdown':'SHUTDOWN',
                                 'operation':'OPERATION',
                                 'contingency':'CONTINGENCY',
                                 'catastrophe':'CATASTROPHE',
                                 'dock':'DOCK',
                                 'preempted':'shutdown'})

            self.add('CONTINGENCY', Contingency(),
                    transitions={'charge':'DOCK',
                                 'catastrophe':'CATASTROPHE'})
            
            self.add('CATASTROPHE', Catastrophe(),
                    transitions={'shutdown':'SHUTDOWN'})

            self.add('SHUTDOWN', Shutdown(),
                    transitions={'shutdown':'shutdown'})

            self.add('DOCK', Dock(),
                    transitions={'shutdown':'SHUTDOWN',
                                 'operation':'OPERATION'})

def node():
    """
    Plan execution and monitoring node.
    """
    rospy.init_node("execution_monitoring")

    sm = ExecutionMonitoringStateMachine()
    smach_ros.set_preempt_handler(sm)
    
    intro_serv = smach_ros.IntrospectionServer('execution_monitoring', sm, '/SM_ROOT')
    intro_serv.start()

    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()
    
    rospy.spin()
    smach_thread.join()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
