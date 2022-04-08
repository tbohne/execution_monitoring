#!/usr/bin/env python
import rospy
from execution_monitoring import config
from std_msgs.msg import String, UInt16, Bool
from arox_performance_parameters.msg import arox_operational_param
from datetime import datetime


class PlanDeploymentMonitor:

    def __init__(self):
        rospy.Subscriber(config.OPERATION_TOPIC, arox_operational_param, self.operation_callback, queue_size=1)
        rospy.Subscriber('/plan_retrieval_failure', UInt16, self.plan_fail_callback, queue_size=1)
        rospy.Subscriber('resolve_plan_deployment_failure_success', Bool, self.resolve_callback, queue_size=1)
        self.aggravate_pub = rospy.Publisher('/aggravate', String, queue_size=1)
        self.contingency_pub = rospy.Publisher('/contingency_preemption', String, queue_size=1)
        self.robot_info_pub = rospy.Publisher('/robot_info', String, queue_size=1)
        self.interrupt_reason_pub = rospy.Publisher('/interrupt_reason', String, queue_size=1)

        self.last_op_time = datetime.now()
        self.start_monitoring()

    def resolve_callback(self, msg):
        if not msg.data:
            self.interrupt_reason_pub.publish(config.PLAN_DEPLOYMENT_CATA)
            self.aggravate_pub.publish(config.PLAN_DEPLOYMENT_CATA)

    def start_monitoring(self):
        while not rospy.is_shutdown():
            time_since_last_op = (datetime.now() - self.last_op_time).total_seconds()
            if time_since_last_op > config.IDLE_THRESH:
                rospy.loginfo("time since last op: %.2f s", round(time_since_last_op, 2))
                # extended idle time -- worth an operator notification
                self.robot_info_pub.publish(config.PLAN_DEPLOYMENT_FAILURES[0])
            rospy.sleep(config.MON_FREQ)

    def operation_callback(self, operation_state):
        # rospy.loginfo("state of operation: %s", operation_state)
        self.last_op_time = datetime.now()
    
    def plan_fail_callback(self, msg):
        if msg.data == 0:
            rospy.loginfo("plan retrieval service timeout: %s", msg.data)
            rospy.sleep(1) # need a delay for some reason
            self.contingency_pub.publish(config.PLAN_DEPLOYMENT_FAILURES[1])
        elif msg.data == 1:
            rospy.loginfo("empty plan: %s", msg.data)
            rospy.sleep(1) # need a delay for some reason
            self.contingency_pub.publish(config.PLAN_DEPLOYMENT_FAILURES[2])
        elif msg.data == 2:
            rospy.loginfo("corrupted / infeasible plan: %s", msg.data)
            self.contingency_pub.publish(config.PLAN_DEPLOYMENT_FAILURES[3])
        else:
            rospy.loginfo("unknown plan fail code: %s", msg.data)
            self.contingency_pub.publish(config.PLAN_DEPLOYMENT_FAILURES[4])


def node():
    rospy.init_node('plan_deployment_monitoring')
    PlanDeploymentMonitor()
    rospy.loginfo("launch plan deployment monitoring..")
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
