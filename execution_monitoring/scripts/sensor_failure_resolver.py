#!/usr/bin/env python
import rospy
from std_msgs.msg import String

class SensorFailureResolver:

    def __init__(self):
        self.resolve_sub = rospy.Subscriber('/resolve_sensor_failure', String, self.resolve_callback, queue_size=1)
        self.toggle_sim_sensor_failure_pub = rospy.Publisher("/toggle_simulated_sensor_failure", String, queue_size=1)
        self.human_operator_pub = rospy.Publisher("/request_help", String, queue_size=1)
        self.human_operator_sub = rospy.Subscriber('/problem_solved', String, self.solved_by_human_callback, queue_size=1)
        self.problem_resolved = False

    def resolve_callback(self, msg):
        rospy.loginfo("launch sensor failure resolver..")
        self.problem_resolved = False
        rospy.loginfo("not able to handle - communicating problem to human operator..")
        self.human_operator_pub.publish("sensor failure detected - preempted NORMAL_OPERATION - need help")

        # TODO:  when it takes too long it should go to CATASTROPHE
        while not self.problem_resolved:
            rospy.loginfo("waiting for human operator to solve the problem..")
            rospy.sleep(30)
        self.toggle_sim_sensor_failure_pub.publish("")
        rospy.loginfo("sensor failure resolved..")

    def solved_by_human_callback(self):
        self.problem_resolved = True

def node():
    rospy.init_node('sensor_failure_resolver')
    rospy.wait_for_message('SMACH_runnning', String)
    SensorFailureResolver()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
