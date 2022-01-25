#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from execution_monitoring import config, util
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point
from tf.transformations import quaternion_from_euler

class NavigationMonitoring:

    def __init__(self):

        rospy.Subscriber('/spawn_obstacles', String, self.spawn_obstacles, queue_size=1)


    def spawn_obstacles(self, msg):
        rospy.wait_for_service("/gazebo/spawn_sdf_model")

        rospy.loginfo("SPAWNING..")

        try:
            spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

            spawn_pose = Pose()
            spawn_pose.position.x = 32.811500
            spawn_pose.position.y = -23.770700
            spawn_pose.position.z = 0.891628
            q = quaternion_from_euler(0.0, 0.0, 0.541292)
            spawn_pose.orientation.x = q[0]
            spawn_pose.orientation.y = q[1]
            spawn_pose.orientation.z = q[2]
            spawn_pose.orientation.w = q[3]

            spawn_model_client(
                model_name='stop_sign',
                model_xml=open('/home/docker/catkin_ws/src/execution_monitoring/models/stop_sign/model.sdf', 'r').read(),
                robot_namespace='/foo',
                initial_pose=spawn_pose,
                reference_frame='world'
            )

        except rospy.ServiceException as e:
            print("Service call failed: ",e)


def node():
    rospy.init_node('navigation_monitoring')
    rospy.loginfo("launch navigation monitoring..")
    NavigationMonitoring()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
