#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from execution_monitoring import config, util
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import math
from geopy import distance

STOP_SIGN_POSES = [
    [30.702585, -23.646406, 0.671698, 0.0, 0.0, 0.619839],
    [31.195600, -23.287600, 0.671698, 0.0, 0.0, 0.619839],
    [31.688100, -22.930900, 0.671698, 0.0, 0.0, 0.619839],
    [32.183200, -22.573900, 0.671698, 0.0, 0.0, 0.619839],
    [32.677800, -22.216500, 0.671698, 0.0, 0.0, 0.619839],
    [33.175500, -21.848200, 0.671698, 0.0, 0.0, 0.619839]
]

BARRIER_POSES = [
    [28.351700, -23.712500, 0.782270, 0.0, 0.0, 0.0],
    [33.595978, -19.534710, 0.833558, 0.0, 0.0, 1.393767]
]

STOP_SIGN_MODEL = "/home/docker/catkin_ws/src/execution_monitoring/models/stop_sign/model.sdf"
BARRIER_MODEL = "/home/docker/catkin_ws/src/execution_monitoring/models/jersey_barrier/model.sdf"

class ObstacleSpawner:

    def __init__(self):
        rospy.Subscriber('/spawn_scenario_one_obstacles', String, self.spawn_scenario_one_obstacles, queue_size=1)
        rospy.Subscriber('/spawn_scenario_two_obstacles', String, self.spawn_scenario_two_obstacles, queue_size=1)
        rospy.Subscriber('/trigger_nav_fail', String, self.trigger_nav_fail, queue_size=1)
        rospy.Subscriber('/fix', NavSatFix, self.gnss_update, queue_size=1)
        self.robot_location = None
        self.insert_goal_pub = rospy.Publisher('introduce_intermediate_nav_goal', String, queue_size=1)

    def gnss_update(self, nav_sat_fix):
        self.robot_location = (nav_sat_fix.latitude, nav_sat_fix.longitude)

    def trigger_nav_fail(self, msg):
        # 0 -> base, 1 -> street, 2 -> field
        if self.robot_location is not None:
            dist_base = distance.distance((config.BASE_POSE[0], config.BASE_POSE[1]), self.robot_location).km
            dist_street = distance.distance((config.STREET[0], config.STREET[1]), self.robot_location).km
            dist_field = distance.distance((config.FIELD[0], config.FIELD[1]), self.robot_location).km

            if dist_base >= dist_street >= dist_field:
                self.insert_goal_pub.publish("0")
            elif dist_street >= dist_base >= dist_field:
                self.insert_goal_pub.publish("1")
            else:
                self.insert_goal_pub.publish("2")

    def spawn_scenario_two_obstacles(self, msg):
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        rospy.loginfo("spawning scenario two obstacles..")
        # create a new subscription to the topic, receive one message, then unsubscribe
        robot_pose = rospy.wait_for_message('/pose_ground_truth', Odometry, timeout=config.SCAN_TIME_LIMIT)
        pos_x = robot_pose.pose.pose.position.x
        pos_y = robot_pose.pose.pose.position.y
        quaternion = robot_pose.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])

        # spawn robot "prison"
        barrier_height = 0.833558
        offset = 2.0
        try:
            spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

            self.spawn_object('barrier_right', spawn_model_client, BARRIER_MODEL, pos_x - offset, pos_y, barrier_height, 0.0, 0.0, yaw)
            self.spawn_object('barrier_left', spawn_model_client, BARRIER_MODEL, pos_x + offset, pos_y, barrier_height, 0.0, 0.0, yaw)
            self.spawn_object('barrier_front', spawn_model_client, BARRIER_MODEL, pos_x, pos_y - offset, barrier_height, 0.0, 0.0, yaw + math.pi / 2)
            self.spawn_object('barrier_back', spawn_model_client, BARRIER_MODEL, pos_x, pos_y + offset, barrier_height, 0.0, 0.0, yaw + math.pi / 2)

        except rospy.ServiceException as e:
            print("Service call failed: ",e)

    def spawn_object(self, name, client, model, x, y, z, roll, pitch, yaw):
        spawn_pose = Pose()
        spawn_pose.position.x = x
        spawn_pose.position.y = y
        spawn_pose.position.z = z
        q = quaternion_from_euler(roll, pitch, yaw)
        spawn_pose.orientation.x = q[0]
        spawn_pose.orientation.y = q[1]
        spawn_pose.orientation.z = q[2]
        spawn_pose.orientation.w = q[3]

        client(
            model_name=name,
            model_xml=open(model, 'r').read(),
            robot_namespace='/foo',
            initial_pose=spawn_pose,
            reference_frame='world'
        )

    def spawn_scenario_one_obstacles(self, msg):
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        rospy.loginfo("spawning scenario one obstacles..")

        try:
            spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

            # spawn stop signs
            for i in range(len(STOP_SIGN_POSES)):
                self.spawn_object('stop_sign_' + str(i), spawn_model_client, STOP_SIGN_MODEL, *STOP_SIGN_POSES[i])

            # barriers
            for i in range(len(BARRIER_POSES)):
                self.spawn_object('barrier_' + str(i), spawn_model_client, BARRIER_MODEL, *BARRIER_POSES[i])
                

        except rospy.ServiceException as e:
            print("Service call failed: ",e)


def node():
    rospy.init_node('obstacle_spawner')
    rospy.loginfo("launch obstacle spawner..")
    ObstacleSpawner()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
