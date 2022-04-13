#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @author Tim Bohne

import math

import rospy
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
from geopy import distance
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from execution_monitoring import config


class ObstacleSpawner:
    """
    Enables systematic simulation of certain scenarios with obstacles in Gazebo.
    """

    def __init__(self):
        self.robot_location = None
        self.nav_status = None
        self.sim_prison_retarded = False
        self.spawned_obstacles = []

        self.insert_goal_pub = rospy.Publisher('introduce_intermediate_nav_goal', String, queue_size=1)
        self.sim_info_pub = rospy.Publisher('/sim_info', String, queue_size=1)

        rospy.Subscriber('/spawn_static_obstacles', String, self.spawn_static_obstacles, queue_size=1)
        rospy.Subscriber('/spawn_robot_prison', String, self.spawn_robot_prison, queue_size=1)
        rospy.Subscriber('/trigger_nav_fail', String, self.trigger_nav_fail, queue_size=1)
        rospy.Subscriber('/fix', NavSatFix, self.gnss_update, queue_size=1)
        rospy.Subscriber(config.GOAL_STATUS_TOPIC, GoalStatusArray, self.nav_status_callback, queue_size=1)
        rospy.Subscriber('/clear_spawned_obstacles', String, self.delete_spawned_obstacles, queue_size=1)

    def nav_status_callback(self, nav_status):
        """
        Callback that receives navigation status updates and spawns a 'robot prison' in case the flag is set and the
        required conditions are met.

        @param nav_status: status of the navigation
        """
        if len(nav_status.status_list) > 0:
            # the last element in the list is the latest (most recent)
            if self.nav_status == GoalStatus.ACTIVE and nav_status.status_list[-1].status != self.nav_status \
                    and self.sim_prison_retarded:
                self.nav_status = nav_status.status_list[-1].status
                self.spawn_robot_prison("")
                self.sim_prison_retarded = False
            self.nav_status = nav_status.status_list[-1].status

    def gnss_update(self, nav_sat_fix):
        """
        Callback that receives GNSS status updates used to keep track of the robot's location.

        @param nav_sat_fix: GNSS data
        """
        self.robot_location = (nav_sat_fix.latitude, nav_sat_fix.longitude)

    def trigger_nav_fail(self, msg):
        """
        Attempts to trigger a navigation failure by introducing an intermediate navigation goal that is supposed to be
        kind of hard to reach from the robot's current location.

        Three defined options for the scenario at hand:
        - 0 --> base
        - 1 --> street
        - 2 --> field

        @param msg: callback message
        """
        if self.robot_location:
            self.sim_info_pub.publish("obstacle spawner: sim nav fail through intermediate 'hard-to-reach' goal")
            dist_base = distance.distance((config.BASE_POSE[0], config.BASE_POSE[1]), self.robot_location).km
            dist_street = distance.distance((config.STREET[0], config.STREET[1]), self.robot_location).km
            dist_field = distance.distance((config.FIELD[0], config.FIELD[1]), self.robot_location).km

            # choosing the one with the largest distance -- heuristic for being 'hard-to-reach'
            if dist_base >= dist_street >= dist_field:
                self.insert_goal_pub.publish("0")
            elif dist_street >= dist_base >= dist_field:
                self.insert_goal_pub.publish("1")
            else:
                self.insert_goal_pub.publish("2")

    def spawn_robot_prison(self, msg):
        """
        Spawns the 'robot prison' - static obstacles that completely confine the robot.

        @param msg: callback message
        """
        if self.nav_status == GoalStatus.SUCCEEDED or self.nav_status == GoalStatus.ABORTED:
            rospy.wait_for_service("/gazebo/spawn_sdf_model")
            rospy.loginfo("spawning 'robot prison'")
            self.sim_info_pub.publish("obstacle spawner: spawning 'robot prison'")

            # the 'prison' should be spawned relative to the robot position
            # -> create a new subscription to the topic, receive one message, then unsubscribe
            robot_pose = rospy.wait_for_message('/pose_ground_truth', Odometry, timeout=config.SCAN_TIME_LIMIT)
            pos_x = robot_pose.pose.pose.position.x
            pos_y = robot_pose.pose.pose.position.y
            quaternion = robot_pose.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])

            try:
                spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

                self.spawn_object('barrier_right', spawn_model_client, config.BARRIER_MODEL,
                                  pos_x + (config.DIST_TO_ROBOT * math.cos(math.radians(90) + yaw)),
                                  pos_y + (config.DIST_TO_ROBOT * math.sin(math.radians(90) + yaw)),
                                  config.BARRIER_HEIGHT, 0.0, 0.0, yaw)

                self.spawn_object('barrier_left', spawn_model_client, config.BARRIER_MODEL,
                                  pos_x + (config.DIST_TO_ROBOT * math.cos(math.radians(270) + yaw)),
                                  pos_y + (config.DIST_TO_ROBOT * math.sin(math.radians(270) + yaw)),
                                  config.BARRIER_HEIGHT, 0.0, 0.0, yaw)

                self.spawn_object('barrier_front', spawn_model_client, config.BARRIER_MODEL,
                                  pos_x + (config.DIST_TO_ROBOT * math.cos(math.radians(0) + yaw)),
                                  pos_y + (config.DIST_TO_ROBOT * math.sin(math.radians(0) + yaw)),
                                  config.BARRIER_HEIGHT, 0.0, 0.0, yaw + math.pi / 2)

                self.spawn_object('barrier_back', spawn_model_client, config.BARRIER_MODEL,
                                  pos_x + (config.DIST_TO_ROBOT * math.cos(math.radians(180) + yaw)),
                                  pos_y + (config.DIST_TO_ROBOT * math.sin(math.radians(180) + yaw)),
                                  config.BARRIER_HEIGHT, 0.0, 0.0, yaw + math.pi / 2)

            except rospy.ServiceException as e:
                print("service call failed: ", e)
        else:
            rospy.loginfo("spawning obstacles when robot stands still..")
            self.sim_prison_retarded = True

    def spawn_object(self, name, client, model, x, y, z, roll, pitch, yaw):
        """
        Spawns an object (obstacle) based on the specified properties.

        @param name: name of the object to be spawned
        @param client: gazebo spawn client
        @param model: model file (sdf) of the object to be spawned
        @param x: x coordinate of the spawn position
        @param y: y coordinate of the spawn position
        @param z: z coordinate of the spawn position
        @param roll: roll orientation of the spawned object
        @param pitch: pitch orientation of the spawned object
        @param yaw: yaw orientation of the spawned object
        """
        self.spawned_obstacles.append(name)
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

    def delete_spawned_obstacles(self, msg):
        """
        Deletes all the spawned obstacles.

        @param msg: callback message
        """
        delete_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
        for obstacle in self.spawned_obstacles:
            delete_model_prox(obstacle)
        self.spawned_obstacles = []

    def spawn_static_obstacles(self, msg):
        """
        Spawns the specified static obstacle scenario.

        @param msg: callback message - specifies the static obstacle scenario to be spawned
        """
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        sign_poses = barrier_poses = None

        if msg.data == "scene_one":
            rospy.loginfo("spawning scenario one obstacles..")
            self.sim_info_pub.publish("obstacle spawner: spawning scenario one obstacles")
            sign_poses = config.STOP_SIGN_POSES_SCENE_ONE
            barrier_poses = config.BARRIER_POSES_SCENE_ONE
        elif msg.data == "scene_two":
            rospy.loginfo("spawning scenario two obstacles..")
            self.sim_info_pub.publish("obstacle spawner: spawning scenario two obstacles")
            sign_poses = config.STOP_SIGN_POSES_SCENE_TWO
            barrier_poses = config.BARRIER_POSES_SCENE_TWO
        elif msg.data == "scene_three":
            rospy.loginfo("spawning scenario three obstacles..")
            self.sim_info_pub.publish("obstacle spawner: spawning scenario three obstacles")
            sign_poses = config.STOP_SIGN_POSES_SCENE_THREE
            barrier_poses = config.BARRIER_POSES_SCENE_THREE
        elif msg.data == "scene_four":
            rospy.loginfo("spawning scenario four obstacles..")
            self.sim_info_pub.publish("obstacle spawner: spawning scenario four obstacles")
            sign_poses = config.STOP_SIGN_POSES_SCENE_FOUR
            barrier_poses = config.BARRIER_POSES_SCENE_FOUR
        else:
            rospy.loginfo("unknown scene: %s", msg.data)
            rospy.loginfo("using default scene (one)..")
            self.sim_info_pub.publish("obstacle spawner: spawning scenario one obstacles")

        try:
            spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            # spawn stop signs
            for i in range(len(sign_poses)):
                self.spawn_object(
                    'stop_sign_' + msg.data + str(i), spawn_model_client, config.STOP_SIGN_MODEL, *sign_poses[i]
                )
            # spawn barriers
            for i in range(len(barrier_poses)):
                self.spawn_object(
                    'barrier_' + msg.data + str(i), spawn_model_client, config.BARRIER_MODEL, *barrier_poses[i]
                )
        except rospy.ServiceException as e:
            print("service call failed: ", e)


def node():
    """
    Node that enables systematic simulation of certain scenarios with obstacles in Gazebo.
    """
    rospy.init_node('obstacle_spawner')
    rospy.loginfo("launch obstacle spawner..")
    ObstacleSpawner()
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
