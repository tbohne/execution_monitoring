#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ODEPhysics
from gazebo_msgs.srv import SetPhysicsProperties, SetPhysicsPropertiesRequest
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Vector3
from std_srvs.srv import Empty
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
import actionlib
from arox_navigation_flex.msg import drive_to_goalAction
from execution_monitoring import util
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import tf2_ros
from sensor_msgs.msg import NavSatFix

TF_BUFFER = None

class PhysicsController:

    def __init__(self):

        self.mbf_status = None
        self.sim_wheel_movement_without_pos_change = False
        self.sim_pos_change_without_wheel_movement = False
        self.sim_yaw_divergence = False
        self.pose_list = None

        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        rospy.Subscriber('/move_base_flex/exe_path/status', GoalStatusArray, self.mbf_status_callback, queue_size=1)

        rospy.Subscriber('/fix', NavSatFix, self.gnss, queue_size=1)
        rospy.Subscriber('/imu_data', Imu, self.imu_callback, queue_size=1)

        rospy.Subscriber('/wheel_movement_without_pos_change', String, self.wheel_movement_without_pos_change_callback, queue_size=1)
        rospy.Subscriber('/pos_change_without_wheel_movement', String, self.pos_change_without_wheel_movement_callback, queue_size=1)
        rospy.Subscriber('/yaw_divergence', String, self.yaw_divergence_callback, queue_size=1)

        self.drive_to_goal_client = actionlib.SimpleActionClient('drive_to_goal', drive_to_goalAction)

        service_name = '/gazebo/set_physics_properties'
        rospy.wait_for_service(service_name)
        rospy.loginfo("service found: %s", str(service_name))

        self.set_physics = rospy.ServiceProxy(service_name, SetPhysicsProperties)
        self.init_values()

    def gnss(self, nav_sat_fix):
        _, _, yaw = euler_from_quaternion([0, 0, self.imu_data.orientation.z, 0])
        self.pose_list = [nav_sat_fix.latitude, nav_sat_fix.longitude, yaw]

    def imu_callback(self, imu):
        self.imu_data = imu

    def mbf_status_callback(self, mbf_status):
        if len(mbf_status.status_list) > 0:
            curr = mbf_status.status_list[-1].status
            if curr == GoalStatus.ACTIVE and self.mbf_status != curr:
                # starting to move -> initiate simulation of low gravity -> spinning wheels
                if self.sim_wheel_movement_without_pos_change:
                    self.low_gravity_sim()
                if self.sim_pos_change_without_wheel_movement:
                    self.force_position_change_sim()

            elif curr != GoalStatus.ACTIVE:
                if self.sim_yaw_divergence:
                    self.yaw_divergence_sim()

            self.mbf_status = curr

    def yaw_divergence_sim(self):
        rospy.loginfo("yaw divergence sim..")

        if self.pose_list is not None:
            action_goal = util.create_dtg_goal(self.pose_list, self.pose_list[2])
            self.drive_to_goal_client.wait_for_server()

            orientation_list = [action_goal.target_pose.pose.orientation.x, action_goal.target_pose.pose.orientation.y, action_goal.target_pose.pose.orientation.z, action_goal.target_pose.pose.orientation.w]
            roll, pitch, yaw = euler_from_quaternion(orientation_list)
            rospy.loginfo("yaw: %s", yaw)
            q = quaternion_from_euler(roll, pitch, yaw + 270)

            action_goal.target_pose.pose.orientation.x = q[0]
            action_goal.target_pose.pose.orientation.y = q[1]
            action_goal.target_pose.pose.orientation.z = q[2]
            action_goal.target_pose.pose.orientation.w = q[3]

            self.drive_to_goal_client.send_goal(action_goal)
            rospy.loginfo("goal sent, wait for accomplishment..")
            success = self.drive_to_goal_client.wait_for_result()

            out = self.drive_to_goal_client.get_result()
            self.sim_yaw_divergence = False

            if out.progress > 0:
                rospy.loginfo("driving goal progress: %s", out.progress)
                return False        

            rospy.loginfo("successfully performed action: %s", success)
            return success        

    def force_position_change_sim(self):
        rospy.loginfo("force position change sim..")
        z = -9.81
        x = 0.0
        # necessary to start movement
        rospy.sleep(0.05)

        y = 8.0
        self.change_gravity(x, y, z)
        rospy.loginfo("changing gravity in y direction to: %s", y)
        rospy.sleep(2)

        y = 0.0
        self.change_gravity(x, y, z)
        rospy.loginfo("changing gravity back to normal")

        self.sim_pos_change_without_wheel_movement = False

    def low_gravity_sim(self):
        rospy.loginfo("init low gravity sim")
        x = y = 0.0
        # necessary to start movement
        rospy.sleep(0.05)

        z = 0.1
        self.change_gravity(x, y, z)
        rospy.loginfo("changing gravity in z direction to: %s", z)
        rospy.sleep(1)

        for _ in range(4):
            z = 0.2
            self.change_gravity(x, y, z)
            rospy.loginfo("changing gravity in z direction to: %s", z)
            rospy.sleep(1.0 / 1.5)
            z = -9.81
            self.change_gravity(x, y, z)
            rospy.loginfo("changing gravity in z direction to: %s", z)
            rospy.sleep(1 / 35.0)

        self.sim_wheel_movement_without_pos_change = False

    def wheel_movement_without_pos_change_callback(self, msg):
        self.sim_wheel_movement_without_pos_change = True

    def pos_change_without_wheel_movement_callback(self, msg):
        self.sim_pos_change_without_wheel_movement = True

    def yaw_divergence_callback(self, msg):
        self.sim_yaw_divergence = True

    def init_values(self):
        ##################### GAZEBO DEFAULT VALUES #####################
        self.time_step = Float64(0.001)
        self.max_update_rate = Float64(1000.0)

        self.gravity = Vector3()
        self.gravity.x = 0.0
        self.gravity.y = 0.0
        self.gravity.z = -9.81

        self.ode_config = ODEPhysics()
        self.ode_config.sor_pgs_precon_iters = 0
        self.ode_config.sor_pgs_iters = 50
        self.ode_config.sor_pgs_w = 1.3
        self.ode_config.sor_pgs_rms_error_tol = 0.0
        self.ode_config.contact_max_correcting_vel = 100.0
        self.ode_config.cfm = 0.0
        self.ode_config.erp = 0.2
        self.ode_config.contact_surface_layer = 0.001
        self.ode_config.max_contacts = 20
        #################################################################
        self.update_gravity_call()

    def update_gravity_call(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except rospy.ServiceException as e:
            rospy.loginfo("error: %s", e)

        set_physics_request = SetPhysicsPropertiesRequest()
        set_physics_request.time_step = self.time_step.data
        set_physics_request.max_update_rate = self.max_update_rate.data
        set_physics_request.gravity = self.gravity
        set_physics_request.ode_config = self.ode_config

        result = self.set_physics(set_physics_request)
        rospy.loginfo("gravity update result: %s, %s", result.success, result.status_message)

        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except rospy.ServiceException as e:
            rospy.loginfo("error: %s", e)

    def change_gravity(self, x, y, z):
        self.gravity.x = x
        self.gravity.y = y
        self.gravity.z = z
        self.update_gravity_call()

def node():
    rospy.init_node('physics_controller')
    PhysicsController()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
