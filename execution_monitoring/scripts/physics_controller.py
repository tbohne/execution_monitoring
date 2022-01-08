#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ODEPhysics
from gazebo_msgs.srv import SetPhysicsProperties, SetPhysicsPropertiesRequest
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Vector3, Twist
from std_srvs.srv import Empty
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
import actionlib
from arox_navigation_flex.msg import drive_to_goalAction
from execution_monitoring import config, util
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import numpy as np
import math
from sensor_msgs.msg import NavSatFix

class PhysicsController:

    def __init__(self):
        self.mbf_status = None
        self.sim_wheel_movement_without_pos_change = False
        self.sim_pos_change_without_wheel_movement = False
        self.sim_moving_although_standing_still_imu = False
        self.sim_moving_although_standing_still_odom = False
        self.sim_yaw_divergence = False
        self.pose_list = None

        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # INFO / DATA TOPICS
        rospy.Subscriber('/move_base_flex/exe_path/status', GoalStatusArray, self.mbf_status_callback, queue_size=1)
        rospy.Subscriber('/fix', NavSatFix, self.gnss_callback, queue_size=1)
        rospy.Subscriber('/imu_data', Imu, self.imu_callback, queue_size=1)
        # SIM TOPICS
        rospy.Subscriber('/wheel_movement_without_pos_change', String, self.wheel_movement_without_pos_change_callback, queue_size=1)
        rospy.Subscriber('/pos_change_without_wheel_movement', String, self.pos_change_without_wheel_movement_callback, queue_size=1)
        rospy.Subscriber('/yaw_divergence', String, self.yaw_divergence_callback, queue_size=1)
        rospy.Subscriber('/moving_although_standing_still_imu', String, self.moving_although_standing_still_imu_callback, queue_size=1)
        rospy.Subscriber('/moving_although_standing_still_odom', String, self.moving_although_standing_still_odom_callback, queue_size=1)

        self.drive_to_goal_client = actionlib.SimpleActionClient('drive_to_goal', drive_to_goalAction)

        service_name = '/gazebo/set_physics_properties'
        rospy.wait_for_service(service_name)
        rospy.loginfo("service found: %s", str(service_name))

        self.set_physics = rospy.ServiceProxy(service_name, SetPhysicsProperties)
        self.init_values()

    def gnss_callback(self, nav_sat_fix):
        _, _, yaw = euler_from_quaternion([self.imu_data.orientation.x, self.imu_data.orientation.y, self.imu_data.orientation.z, self.imu_data.orientation.w])
        self.pose_list = [nav_sat_fix.latitude, nav_sat_fix.longitude, yaw * 180 / np.pi]

    def imu_callback(self, imu):
        self.imu_data = imu

    def mbf_status_callback(self, mbf_status):
        if len(mbf_status.status_list) > 0:
            curr = mbf_status.status_list[-1].status

            if curr == GoalStatus.ACTIVE:
                if self.sim_yaw_divergence:
                    self.yaw_divergence()
                # starting to move -> initiate simulation of low gravity -> spinning wheels
                if self.mbf_status != curr and self.sim_wheel_movement_without_pos_change:
                    self.wheel_movement_without_pos_change()

            elif curr != GoalStatus.ACTIVE:
                if self.sim_moving_although_standing_still_imu:
                    self.moving_although_standing_still_imu()
                if self.sim_moving_although_standing_still_odom:
                    self.moving_although_standing_still_odom()
                if self.sim_pos_change_without_wheel_movement:
                    self.pos_change_without_wheel_movement()

            self.mbf_status = curr

    def moving_although_standing_still_odom(self):
        rospy.sleep(config.STATUS_SWITCH_DELAY + 1)
        rospy.loginfo("PHYS CON: sim movement without cause (cmd_vel)..")
        self.sim_moving_although_standing_still_odom = False
        twist = Twist()
        twist.linear.x = 3.0
        self.cmd_vel_pub.publish(twist)

    def moving_although_standing_still_imu(self):
        rospy.sleep(config.STATUS_SWITCH_DELAY + 1)
        rospy.loginfo("PHYS CON: sim movement without cause (gravity)..")
        self.sim_moving_although_standing_still_imu = False

        z = -9.81
        x = 0.0
        y = 3.5

        self.change_gravity(x, y, z)
        rospy.loginfo("changing gravity..")
        rospy.sleep(1)

        x = y = 0.0
        z = -9.81
        self.change_gravity(x, y, z)
        rospy.loginfo("changing gravity back to normal..")

    def yaw_divergence(self):
        rospy.loginfo("PHYS CON: yaw divergence sim..")
        self.sim_yaw_divergence = False

        if self.pose_list is not None:
            
            yaw_deg = self.pose_list[2]
            if yaw_deg + 180 > 180:
                yaw_deg = -180 + ((yaw_deg + 180) % 180)
            else:
                yaw_deg += 180


        z = -9.81
        x = 2.5
        y = 0.0

        self.change_gravity(x, y, z)
        rospy.loginfo("changing gravity..")

        twist = Twist()

        for _ in range(200):
            #twist.linear.x = 0.7
            twist.angular.z = math.radians(yaw_deg)
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.01)

        x = y = 0.0
        z = -9.81
        self.change_gravity(x, y, z)
        rospy.loginfo("changing gravity back to normal..")        
            
            # action_goal = util.create_dtg_goal(self.pose_list, math.radians(yaw_deg))
            # self.drive_to_goal_client.wait_for_server()

            # self.drive_to_goal_client.send_goal(action_goal)
            # rospy.loginfo("goal sent, wait for accomplishment..")
            # success = self.drive_to_goal_client.wait_for_result()

            # out = self.drive_to_goal_client.get_result()
            # self.sim_yaw_divergence = False

            # if out.progress > 0:
            #     rospy.loginfo("driving goal progress: %s", out.progress)
            #     return False        

            # rospy.loginfo("successfully performed action: %s", success)
            # return success

    def pos_change_without_wheel_movement(self):
        rospy.loginfo("PHYS CON: sim pos change without wheel rotations..")
        z = -9.81
        x = 0.0
        # necessary to start movement
        rospy.sleep(0.05)

        # TODO: time or force should be reduced.. moved too far away..
        y = 8.0
        self.change_gravity(x, y, z)
        rospy.loginfo("changing gravity in y direction to: %s", y)
        rospy.sleep(1)

        y = 0.0
        self.change_gravity(x, y, z)
        rospy.loginfo("changing gravity back to normal")

        self.sim_pos_change_without_wheel_movement = False

    def wheel_movement_without_pos_change(self):
        rospy.loginfo("PHYS CON: sim wheel rotations without pos change..")
        x = y = 0.0

        # let robot hover a bit
        z = 0.4
        self.change_gravity(x, y, z)
        rospy.loginfo("changing gravity in z direction to: %s", z)
        rospy.sleep(0.2)

        # prevent it from flying away
        z = -0.1
        self.change_gravity(x, y, z)
        rospy.loginfo("changing gravity in z direction to: %s", z)
        rospy.sleep(0.2)

        # wait in zero gravity
        z = 0.0
        self.change_gravity(x, y, z)
        rospy.loginfo("changing gravity in z direction to: %s", z)
        rospy.sleep(1.5)

        # back to normal
        z = -9.81
        rospy.loginfo("changing back to normal gravtiy..")
        self.change_gravity(x, y, z)

        self.sim_wheel_movement_without_pos_change = False

    def wheel_movement_without_pos_change_callback(self, msg):
        self.sim_wheel_movement_without_pos_change = True

    def pos_change_without_wheel_movement_callback(self, msg):
        self.sim_pos_change_without_wheel_movement = True

    def moving_although_standing_still_imu_callback(self, msg):
        self.sim_moving_although_standing_still_imu = True

    def moving_although_standing_still_odom_callback(self, msg):
        self.sim_moving_although_standing_still_odom = True

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
