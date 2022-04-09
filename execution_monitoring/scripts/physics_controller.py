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
from arox_performance_parameters.msg import arox_operational_param
import numpy as np
from sensor_msgs.msg import NavSatFix


class PhysicsController:
    """
    The evaluation of localization problems as well as the corresponding monitoring procedures demands the simulation
    of such cases. For this purpose, this class was written. The general idea of the PhysicsController is
    to manipulate certain aspects of the physics in the Gazebo simulation to provoke the circumstances that lead to
    localization problems.
    """

    def __init__(self):
        self.mbf_status = None
        self.pose_list = None
        self.imu_data = None
        self.operation_mode = None
        self.sim_yaw_divergence = False
        self.time_step = None
        self.max_update_rate = None
        self.gravity = None
        self.ode_config = None
        self.sim_wheel_movement_without_pos_change = False
        self.sim_pos_change_without_wheel_movement = False
        self.sim_moving_although_standing_still_imu = False
        self.sim_moving_although_standing_still_odom = False

        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.sim_info_pub = rospy.Publisher('/sim_info', String, queue_size=1)

        # INFO / DATA TOPICS
        rospy.Subscriber(config.GOAL_STATUS_TOPIC, GoalStatusArray, self.nav_status_callback, queue_size=1)
        rospy.Subscriber('/fix', NavSatFix, self.gnss_callback, queue_size=1)
        rospy.Subscriber('/imu_data', Imu, self.imu_callback, queue_size=1)
        rospy.Subscriber("/arox/ongoing_operation", arox_operational_param, self.operation_callback)

        # SIM TOPICS
        rospy.Subscriber('/yaw_divergence', String, self.yaw_divergence_callback, queue_size=1)
        rospy.Subscriber('/wheel_movement_without_pos_change', String,
                         self.wheel_movement_without_pos_change_callback, queue_size=1)
        rospy.Subscriber('/pos_change_without_wheel_movement', String,
                         self.pos_change_without_wheel_movement_callback, queue_size=1)
        rospy.Subscriber('/moving_although_standing_still_imu', String,
                         self.moving_although_standing_still_imu_callback, queue_size=1)
        rospy.Subscriber('/moving_although_standing_still_odom', String,
                         self.moving_although_standing_still_odom_callback, queue_size=1)

        self.drive_to_goal_client = actionlib.SimpleActionClient('drive_to_goal', drive_to_goalAction)

        service_name = '/gazebo/set_physics_properties'
        rospy.wait_for_service(service_name)
        self.set_physics = rospy.ServiceProxy(service_name, SetPhysicsProperties)
        self.init_values()

    def operation_callback(self, msg):
        """
        Callback providing information about the current state of the operation.

        @param msg: callback message - operational parameters
        """
        self.operation_mode = msg.operation_mode

    def gnss_callback(self, nav_sat_fix):
        """
        Callback providing GNSS information used to track the robot's pose.

        @param nav_sat_fix: GNSS information
        """
        _, _, yaw = euler_from_quaternion(
            [self.imu_data.orientation.x, self.imu_data.orientation.y, self.imu_data.orientation.z,
             self.imu_data.orientation.w])
        self.pose_list = [nav_sat_fix.latitude, nav_sat_fix.longitude, yaw * 180 / np.pi]

    def imu_callback(self, imu):
        """
        Callback providing information from the inertial measurement unit.

        @param imu: IMU data
        """
        self.imu_data = imu

    def nav_status_callback(self, nav_status):
        """
        Callback providing information about the current status of navigation.
        Used to initiate certain simulations that require special navigational conditions.

        @param nav_status: current status of navigation
        """
        if len(nav_status.status_list) > 0:
            curr = nav_status.status_list[-1].status

            if curr == GoalStatus.ACTIVE and self.operation_mode not in ["docking", "undocking"]:
                if self.sim_yaw_divergence:
                    self.yaw_divergence()
                # starting to move -> initiate simulation of low gravity -> spinning wheels
                if self.mbf_status != curr and self.sim_wheel_movement_without_pos_change:
                    self.wheel_movement_without_pos_change()

            # only during scanning -- prevents issues with docking etc.
            elif curr != GoalStatus.ACTIVE and self.operation_mode == "scanning":
                if self.sim_moving_although_standing_still_imu:
                    self.moving_although_standing_still_imu()
                if self.sim_moving_although_standing_still_odom:
                    self.moving_although_standing_still_odom()
                if self.sim_pos_change_without_wheel_movement:
                    self.pos_change_without_wheel_movement()

            self.mbf_status = curr

    def moving_although_standing_still_odom(self):
        """
        Simulates a situation in which the odometry indicates movement, although there is no active navigation.
        """
        rospy.sleep(config.STATUS_SWITCH_DELAY)
        rospy.loginfo("physics controller: sim unplanned movement via cmd_vel")
        self.sim_info_pub.publish("physics controller: sim unplanned odometry movement via cmd_vel")
        self.sim_moving_although_standing_still_odom = False
        twist = Twist()
        twist.linear.x = 3.0
        self.cmd_vel_pub.publish(twist)

    def moving_although_standing_still_imu(self):
        """
        Simulates a situation in which the IMU indicates movement, although there is no active navigation.
        """
        rospy.sleep(config.STATUS_SWITCH_DELAY)
        rospy.loginfo("physics controller: sim unplanned IMU movement via gravity manipulation")
        self.sim_info_pub.publish("physics controller: sim unplanned IMU movement via gravity manipulation")
        self.sim_moving_although_standing_still_imu = False
        self.change_gravity(0.0, 5.5, -9.81)
        rospy.loginfo("changing gravity..")
        rospy.sleep(1)
        rospy.loginfo("changing gravity back to normal..")
        self.change_gravity(0.0, 0.0, -9.81)

    def yaw_divergence(self):
        """
        Simulates a situation in which the yaw estimates of the different components diverge.
        """
        rospy.loginfo("physics controller: yaw divergence sim..")
        self.sim_info_pub.publish("physics controller: sim yaw divergence")
        self.sim_yaw_divergence = False
        yaw_deg = None

        if self.pose_list:
            yaw_deg = self.pose_list[2]
            if yaw_deg + 50 > 180:
                yaw_deg = -180 + ((yaw_deg + 50) % 180)
            else:
                yaw_deg += 50

        twist = Twist()
        cnt = 0
        while abs(self.pose_list[2] - yaw_deg) > 10:
            if cnt == 45:
                self.change_gravity(2.5, 2.5, 0.2)
                rospy.loginfo("changing gravity..")
            twist.angular.z = np.radians(180) if yaw_deg > 0 else -np.radians(180)
            self.cmd_vel_pub.publish(twist)
            cnt += 1
            rospy.sleep(0.01)

        self.change_gravity(0.0, 0.0, -9.81)
        rospy.loginfo("changing gravity back to normal..")

    def pos_change_without_wheel_movement(self):
        """
        Simulates a situation with a position change without wheel movement, or, more generally, a situation in which
        the change in position cannot be fully explained by the wheel movement.
        """
        rospy.loginfo("physics controller: sim pos change without wheel rotations")
        self.sim_info_pub.publish("physics controller: sim pos change without wheel rotations")
        x = 0.0
        y = 8.0
        z = -9.81
        # necessary to start movement
        rospy.sleep(0.05)
        self.change_gravity(x, y, z)
        rospy.loginfo("changing gravity in y direction to: %s", y)
        rospy.sleep(1)
        y = 0.0
        self.change_gravity(x, y, z)
        rospy.loginfo("changing gravity back to normal")
        self.sim_pos_change_without_wheel_movement = False

    def wheel_movement_without_pos_change(self):
        """
        Simulates wheel motion with no change in position, resulting in a divergence between the estimated traveled
        distances by odometry and GNSS. In practice, such a situation can for example occur when the wheels spin due to
        low friction on an icy or muddy surface.
        """
        rospy.loginfo("physics controller: sim wheel rotations without pos change")
        self.sim_info_pub.publish("physics controller: sim wheel rotations without pos change")
        x = y = 0.0
        z = 0.4  # let robot hover a bit
        self.change_gravity(x, y, z)
        rospy.loginfo("changing gravity in z direction to: %s", z)
        rospy.sleep(0.2)
        z = -0.1  # prevent it from flying away
        self.change_gravity(x, y, z)
        rospy.loginfo("changing gravity in z direction to: %s", z)
        rospy.sleep(0.2)
        z = 0.0  # wait in zero gravity
        self.change_gravity(x, y, z)
        rospy.loginfo("changing gravity in z direction to: %s", z)
        twist = Twist()
        twist.linear.x = 1  # amplify wheel rotations in the air
        for i in range(1500):
            self.cmd_vel_pub.publish(twist)
            if i < 500:
                rospy.sleep(0.01)
            else:
                rospy.sleep(0.001)
        z = -9.81  # back to normal
        rospy.loginfo("changing back to normal gravity..")
        self.change_gravity(x, y, z)
        self.sim_wheel_movement_without_pos_change = False

    def wheel_movement_without_pos_change_callback(self, msg):
        """
        Initiates the simulation of a wheel movement without a change in position.

        @param msg: callback message
        """
        self.sim_wheel_movement_without_pos_change = True

    def pos_change_without_wheel_movement_callback(self, msg):
        """
        Initiates the simulation of a change in position without wheel movement.

        @param msg: callback message
        """
        self.sim_pos_change_without_wheel_movement = True

    def moving_although_standing_still_imu_callback(self, msg):
        """
        Initiates the simulation of IMU indicating movement without active navigation.

        @param msg: callback message
        """
        self.sim_moving_although_standing_still_imu = True

    def moving_although_standing_still_odom_callback(self, msg):
        """
        Initiates the simulation of odometry indicating movement without active navigation.

        @param msg: callback message
        """
        self.sim_moving_although_standing_still_odom = True

    def yaw_divergence_callback(self, msg):
        """
        Initiates the simulation of a yaw estimate divergence situation.

        @param msg: callback message
        """
        self.sim_yaw_divergence = True

    def init_values(self):
        """
        Initializes physics properties of the Gazebo simulation / Open Dynamics Engine (ODE) with default values
        from the literature / documentation.
        """
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

        self.update_gravity_call()

    def update_gravity_call(self):
        """
        Updates the gravity settings of the Gazebo simulation based on set properties.
        """
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
        """
        Manipulates the gravity of the Gazebo simulation based on the specified components.

        @param x: x component of the gravity vector
        @param y: y component of the gravity vector
        @param z: z component of the gravity vector
        """
        self.gravity.x = x
        self.gravity.y = y
        self.gravity.z = z
        self.update_gravity_call()


def node():
    """
    Physics manipulation node.
    """
    rospy.init_node('physics_controller')
    PhysicsController()
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
