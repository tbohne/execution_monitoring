#!/usr/bin/env python
import rospy
import time
from gazebo_msgs.msg import ODEPhysics
from gazebo_msgs.srv import SetPhysicsProperties, SetPhysicsPropertiesRequest
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Vector3, Twist
from std_srvs.srv import Empty
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from datetime import datetime

class PhysicsController:

    def __init__(self):
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)

        rospy.Subscriber('/move_base_flex/exe_path/status', GoalStatusArray, self.mbf_status_callback, queue_size=1)

        rospy.Subscriber('/change_gravity', String, self.change_gravity_callback, queue_size=1)

        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        service_name = '/gazebo/set_physics_properties'
        rospy.wait_for_service(service_name)
        rospy.loginfo("service found: %s", str(service_name))

        self.set_physics = rospy.ServiceProxy(service_name, SetPhysicsProperties)
        self.init_values()

        self.mbf_status = None
        self.sim_low_gravity = False

    def mbf_status_callback(self, mbf_status):
        if len(mbf_status.status_list) > 0:
            curr = mbf_status.status_list[-1].status
            if curr == GoalStatus.ACTIVE and self.mbf_status != curr:
                # starting to move -> initiate simulation of low gravity -> spinning wheels
                if self.sim_low_gravity:
                    self.low_gravity_sim()
            self.mbf_status = curr

    def low_gravity_sim(self):
        rospy.loginfo("init low gravity sim")
        x = y = 0.0
        
        for _ in range(2):
            gravity_value = 0.3
            z = gravity_value
            self.change_gravity(x, y, z)
            rospy.loginfo("changing gravity in z direction to: %s", gravity_value)

            twist = Twist()
            twist.linear.x = 30.0
            for _ in range(2):
                self.cmd_vel_pub.publish(twist)
                rospy.sleep(1)

            z = -9.81
            self.change_gravity(x, y, z)
            rospy.loginfo("changing gravity in z direction to: %s", z)
            rospy.sleep(0.1)

        self.sim_low_gravity = False

    def change_gravity_callback(self, msg):
        self.sim_low_gravity = True

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
