#!/usr/bin/env python
import rospy
import time
from gazebo_msgs.msg import ODEPhysics
from gazebo_msgs.srv import SetPhysicsProperties, SetPhysicsPropertiesRequest
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Vector3
from std_srvs.srv import Empty

class PhysicsController:

    def __init__(self):
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)

        rospy.Subscriber('/change_gravity', String, self.change_gravity_callback, queue_size=1)

        service_name = '/gazebo/set_physics_properties'
        rospy.wait_for_service(service_name)
        rospy.loginfo("service found: %s", str(service_name))

        self.set_physics = rospy.ServiceProxy(service_name, SetPhysicsProperties)
        self.init_values()

    def change_gravity_callback(self, msg):

        gravity_value = 1.0
        rospy.loginfo("changing gravity in z direction to: %s", gravity_value)
        x = 0.0
        y = 0.0
        z = gravity_value
        self.change_gravity(x, y, z)
        
        rospy.sleep(2)

        x = 0.0
        y = 0.0
        z = -9.81
        rospy.loginfo("changing gravity in z direction to: %s", gravity_value)
        self.change_gravity(x, y, z)

    def init_values(self):

        self.time_step = Float64(0.001)
        self.max_update_rate = Float64(1000.0)

        self.gravity = Vector3()
        self.gravity.x = 0.0
        self.gravity.y = 0.0
        self.gravity.z = -9.81

        self.ode_config = ODEPhysics()
        self.ode_config.auto_disable_bodies = False
        self.ode_config.sor_pgs_precon_iters = 0
        self.ode_config.sor_pgs_iters = 50
        self.ode_config.sor_pgs_w = 1.3
        self.ode_config.sor_pgs_rms_error_tol = 0.0
        self.ode_config.contact_surface_layer = 0.001
        self.ode_config.contact_max_correcting_vel = 0.0
        self.ode_config.cfm = 0.0
        self.ode_config.erp = 0.2
        self.ode_config.max_contacts = 20

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
