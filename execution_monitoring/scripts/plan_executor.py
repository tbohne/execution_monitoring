#!/usr/bin/env python
import smach
import actionlib 
import rospy
import time
import numpy as np
from osgeo import osr
from tf.transformations import quaternion_from_euler
from plan_generation.srv import get_plan
from plan_generation.msg import action
from arox_navigation_flex.msg import drive_to_goalAction
from arox_navigation_flex.msg import drive_to_goalGoal as dtg_Goal
from arox_performance_parameters.msg import arox_operational_param
from arox_performance_parameters.msg import arox_battery_params

BASE_POSE = [52.3203191407, 8.153625154949, 270]
plan_remaining = []
plan_initial = []
completed_tasks = 0


def publish_state_of_ongoing_operation(mode):
    global plan_initial, plan_remaining, completed_tasks
    operation_pub = rospy.Publisher('arox/ongoing_operation', arox_operational_param, queue_size=1)
    msg = arox_operational_param()
    msg.operation_mode = mode
    msg.total_tasks = len(plan_initial)
    msg.ongoing_task = len(plan_initial) - len(plan_remaining)
    msg.rewards_gained = completed_tasks
    operation_pub.publish(msg)


class Idle(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['idle', 'execute_plan'], output_keys=['plan'])

    def get_plan(self):
        rospy.wait_for_service('arox_planner/get_plan')
        try:
            res = rospy.ServiceProxy('arox_planner/get_plan', get_plan)()
            if res.succeeded:
                return res.generated_plan.actions
            return None                
        except rospy.ServiceException as e:
            print("service call failed: %s", e)
            return None

    def execute(self, userdata):
        global plan_initial, plan_remaining, completed_tasks

        if len(plan_initial) == 0:
            rospy.loginfo ("no active plan..")
            plan_initial = []
            publish_state_of_ongoing_operation("waiting")
            plan = self.get_plan()
            
            if plan != None:
                plan_initial = plan[:]
                userdata.plan = plan
                plan_remaining = plan_initial
                publish_state_of_ongoing_operation("undocking")
                return 'execute_plan'
            return 'idle'
         
        else:
            if completed_tasks == len(plan_initial):
                rospy.loginfo("completed plan, waiting for new plan..")
                time.sleep(3)
                return 'idle'
               
            else:
                rospy.loginfo("continuing preempted plan..")
                publish_state_of_ongoing_operation("undocking")
                userdata.plan = plan_remaining
                return 'execute_plan'


class ExecutePlan(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['complete', 'soft_failure', 'hard_failure', 'success', 'preemption'],
                                   input_keys=['plan'])

        rospy.Subscriber("/arox/battery_param", arox_battery_params, self.battery_callback, queue_size=1)
        self.battery_discharged = False

    def battery_callback(self, data):
        global plan_remaining

        if data.charge == 0.0:
            self.battery_discharged = True
            rospy.loginfo("battery completely discharged..")
            plan_remaining_length = len(self.plan) + 1
            plan_remaining = plan_initial[-plan_remaining_length:]
            publish_state_of_ongoing_operation("dead")
            client = actionlib.SimpleActionClient('drive_to_goal', drive_to_goalAction)
            client.cancel_all_goals()
            return 'shutdown'

    @staticmethod
    def perform_action(action):
        """
        TODO: add doc
        """
        global BASE_POSE

        rospy.loginfo("performing action %s..", action.name)

        # moving actions
        if action.name == "drive_to" or action.name == "return_to_base":

            if action.name == "return_to_base":
                action.pose = BASE_POSE

            publish_state_of_ongoing_operation("processing")
            
            client = actionlib.SimpleActionClient('drive_to_goal', drive_to_goalAction)
            action_goal = dtg_Goal()
            # TODO: could we use wgs84 here?
            action_goal.target_pose.header.frame_id= 'utm'

            # the coordinates in the plan are wgs84 - transform
            source = osr.SpatialReference()
            source.ImportFromEPSG(4326)
            target = osr.SpatialReference()
            target.ImportFromEPSG(32632)
            transform = osr.CoordinateTransformation(source, target)
            x, y = transform.TransformPoint(action.pose[1], action.pose[0])[0:2] # switch lat / lon
            action_goal.target_pose.pose.position.x = x
            action_goal.target_pose.pose.position.y = y
            
            q = quaternion_from_euler(0, 0, np.pi * (action.pose[2] + 90) / 180)
            action_goal.target_pose.pose.orientation.x = q[0]
            action_goal.target_pose.pose.orientation.y = q[1]
            action_goal.target_pose.pose.orientation.z = q[2]
            action_goal.target_pose.pose.orientation.w = q[3]
            client.wait_for_server()
            client.send_goal(action_goal)
            rospy.loginfo("goal sent, wait for accomplishment")
            publish_state_of_ongoing_operation("traversing")
            
            success = client.wait_for_result()
            rospy.loginfo("successfully performed action: %s", success)

            return success

        elif action.name == "scan":
            rospy.loginfo("start scanning procedure..")
            rospy.sleep(3)
            rospy.loginfo("scanning procedure finished..")
            return True

        elif action.name == "charge":
            rospy.loginfo("start docking procedure..")
            rospy.set_param("charging_mode", True)
            charge_mode = rospy.get_param("charging_mode")
            
            while charge_mode:
                charge_mode = rospy.get_param("charging_mode")
                rospy.loginfo("charging battery..")
                rospy.sleep(2)

            if not charge_mode:
                rospy.loginfo("battery charged..")
                return True
        else:
            rospy.loginfo("error - unknown action: %s", action.name)
        return False
                
    def execute(self, userdata):
        global plan_remaining, completed_tasks
        self.plan = userdata.plan
        
        if self.preempt_requested():
            return 'preemption'
        elif len(self.plan) == 0:            
            return 'complete'
        else:
            rospy.loginfo("executing plan with length %s..", len(self.plan))
            
            action = self.plan.pop(0)
            plan_remaining = self.plan
            if not self.battery_discharged:
                action_successfully_performed = self.perform_action(action)
            else:
                return 'hard_failure'
            
            if action_successfully_performed:
                completed_tasks += 1
                return 'success'
            else:
                return 'soft_failure'


class PlanExecutionStateMachine(smach.StateMachine):
    def __init__(self):

        super(PlanExecutionStateMachine, self).__init__(
            outcomes=['operation', 'catastrophe', 'contingency'],
            input_keys=[],
            output_keys=[]
        )

        with self:
            self.add('IDLE', Idle(),
                    transitions={'idle':'IDLE',
                                 'execute_plan':'EXECUTE_PLAN'})

            self.add('EXECUTE_PLAN', ExecutePlan(),
                    transitions={'complete':'operation',
                                 'soft_failure':'contingency',
                                 'hard_failure':'catastrophe',
                                 'success':'EXECUTE_PLAN',
                                 'preemption':'IDLE'})
