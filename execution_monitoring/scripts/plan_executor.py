#!/usr/bin/env python
import smach
import actionlib 
import rospy
import time
import numpy as np
from osgeo import osr
from tf.transformations import quaternion_from_euler
from arox_planning.srv import get_plan ,set_plan
from arox_navigation_flex.msg import drive_to_goalAction
from arox_navigation_flex.msg import drive_to_goalGoal as dtg_Goal
from std_msgs.msg import String
from arox_planning.msg import arox_action
from arox_performance_parameters.msg import arox_operational_param
from arox_performance_parameters.msg import arox_battery_params

plan_remaining = []
plan_initial = []
battery_dead = False
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

    def set_plan(self):
        rospy.wait_for_service('arox_planner/set_fakeplan')
        try:
            if rospy.ServiceProxy('arox_planner/set_fakeplan', set_plan)().succeeded:
                return True
            return None
        except rospy.ServiceException as e:
            print("service call failed: %s", e)
            return None

    def get_plan(self):
        rospy.wait_for_service('arox_planner/get_plan')
        try:
            res = rospy.ServiceProxy('arox_planner/get_plan', get_plan)()
            if res.succeeded:
                return res.plan.actions
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
            self.set_plan()
            plan = self.get_plan()
            
            if plan != None:
                plan_initial = plan[:]
                userdata.plan = plan
                plan_remaining = plan_initial
                publish_state_of_ongoing_operation("undocking")
                return 'execute_plan'
            return 'wait_for_plan'
         
        else:
            if completed_tasks == len(plan_initial):
                rospy.loginfo("completed plan, waiting for new plan..")
                time.sleep(3)
                return 'wait_for_plan'
               
            else:
                rospy.loginfo("continuing preempted plan..")
                publish_state_of_ongoing_operation("undocking")
                userdata.plan = plan_remaining
                return 'execute_plan'


class ExecutePlan(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['finished', 'failed', 'action_accomplished', 'preempted', 'failed_bizarre', 'should_charge', 'shutdown_execution'],
                                   input_keys=['plan'])

        rospy.Subscriber("/arox/battery_param", arox_battery_params, self.battery_callback, queue_size=1)

    def battery_callback(self, data):
        global battery_dead, plan_remaining

        if data.charge == 0.0:
            battery_dead = True
            rospy.loginfo("battery completely discharged..")
            plan_remaining_length = len(self.plan) + 1
            plan_remaining = plan_initial[-plan_remaining_length:]
            publish_state_of_ongoing_operation("dead")

            client = actionlib.SimpleActionClient('drive_to_goal', drive_to_goalAction)
            client.cancel_all_goals()
            return 'shutdown_execution'

    @staticmethod
    def perform_action(action):
        """
        TODO: add doc
        """
        global battery_dead

        rospy.loginfo("performing action %s with task %s..", action.name, action.task)

        if action.name == "drive_to" or action.name == "drive_to_container":

            if action.name == "drive_to_container":
                action = arox_action()
                action.str_args.append("wgs84")
                action.flt_args.append(52.3203191407)
                action.flt_args.append(8.153625154949)
                action.flt_args.append(270)
                action.task="charge"

            publish_state_of_ongoing_operation("processing")
            
            client = actionlib.SimpleActionClient('drive_to_goal', drive_to_goalAction)
            action_goal = dtg_Goal()
            action_goal.target_pose.header.frame_id= 'utm'
            if action.str_args[0] == "wgs84":
                source = osr.SpatialReference()
                source.ImportFromEPSG(4326)
                target = osr.SpatialReference()
                target.ImportFromEPSG(32632)
                transform = osr.CoordinateTransformation(source, target)
                x, y = transform.TransformPoint(action.flt_args[1], action.flt_args[0])[0:2] # switch lat / lon
                action_goal.target_pose.pose.position.x = x
                action_goal.target_pose.pose.position.y = y
            else:
                # unsupported coordinates
                return

            q = quaternion_from_euler(0, 0, np.pi * (action.flt_args[2] + 90) / 180)
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

            if battery_dead:
                return str('dead')
            elif success:
                if action.task == "charge":
                    return str('near_container')
                else:
                    return str('traverse success')

        else:
            rospy.logerr("unknown action..")
                
    def execute(self, userdata):
        global plan_remaining, completed_tasks
        self.plan = userdata.plan
        
        if self.preempt_requested():
            return 'preempted'
        elif len(self.plan) == 0:            
            return 'finished'
        else:
            rospy.loginfo("executing plan with length %s..", len(self.plan))
            
            action = self.plan.pop(0)
            plan_remaining = self.plan
            status = self.perform_action(action)
            
            if status == 'dead':
                return 'shutdown_execution'
            elif status == 'near_container':
                publish_state_of_ongoing_operation("docking")
                rospy.sleep(1)
                completed_tasks += 1
                publish_state_of_ongoing_operation("charging")
                return 'should_charge'
            elif status == 'traverse success':
                completed_tasks += 1
                return 'action_accomplished'
            elif status == 'call_help':
                return 'failed_bizarre'


class PlanExecutionStateMachine(smach.StateMachine):
    def __init__(self):

        super(PlanExecutionStateMachine, self).__init__(
            outcomes=['catastrophe', 'contingency', 'operation', 'shutdown', 'preempted', 'dock'],
            input_keys=[],
            output_keys=[]
        )

        with self:
            self.add('IDLE', Idle(),
                    transitions={'idle':'IDLE',
                                 'execute_plan':'EXECUTE_PLAN'})

            self.add('EXECUTE_PLAN', ExecutePlan(),
                    transitions={'finished':'IDLE',
                                 'failed':'contingency',
                                 'failed_bizarre':'catastrophe',
                                 'action_accomplished':'EXECUTE_PLAN',
                                 'should_charge':'dock',
                                 'shutdown_execution':'shutdown'})
