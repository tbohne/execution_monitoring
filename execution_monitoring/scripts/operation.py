#!/usr/bin/env python
import smach
import actionlib
import rospy
import numpy as np
from osgeo import osr
from tf.transformations import quaternion_from_euler
from plan_generation.srv import get_plan
from arox_navigation_flex.msg import drive_to_goalAction
from arox_navigation_flex.msg import drive_to_goalGoal as dtg_Goal
from arox_performance_parameters.msg import arox_operational_param
from arox_performance_parameters.msg import arox_battery_params

# TODO: put into parameter in launch file
BASE_POSE = [52.3203191407, 8.153625154949, 270]


class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['waiting_for_plan', 'plan_received', 'end_of_episode_signal'], input_keys=['input_plan'],
                             output_keys=['output_plan'])

    @staticmethod
    def get_plan():
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
        rospy.loginfo("executing IDLE state..")

        # if LTA episode ends: return "end_of_episode_signal", e.g. via topic

        if len(userdata.keys()) > 0 and len(userdata.input_plan) > 0:
            rospy.loginfo("continuing preempted plan..")
            userdata.output_plan = userdata.input_plan
            return "plan_received"

        rospy.loginfo("waiting for available plan from plan provider..")
        plan = self.get_plan()
        if plan is not None:
            rospy.loginfo("received plan..")
            userdata.output_plan = plan
            return "plan_received"
        else:
            rospy.loginfo("waiting for plan..")
            return "waiting_for_plan"


class ExecutePlan(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['action_completed', 'plan_completed', 'soft_failure', 'hard_failure'],
                             input_keys=['plan'])

        rospy.Subscriber('/arox/battery_param', arox_battery_params, self.battery_callback, queue_size=1)
        self.operation_pub = rospy.Publisher('arox/ongoing_operation', arox_operational_param, queue_size=1)
        self.battery_discharged = False
        self.remaining_tasks = 0
        self.plan = []
        self.completed_tasks = 0

    def publish_state_of_ongoing_operation(self, mode):
        msg = arox_operational_param()
        msg.operation_mode = mode
        msg.total_tasks = self.remaining_tasks
        msg.ongoing_task = self.completed_tasks + 1
        msg.rewards_gained = self.completed_tasks
        self.operation_pub.publish(msg)

    def battery_callback(self, data):
        if data.charge == 0.0:
            self.battery_discharged = True
            rospy.loginfo("battery completely discharged..")
            client = actionlib.SimpleActionClient('drive_to_goal', drive_to_goalAction)
            client.cancel_all_goals()

    def perform_action(self, action):
        global BASE_POSE
        rospy.loginfo("performing action %s..", action.name)

        # moving actions
        if action.name == "drive_to" or action.name == "return_to_base":

            self.publish_state_of_ongoing_operation("moving")

            if action.name == "return_to_base":
                action.pose = BASE_POSE

            client = actionlib.SimpleActionClient('drive_to_goal', drive_to_goalAction)
            action_goal = dtg_Goal()
            # TODO: could we use wgs84 here?
            action_goal.target_pose.header.frame_id = "utm"

            # the coordinates in the plan are wgs84 - transform
            source = osr.SpatialReference()
            source.ImportFromEPSG(4326)
            target = osr.SpatialReference()
            target.ImportFromEPSG(32632)
            transform = osr.CoordinateTransformation(source, target)
            x, y = transform.TransformPoint(action.pose[1], action.pose[0])[0:2]  # switch lat / lon
            action_goal.target_pose.pose.position.x = x
            action_goal.target_pose.pose.position.y = y

            q = quaternion_from_euler(0, 0, np.pi * (action.pose[2] + 90) / 180)
            action_goal.target_pose.pose.orientation.x = q[0]
            action_goal.target_pose.pose.orientation.y = q[1]
            action_goal.target_pose.pose.orientation.z = q[2]
            action_goal.target_pose.pose.orientation.w = q[3]
            client.wait_for_server()
            client.send_goal(action_goal)
            rospy.loginfo("goal sent, wait for accomplishment..")
            success = client.wait_for_result()
            rospy.loginfo("successfully performed action: %s", success)

            return success

        elif action.name == "scan":
            self.publish_state_of_ongoing_operation("scanning")
            rospy.loginfo("start scanning procedure..")
            rospy.sleep(3)
            rospy.loginfo("scanning procedure finished..")
            return True

        elif action.name == "charge":
            self.publish_state_of_ongoing_operation("charging")
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
        rospy.loginfo("executing EXECUTE_PLAN state..")

        self.plan = userdata.plan
        self.remaining_tasks = len(userdata.plan)

        if len(self.plan) == 0:
            rospy.loginfo("plan successfully executed..")
            return "plan_completed"
        else:
            rospy.loginfo("executing plan - remaining actions: %s", len(self.plan))
            action = self.plan.pop(0)
            if not self.battery_discharged:
                action_successfully_performed = self.perform_action(action)
            else:
                rospy.loginfo("hard failure..")
                return "hard_failure"

            if action_successfully_performed:
                rospy.loginfo("action successfully completed - executing rest of plan..")
                self.completed_tasks += 1
                return "action_completed"
            else:
                rospy.loginfo("soft failure..")
                return "soft_failure"


class OperationStateMachine(smach.StateMachine):
    def __init__(self):
        super(OperationStateMachine, self).__init__(
            outcomes=['minor_complication', 'critical_complication', 'end_of_episode'],
            input_keys=[],
            output_keys=[]
        )

        with self:
            self.add('IDLE', Idle(),
                     transitions={'waiting_for_plan': 'IDLE',
                                  'plan_received': 'EXECUTE_PLAN',
                                  'end_of_episode_signal': 'end_of_episode'},
                     remapping={'sm_input': 'input_plan',
                                'output_plan': 'sm_input'})

            self.add('EXECUTE_PLAN', ExecutePlan(),
                     transitions={'action_completed': 'EXECUTE_PLAN',
                                  'plan_completed': 'IDLE',
                                  'soft_failure': 'minor_complication',
                                  'hard_failure': 'critical_complication'},
                     remapping={'plan': 'sm_input'})
