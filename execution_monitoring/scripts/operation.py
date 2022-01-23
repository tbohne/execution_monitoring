#!/usr/bin/env python
import smach
import actionlib
import rospy
from plan_generation.srv import get_plan
from arox_navigation_flex.msg import drive_to_goalAction
from execution_monitoring.msg import ScanAction, ScanGoal
from arox_performance_parameters.msg import arox_operational_param
from arox_performance_parameters.msg import arox_battery_params
from std_msgs.msg import String, UInt16
from datetime import datetime
from execution_monitoring import config, util


class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['waiting_for_plan', 'plan_received', 'end_of_episode_signal', 'external_problem'], 
                             input_keys=['input_plan'],
                             output_keys=['output_plan'])
        
        self.mission_name_pub = rospy.Publisher('/mission_name', String, queue_size=1)
        self.exception_pub = rospy.Publisher('/plan_retrieval_failure', UInt16, queue_size=1)

    def get_plan(self):
        try:
            rospy.wait_for_service('arox_planner/get_plan', timeout=10)
            res = rospy.ServiceProxy('arox_planner/get_plan', get_plan)()
            if res.succeeded:
                return res.generated_plan.actions
            return None
        except rospy.ROSException as e:
            rospy.loginfo("exception during plan retrieval: %s", e)
            self.exception_pub.publish(config.PLAN_RETRIEVAL_TIMEOUT_CODE)
            return None

    def execute(self, userdata):
        rospy.loginfo("executing IDLE state..")

        if self.preempt_requested():
            rospy.loginfo("external problem detected by monitoring procedures - preempting normal operation..")
            self.service_preempt()
            return 'external_problem'

        # if LTA episode ends: return "end_of_episode_signal", e.g. via topic

        if len(userdata.input_plan) > 0:
            rospy.loginfo("input_plan: %s", userdata.input_plan)
            rospy.loginfo("continuing preempted plan..")
            userdata.output_plan = userdata.input_plan
            return "plan_received"

        rospy.loginfo("waiting for available plan from plan provider..")
        plan = self.get_plan()
        if plan is not None:
            rospy.loginfo("received plan..")

            if len(plan) == 0:
                self.exception_pub.publish(config.EMPTY_PLAN_CODE)

            for i in range(len(plan)):
                if plan[i].name not in config.FEASIBLE_ACTIONS:
                    rospy.loginfo("infeasible action: %s", plan[i].name)
                    self.exception_pub.publish(config.INFEASIBLE_PLAN_CODE)

            userdata.output_plan = plan
            self.mission_name_pub.publish(datetime.fromtimestamp(rospy.get_time()).strftime("%m_%d_%Y"))
            return "plan_received"
        else:
            rospy.sleep(10)
            rospy.loginfo("waiting for plan..")
            return "waiting_for_plan"


class ExecutePlan(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['action_completed', 'plan_completed', 'soft_failure', 'hard_failure', 'external_problem'],
                             input_keys=['plan'])

        rospy.Subscriber('/interrupt_active_goals', String, self.interrupt_active_goals, queue_size=1)
        rospy.Subscriber('/arox/battery_param', arox_battery_params, self.battery_callback, queue_size=1)
        self.drive_to_goal_client = actionlib.SimpleActionClient('drive_to_goal', drive_to_goalAction)
        self.scan_client = actionlib.SimpleActionClient('dummy_scanner', ScanAction)
        self.operation_pub = rospy.Publisher('arox/ongoing_operation', arox_operational_param, queue_size=1)
        self.battery_discharged = False
        self.remaining_tasks = 0
        self.completed_tasks = 0
        self.latest_action = None

    def interrupt_active_goals(self, msg):
        self.drive_to_goal_client.cancel_all_goals()
        self.scan_client.cancel_all_goals()

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

    def perform_action(self, action):
        rospy.loginfo("performing action %s..", action.name)

        # moving actions
        if action.name == "drive_to" or action.name == "return_to_base":

            self.publish_state_of_ongoing_operation("moving")

            if action.name == "return_to_base":
                action.pose = config.BASE_POSE

            action_goal = util.create_dtg_goal(action.pose, None)
            self.drive_to_goal_client.wait_for_server()
            self.drive_to_goal_client.send_goal(action_goal)
            rospy.loginfo("goal sent, wait for accomplishment..")
            success = self.drive_to_goal_client.wait_for_result()

            out = self.drive_to_goal_client.get_result()
            if out.progress > 0:
                rospy.loginfo("driving goal progress: %s", out.progress)
                return False

            rospy.loginfo("successfully performed action: %s", success)
            return success

        elif action.name == "scan":
            self.publish_state_of_ongoing_operation("scanning")
            rospy.loginfo("start scanning procedure..")
            action_goal = ScanGoal()
            self.scan_client.wait_for_server()
            self.scan_client.send_goal(action_goal)
            rospy.loginfo("goal sent, wait for accomplishment..")
            success = self.scan_client.wait_for_result()
            rospy.loginfo("successfully performed action: %s", success)
            rospy.loginfo(self.scan_client.get_result())
            return success

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

        self.remaining_tasks = len(userdata.plan)

        if len(userdata.plan) == 0:
            rospy.loginfo("plan successfully executed..")
            return "plan_completed"
        else:
            rospy.loginfo("executing plan - remaining actions: %s", len(userdata.plan))
            action = userdata.plan.pop(0)
            self.latest_action = action
            if not self.battery_discharged:
                action_successfully_performed = self.perform_action(action)
            else:
                rospy.loginfo("hard failure..")
                return "hard_failure"

            if self.preempt_requested():
                rospy.loginfo("external problem detected by monitoring procedures - preempting normal operation..")
                # last action should be repeated
                userdata.plan.insert(0, self.latest_action)
                self.service_preempt()
                return 'external_problem'

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
            outcomes=['minor_complication', 'critical_complication', 'end_of_episode', 'preempted'],
            input_keys=[],
            output_keys=[]
        )

        self.userdata.sm_input = []

        with self:
            self.add('IDLE', Idle(),
                     transitions={'waiting_for_plan': 'IDLE',
                                  'plan_received': 'EXECUTE_PLAN',
                                  'end_of_episode_signal': 'end_of_episode',
                                  'external_problem': 'preempted'},
                     remapping={'input_plan': 'sm_input',
                                'output_plan': 'sm_input'})

            self.add('EXECUTE_PLAN', ExecutePlan(),
                     transitions={'action_completed': 'EXECUTE_PLAN',
                                  'plan_completed': 'IDLE',
                                  'soft_failure': 'minor_complication',
                                  'hard_failure': 'critical_complication',
                                  'external_problem': 'preempted'},
                     remapping={'plan': 'sm_input'})
