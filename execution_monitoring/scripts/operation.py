#!/usr/bin/env python
import smach
import actionlib
import rospy
from plan_generation.srv import get_plan
from geometry_msgs.msg import PoseStamped
from arox_navigation_flex.msg import drive_to_goalAction
from execution_monitoring.msg import ScanAction, ScanGoal
from arox_performance_parameters.msg import arox_operational_param
from arox_performance_parameters.msg import arox_battery_params
from arox_docking.msg import DockAction, UndockAction, UndockGoal, DockGoal
from std_msgs.msg import String, UInt16
from datetime import datetime
from execution_monitoring import config, util
from plan_generation.msg import plan, action
from mbf_msgs.msg import RecoveryAction, RecoveryGoal
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry


class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['waiting_for_plan', 'plan_received', 'end_of_episode_signal', 'external_problem'],
                             input_keys=['input_plan'],
                             output_keys=['output_plan'])
        
        self.mission_name_pub = rospy.Publisher('/mission_name', String, queue_size=1)
        self.exception_pub = rospy.Publisher('/plan_retrieval_failure', UInt16, queue_size=1)
        self.action_info_pub = rospy.Publisher('/action_info', String, queue_size=1)
        self.robot_info_pub = rospy.Publisher('/robot_info', String, queue_size=1)
        self.operation_pub = rospy.Publisher('arox/ongoing_operation', arox_operational_param, queue_size=1)

    def publish_state_of_ongoing_operation(self, mode):
        msg = arox_operational_param()
        msg.operation_mode = mode
        msg.total_tasks = 0
        msg.ongoing_task = 0
        msg.rewards_gained = 0
        self.operation_pub.publish(msg)

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
            rospy.sleep(2)
            return None

    def execute(self, userdata):
        rospy.loginfo("executing IDLE state..")
        self.action_info_pub.publish("executing IDLE state..")

        self.publish_state_of_ongoing_operation("waiting")

        if self.preempt_requested():
            rospy.loginfo("external problem detected by monitoring procedures - preempting normal operation..")
            self.robot_info_pub.publish("external problem detected by monitoring procedures - preempting normal operation..")
            self.publish_state_of_ongoing_operation("contingency")
            self.service_preempt()
            return 'external_problem'

        # if LTA episode ends: return "end_of_episode_signal", e.g. via topic

        if len(userdata.input_plan) > 0:
            rospy.loginfo("input_plan: %s", userdata.input_plan)
            rospy.loginfo("continuing preempted plan..")
            self.robot_info_pub.publish("continuing preempted plan: " + str(userdata.input_plan))
            userdata.output_plan = userdata.input_plan
            return "plan_received"

        rospy.loginfo("waiting for available plan from plan provider..")
        plan = self.get_plan()

        if plan is not None:
            rospy.loginfo("received plan..")

            if len(plan) == 0:
                rospy.loginfo("empty plan..")
                self.exception_pub.publish(config.EMPTY_PLAN_CODE)
                rospy.sleep(2)
                return "waiting_for_plan"
            for i in range(len(plan)):
                if plan[i].name not in config.FEASIBLE_ACTIONS:
                    rospy.loginfo("infeasible action: %s", plan[i].name)
                    self.exception_pub.publish(config.INFEASIBLE_PLAN_CODE)
                    rospy.sleep(2)
                    return "waiting_for_plan"

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
        rospy.Subscriber('introduce_intermediate_nav_goal', String, self.introduce_intermediate_nav_goal, queue_size=1)
        rospy.Subscriber('introduce_intermediate_recharge_goal', String, self.introduce_intermediate_recharge_goal, queue_size=1)
        rospy.Subscriber('/sim_docking_failure_base_pose', String, self.sim_docking_fail_callback, queue_size=1)
        rospy.Subscriber('/sim_charging_failure', String, self.sim_charge_fail_callback, queue_size=1)

        self.drive_to_goal_client = actionlib.SimpleActionClient('drive_to_goal', drive_to_goalAction)
        self.scan_client = actionlib.SimpleActionClient('dummy_scanner', ScanAction)
        self.operation_pub = rospy.Publisher('arox/ongoing_operation', arox_operational_param, queue_size=1)
        self.nav_fail_pub = rospy.Publisher('/explicit_nav_failure', String, queue_size=1)
        self.charge_fail_pub = rospy.Publisher('/explicit_charging_failure', String, queue_size=1)
        self.charge_action_pub = rospy.Publisher('/charge_action', String, queue_size=1)
        self.action_info_pub = rospy.Publisher('/action_info', String, queue_size=1)
        self.robot_info_pub = rospy.Publisher('/robot_info', String, queue_size=1)
        self.sim_info_pub = rospy.Publisher('/sim_info', String, queue_size=1)
        self.fully_charged_pub = rospy.Publisher('/fully_charged', String, queue_size=1)

        self.robot_pose = None
        self.pose_in_front_of_container = None
        self.pose_sub = rospy.Subscriber("/odometry/filtered_odom", Odometry, self.odom_callback, queue_size=1)

        self.activate_localization_pub = rospy.Publisher('/activate_localization_monitoring', String, queue_size=1)
        self.deactivate_localization_pub = rospy.Publisher('/deactivate_localization_monitoring', String, queue_size=1)

        self.sim_docking_fail = False
        self.sim_charge_fail = False

        self.battery_discharged = False
        self.remaining_tasks = 0
        self.completed_tasks = 0
        self.introduce_nav_goal = False
        self.introduce_recharge_goal = False
        self.intermediate_nav_goal_pose = None
        self.latest_action = None

    def sim_docking_fail_callback(self, msg):
        rospy.loginfo("preparing simulation of docking failure..")
        self.sim_docking_fail = True

    def sim_charge_fail_callback(self, msg):
        rospy.loginfo("preparing simulation of charging failure..")
        self.sim_charge_fail = True

    def odom_callback(self, odom):
        """
        Is called whenever new odometry data arrives.

        :param odom: odometry data to update robot pos with
        """
        pose = PoseStamped()
        pose.header.frame_id = odom.header.frame_id
        pose.pose = odom.pose.pose
        self.robot_pose = pose

    def introduce_intermediate_nav_goal(self, msg):
        rospy.loginfo("preparing introduction of intermediate nav goal - code: %s", msg.data)
        self.introduce_nav_goal = True
        if msg.data == "0":
            self.intermediate_nav_goal_pose = config.DOCKING_BASE_POSE if config.DOCKING else config.BASE_POSE
        elif msg.data == "1":
            self.intermediate_nav_goal_pose = config.STREET
        elif msg.data == "2":
            self.intermediate_nav_goal_pose = config.FIELD

    def introduce_intermediate_recharge_goal(self, msg):
        rospy.loginfo("preparing introduction of intermediate recharge goal..")
        self.introduce_recharge_goal = True

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
            self.publish_state_of_ongoing_operation("dead")
            self.robot_info_pub.publish("battery completely discharged..")

    def perform_action(self, action):

        # moving actions
        if action.name == "drive_to" or action.name == "return_to_base":

            self.publish_state_of_ongoing_operation("traversing")

            if action.name == "return_to_base" and config.DOCKING:
                if self.sim_docking_fail:
                    rospy.loginfo("simulating docking fail -- wrong container pos..")
                    self.sim_info_pub.publish("simulating docking fail -- wrong container pos")
                    action.pose = config.DOCKING_BASE_POSE_FAIL
                    self.sim_docking_fail = False
                else:
                    action.pose = config.DOCKING_BASE_POSE

            elif action.name == "return_to_base":
                action.pose = config.BASE_POSE

            rospy.loginfo("performing action %s with target pose: %s", action.name, str(action.pose))
            self.action_info_pub.publish("performing action " + str(action.name) + " with target pose: " + str(action.pose))

            action_goal = util.create_dtg_goal(action.pose, None)

            self.drive_to_goal_client.wait_for_server()
            self.drive_to_goal_client.send_goal(action_goal)
            rospy.loginfo("goal sent, wait for accomplishment..")
            success = self.drive_to_goal_client.wait_for_result()
            out = self.drive_to_goal_client.get_result()
            rospy.loginfo("out: %s, state: %s, succ: %s, txt: %s", out,  self.drive_to_goal_client.get_state(), success, self.drive_to_goal_client.get_goal_status_text())

            # explicit navigation failure -- goal not reached -- trigger nav monitoring
            if self.drive_to_goal_client.get_state() == config.GOAL_STATUS_ABORTED:
                rospy.loginfo("explicit nav failure -- reporting to nav monitoring..")
                self.nav_fail_pub.publish("")
                rospy.sleep(5)
                return False
            elif self.drive_to_goal_client.get_state() == config.GOAL_STATUS_PREEMPTED:
                rospy.loginfo("DRIVE TO GOAL PREEMPTED..")
                # wait for external contingency to be executed
                rospy.sleep(10)
                return False

            if out.progress > 0:
                rospy.loginfo("driving goal progress: %s", out.progress)
                return False

            # in container scenarios "return_to_base" includes docking
            if action.name == "return_to_base" and config.DOCKING:
                self.pose_in_front_of_container = self.robot_pose
                if not self.dock_to_charging_station():
                    return False

            rospy.loginfo("successfully performed action: %s", success)
            return success

        elif action.name == "scan":
            rospy.loginfo("performing action %s at pose: %s", action.name, str(self.robot_pose.pose))
            self.action_info_pub.publish("performing action " + str(action.name) + " at pose: " + str(self.robot_pose.pose))
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
            rospy.loginfo("performing action %s", action.name)
            self.action_info_pub.publish("performing action " + str(action.name))
            self.publish_state_of_ongoing_operation("charging")

            # notify charge failure monitoring that charging starts
            self.charge_action_pub.publish("")

            charge_mode = False

            if self.sim_charge_fail:
                rospy.loginfo("simulating charging failure..")
                self.sim_info_pub.publish("simulating charging failure")
                self.sim_charge_fail = False
                # just sleep for some time to trigger charge fail
                rospy.sleep(config.CHARGING_FAILURE_TIME + 5)
                return False
            else:
                rospy.set_param("charging_mode", True)
                charge_mode = rospy.get_param("charging_mode")

            while charge_mode:
                charge_mode = rospy.get_param("charging_mode")
                rospy.loginfo("charging battery..")
                rospy.sleep(2)

            if not charge_mode:
                rospy.loginfo("battery charged..")
                self.fully_charged_pub.publish("")
                self.robot_info_pub.publish("battery charged..")
                return self.undock_from_charging_station(self.pose_in_front_of_container)
        else:
            rospy.loginfo("error - unknown action: %s", action.name)
        return False

    def dock_to_charging_station(self):
        docking_client = actionlib.SimpleActionClient('dock_to_charging_station', DockAction)
        self.publish_state_of_ongoing_operation("docking")
        goal = DockGoal()
        goal.goal = "custom goal"
        docking_client.wait_for_server()
        rospy.loginfo("START DOCKING PROCEDURE..")
        self.robot_info_pub.publish("start docking procedure")
        self.deactivate_localization_pub.publish("")
        docking_client.send_goal(goal)
        rospy.loginfo("goal sent, wait for accomplishment..")

        # success just means that the smach execution has been successful, not the docking itself
        success = docking_client.wait_for_result()

        if success:
            rospy.loginfo("SMACH execution terminated successfully")
            rospy.loginfo("DOCKING PROCEDURE FINISHED: %s", docking_client.get_result().result_state)

            if docking_client.get_result().result_state == "success":
                rospy.loginfo("successfully docked to charging station..")
                self.robot_info_pub.publish("successfully docked to charging station..")
                return True
            else:
                rospy.loginfo("docking failure..")
                self.charge_fail_pub.publish(config.CHARGING_FAILURE_ONE)
                # sleep to let monitoring detect the problem
                rospy.sleep(5)
        else:
            rospy.loginfo("SMACH execution failed: %s", docking_client.get_goal_status_text())
            self.charge_fail_pub.publish(config.CHARGING_FAILURE_ONE)
            # sleep to let monitoring detect the problem
            rospy.sleep(5)
        return False

    def undock_from_charging_station(self, pose_in_front_of_container):
        undocking_client = actionlib.SimpleActionClient('undock_from_charging_station', UndockAction)
        self.publish_state_of_ongoing_operation("undocking")
        goal = UndockGoal()
        goal.ramp_alignment_pose = pose_in_front_of_container
        undocking_client.wait_for_server()
        rospy.loginfo("START UNDOCKING PROCEDURE..")
        self.robot_info_pub.publish("start undocking procedure")
        undocking_client.send_goal(goal)
        rospy.loginfo("goal sent, wait for accomplishment")
        success = undocking_client.wait_for_result()
        if success:
            rospy.loginfo("SMACH execution terminated successfully")
            rospy.loginfo("UNDOCKING PROCEDURE FINISHED: %s", undocking_client.get_result().result_state)

            if undocking_client.get_result().result_state == "success":
                rospy.loginfo("successfully undocked from charging station..")
                self.robot_info_pub.publish("successfully undocked from charging station..")
                # after docking-undocking done - reactivate localization monitoring
                self.activate_localization_pub.publish("")
                # clear costmap due to ramp movement
                self.clear_costmaps()
                return True
            else:
                rospy.loginfo("undocking failed..")
                self.charge_fail_pub.publish(config.CHARGING_FAILURE_TWO)
                rospy.sleep(5)
        else:
            rospy.loginfo("SMACH execution failed: %s", undocking_client.get_goal_status_text())
            self.charge_fail_pub.publish(config.CHARGING_FAILURE_TWO)
            rospy.sleep(5)
        return False

    def clear_costmaps(self):
        rospy.wait_for_service('/move_base_flex/clear_costmaps')
        clear_costmaps_service = rospy.ServiceProxy('/move_base_flex/clear_costmaps', Empty)
        rec_client = actionlib.SimpleActionClient("move_base_flex/recovery", RecoveryAction)
        rec_client.wait_for_server()

        # order matters -> first global, then local
        rospy.loginfo("clearing global costmap..")
        self.robot_info_pub.publish("clearing global costmap..")
        try:
            clear_costmaps_service()
        except rospy.ServiceException as e:
            rospy.loginfo("error: %s", e)

        rospy.loginfo("clearing local costmap..")
        self.robot_info_pub.publish("clearing local costmap..")
        # concurrency_slot 3
        clear_local_costmap_goal = RecoveryGoal('clear_costmap', 3)
        rec_client.send_goal(clear_local_costmap_goal)
        res = rec_client.wait_for_result()
        if res:
            rospy.loginfo("cleared costmap..")

    def execute(self, userdata):
        rospy.loginfo("executing EXECUTE_PLAN state..")
        self.publish_state_of_ongoing_operation("processing")

        self.remaining_tasks = len(userdata.plan)

        if len(userdata.plan) == 0:
            rospy.loginfo("plan successfully executed..")
            return "plan_completed"
        else:
            rospy.loginfo("executing plan - remaining actions: %s", len(userdata.plan))

            if self.introduce_recharge_goal:
                rospy.loginfo("introducing intermediate recharge goal [return_to_base, charge]")
                self.robot_info_pub.publish("introducing intermediate recharge goal [return_to_base, charge]")
                self.introduce_recharge_goal = False
                # first return to base, then charge
                a = action()
                a.name = "charge"
                userdata.plan.insert(0, a)
                a = action()
                a.name = "return_to_base"
                userdata.plan.insert(0, a)

            a = userdata.plan.pop(0)
            self.latest_action = a
            if not self.battery_discharged:
                action_successfully_performed = self.perform_action(a)
            else:
                rospy.loginfo("hard failure..")
                self.robot_info_pub.publish("hard failure -- battery discharged")
                self.publish_state_of_ongoing_operation("catastrophe")
                return "hard_failure"

            if self.preempt_requested():
                rospy.loginfo("external problem detected by monitoring procedures - preempting normal operation..")
                self.robot_info_pub.publish("external problem detected by monitoring procedures - preempting normal operation..")
                # last action should be repeated
                userdata.plan.insert(0, self.latest_action)
                self.publish_state_of_ongoing_operation("contingency")
                self.service_preempt()
                return 'external_problem'

            if action_successfully_performed:
                rospy.loginfo("%s action successfully completed - executing rest of plan..", a.name)
                self.robot_info_pub.publish("action " + a.name + " successfully completed - executing rest of plan..")
                rospy.sleep(2)

                if self.introduce_nav_goal and self.intermediate_nav_goal_pose is not None:
                    rospy.loginfo("introducing intermediate nav goal..")
                    self.robot_info_pub.publish("introducing intermediate nav goal..")
                    self.introduce_nav_goal = False
                    a = action()
                    a.name = "drive_to"
                    a.pose = self.intermediate_nav_goal_pose
                    userdata.plan.insert(0, a)
                self.completed_tasks += 1
                return "action_completed"
            else:
                rospy.loginfo("soft failure..")
                self.robot_info_pub.publish("soft failure -- action not successfully performed")
                self.publish_state_of_ongoing_operation("contingency")
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
