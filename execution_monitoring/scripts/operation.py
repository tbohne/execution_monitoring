#!/usr/bin/env python
from datetime import datetime

import actionlib
import rospy
import smach
from actionlib_msgs.msg import GoalStatus
from arox_docking.msg import DockAction, UndockAction, UndockGoal, DockGoal
from arox_navigation_flex.msg import drive_to_goalAction
from arox_performance_parameters.msg import arox_battery_params
from arox_performance_parameters.msg import arox_operational_param
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from plan_generation.msg import action
from plan_generation.srv import get_plan
from std_msgs.msg import String, UInt16, Bool

from execution_monitoring import config, util
from execution_monitoring.msg import ScanAction, ScanGoal


class Idle(smach.State):
    """
    State in the low-level operational SMACH representing situations where the robot is idle and awaiting orders.
    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['waiting_for_plan', 'plan_received', 'end_of_episode_signal',
                                       'external_problem'],
                             input_keys=['input_plan'],
                             output_keys=['output_plan'])

        self.end_of_episode = False
        self.mission_name_pub = rospy.Publisher('/mission_name', String, queue_size=1)
        self.exception_pub = rospy.Publisher('/plan_retrieval_failure', UInt16, queue_size=1)
        self.action_info_pub = rospy.Publisher('/action_info', String, queue_size=1)
        self.robot_info_pub = rospy.Publisher('/robot_info', String, queue_size=1)
        self.operation_pub = rospy.Publisher(config.OPERATION_TOPIC, arox_operational_param, queue_size=1)
        rospy.Subscriber("/end_of_episode", String, self.end_of_episode_callback, queue_size=1)

    def end_of_episode_callback(self, msg):
        """
        Callback that indicates an end of episode.

        @param msg: callback message
        """
        self.end_of_episode = True

    def publish_state_of_ongoing_operation(self, mode):
        """
        Publishes the state of the ongoing operation.

        @param mode: operation mode (scanning, traversing, waiting, (un)docking, charging, contingency, catastrophe)
        """
        msg = arox_operational_param()
        msg.operation_mode = mode
        # the number of tasks is disregarded here, it is just about the mode
        msg.total_tasks = msg.ongoing_task = msg.rewards_gained = 0
        self.operation_pub.publish(msg)

    def get_plan(self):
        """
        Retrieves a plan that defines an LTA mission for the robot to perform.

        @return: retrieved mission plan
        """
        try:
            rospy.wait_for_service('plan_generation/get_plan', timeout=10)
            res = rospy.ServiceProxy('plan_generation/get_plan', get_plan)()
            if res.succeeded:
                return res.generated_plan.actions
            return None
        except rospy.ROSException as e:
            rospy.loginfo("exception during plan retrieval: %s", e)
            self.exception_pub.publish(config.PLAN_RETRIEVAL_TIMEOUT_CODE)
            rospy.sleep(config.SHORT_DELAY)
            return None

    def execute(self, userdata):
        """
        Execution of 'IDLE' state - initiation of plan retrieval.

        @param userdata: input of the state
        @return: outcome of the state ("external_problem", "plan_received", "waiting_for_plan", "end_of_episode_signal")
        """
        rospy.loginfo("executing IDLE state..")
        self.action_info_pub.publish("executing IDLE state..")
        self.publish_state_of_ongoing_operation("waiting")

        if self.end_of_episode:
            return 'end_of_episode_signal'

        if self.preempt_requested():
            rospy.loginfo("external problem detected by monitoring procedures - preempting normal operation..")
            self.robot_info_pub.publish("external problem detected by mon. proc. - preempting normal operation..")
            self.publish_state_of_ongoing_operation("contingency")
            self.service_preempt()
            return 'external_problem'

        if len(userdata.input_plan) > 0:
            rospy.loginfo("input_plan: %s", userdata.input_plan)
            rospy.loginfo("continuing preempted plan..")
            self.robot_info_pub.publish("continuing preempted plan - remaining op.: " + str(len(userdata.input_plan)))
            userdata.output_plan = userdata.input_plan
            return "plan_received"

        rospy.loginfo("waiting for available plan from plan provider..")
        plan = self.get_plan()

        if plan:
            rospy.loginfo("received plan..")
            if len(plan) == 0:
                rospy.loginfo("empty plan..")
                self.exception_pub.publish(config.EMPTY_PLAN_CODE)
                rospy.sleep(config.SHORT_DELAY)
                return "waiting_for_plan"
            for i in range(len(plan)):
                if plan[i].name not in config.FEASIBLE_ACTIONS:
                    rospy.loginfo("infeasible action: %s", plan[i].name)
                    self.exception_pub.publish(config.INFEASIBLE_PLAN_CODE)
                    rospy.sleep(config.SHORT_DELAY)
                    return "waiting_for_plan"

            userdata.output_plan = plan
            self.mission_name_pub.publish(datetime.fromtimestamp(rospy.get_time()).strftime("%m_%d_%Y"))
            return "plan_received"
        else:
            rospy.sleep(config.PLAN_CHECK_FREQ)
            rospy.loginfo("waiting for plan..")
            return "waiting_for_plan"


class ExecutePlan(smach.State):
    """
    State in the low-level operational SMACH representing situations where the robot executes a given plan.
    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['action_completed', 'plan_completed', 'soft_failure', 'hard_failure',
                                       'external_problem'],
                             input_keys=['plan'])

        self.robot_pose = None
        self.pose_in_front_of_container = None
        self.sim_docking_fail = False
        self.sim_charge_fail = False
        self.waiting = False
        self.battery_charge = None
        self.battery_discharged = False
        self.introduce_nav_goal = False
        self.introduce_recharge_goal = False
        self.introduce_shelter_goal = False
        self.intermediate_nav_goal_pose = None
        self.latest_action = None
        self.remaining_tasks = 0
        self.completed_tasks = 0

        self.drive_to_goal_client = actionlib.SimpleActionClient('drive_to_goal', drive_to_goalAction)
        self.scan_client = actionlib.SimpleActionClient('dummy_scanner', ScanAction)
        self.docking_client = actionlib.SimpleActionClient('dock_to_charging_station', DockAction)
        self.undocking_client = actionlib.SimpleActionClient('undock_from_charging_station', UndockAction)

        self.operation_pub = rospy.Publisher(config.OPERATION_TOPIC, arox_operational_param, queue_size=1)
        self.nav_fail_pub = rospy.Publisher('/explicit_nav_failure', String, queue_size=1)
        self.charge_fail_pub = rospy.Publisher('/explicit_charging_failure', String, queue_size=1)
        self.charge_action_pub = rospy.Publisher('/charge_action', String, queue_size=1)
        self.action_info_pub = rospy.Publisher('/action_info', String, queue_size=1)
        self.robot_info_pub = rospy.Publisher('/robot_info', String, queue_size=1)
        self.sim_info_pub = rospy.Publisher('/sim_info', String, queue_size=1)
        self.fully_charged_pub = rospy.Publisher('/fully_charged', String, queue_size=1)
        self.activate_localization_pub = rospy.Publisher('/activate_localization_monitoring', String, queue_size=1)
        self.deactivate_localization_pub = rospy.Publisher('/deactivate_localization_monitoring', String, queue_size=1)
        self.loc_pub = rospy.Publisher('/resolve_localization_failure_success', Bool, queue_size=1)

        rospy.Subscriber("/odometry/filtered_odom", Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber('/interrupt_active_goals', String, self.interrupt_active_goals, queue_size=1)
        rospy.Subscriber('/arox/battery_param', arox_battery_params, self.battery_callback, queue_size=1)
        rospy.Subscriber('introduce_intermediate_nav_goal', String, self.introduce_intermediate_nav_goal, queue_size=1)
        rospy.Subscriber('introduce_intermediate_recharge_goal', String, self.intro_inter_recharge_goal, queue_size=1)
        rospy.Subscriber('introduce_intermediate_shelter_goal', String, self.intro_inter_shelter_goal, queue_size=1)
        rospy.Subscriber('/sim_docking_failure_base_pose', String, self.sim_docking_fail_callback, queue_size=1)
        rospy.Subscriber('/sim_charging_failure', String, self.sim_charge_fail_callback, queue_size=1)
        rospy.Subscriber('/stop_waiting', String, self.stop_waiting_callback, queue_size=1)

    def stop_waiting_callback(self, msg):
        """
        Callback setting the flag to stop waiting.

        @param msg: callback message
        """
        self.waiting = False

    def sim_docking_fail_callback(self, msg):
        """
        Callback that initiates a docking failure based on an incorrectly set base pose.

        @param msg: callback message
        """
        rospy.loginfo("preparing simulation of docking failure..")
        self.sim_docking_fail = True

    def sim_charge_fail_callback(self, msg):
        """
        Callback that initiates a charging failure.

        @param msg: callback message
        """
        rospy.loginfo("preparing simulation of charging failure..")
        self.sim_charge_fail = True

    def odom_callback(self, odom):
        """
        Called whenever new odometry data arrives - used to keep track of the robot pose.

        :param odom: odometry data to update robot pose with
        """
        pose = PoseStamped()
        pose.header.frame_id = odom.header.frame_id
        pose.pose = odom.pose.pose
        self.robot_pose = pose

    def introduce_intermediate_nav_goal(self, msg):
        """
        Callback to introduce intermediate navigation goals into the plan.

        @param msg: callback message - specifying the particular intermediate goal (out of three options)
        """
        rospy.loginfo("preparing introduction of intermediate nav goal - code: %s", msg.data)
        self.introduce_nav_goal = True
        if msg.data == "0":
            self.intermediate_nav_goal_pose = config.DOCKING_BASE_POSE if config.DOCKING else config.BASE_POSE
        elif msg.data == "1":
            self.intermediate_nav_goal_pose = config.STREET
        elif msg.data == "2":
            self.intermediate_nav_goal_pose = config.FIELD

    def intro_inter_shelter_goal(self, msg):
        """
        Callback to introduce intermediate shelter goals into the plan.
        E.g. used for extreme weather situations to let the robot wait inside the container.

        @param msg: callback message
        """
        rospy.loginfo("preparing introduction of intermediate shelter goal..")
        self.introduce_shelter_goal = True

    def intro_inter_recharge_goal(self, msg):
        """
        Callback to introduce intermediate recharge goals into the plan.

        @param msg: callback message
        """
        rospy.loginfo("preparing introduction of intermediate recharge goal..")
        self.introduce_recharge_goal = True

    def interrupt_active_goals(self, msg):
        """
        Callback that interrupts all active goals of the robot.

        @param msg: callback message
        """
        self.drive_to_goal_client.cancel_all_goals()
        self.scan_client.cancel_all_goals()
        self.docking_client.cancel_all_goals()
        self.undocking_client.cancel_all_goals()

    def publish_state_of_ongoing_operation(self, mode):
        """
        Publishes the state of the robot's ongoing operation.

        @param mode: current operation mode
        """
        msg = arox_operational_param()
        msg.operation_mode = mode
        msg.total_tasks = self.remaining_tasks
        msg.ongoing_task = self.completed_tasks + 1
        msg.rewards_gained = self.completed_tasks
        self.operation_pub.publish(msg)

    def battery_callback(self, data):
        """
        Callback receiving current information about the robot's battery parameters.

        @param data: current battery parameters
        """
        self.battery_charge = data.charge
        if data.charge == 0.0:
            self.battery_discharged = True
            rospy.loginfo("battery completely discharged..")
            self.publish_state_of_ongoing_operation("dead")
            self.robot_info_pub.publish("battery completely discharged..")

    def perform_moving_action(self, action):
        """
        Performs a moving action ('return_to_base' or 'drive_to').

        @param action: moving action to be performed
        @return: whether the moving action was completed successfully
        """
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
        self.action_info_pub.publish("performing act. " + str(action.name) + " with target pose: " + str(action.pose))

        action_goal = util.create_nav_goal(action.pose, None)
        self.drive_to_goal_client.wait_for_server()
        self.drive_to_goal_client.send_goal(action_goal)
        rospy.loginfo("goal sent, waiting for completion..")
        success = self.drive_to_goal_client.wait_for_result()
        out = self.drive_to_goal_client.get_result()
        rospy.loginfo("out: %s, state: %s, succ: %s, txt: %s", out, self.drive_to_goal_client.get_state(), success,
                      self.drive_to_goal_client.get_goal_status_text())

        # explicit navigation failure -- goal not reached -- trigger nav monitoring
        if self.drive_to_goal_client.get_state() == GoalStatus.ABORTED:
            rospy.loginfo("explicit nav failure -- reporting to nav monitoring..")
            self.nav_fail_pub.publish("")
            rospy.sleep(config.ERROR_SLEEP_TIME)
            return False
        elif self.drive_to_goal_client.get_state() == GoalStatus.PREEMPTED:
            rospy.loginfo("DRIVE TO GOAL PREEMPTED..")
            # wait for external contingency to be executed
            rospy.sleep(config.PREEMPTION_SLEEP_TIME)
            return False
        elif out.progress > 0:
            rospy.loginfo("driving goal progress: %s", out.progress)
            return False

        # in container scenarios, "return_to_base" includes docking
        if action.name == "return_to_base" and config.DOCKING:
            self.pose_in_front_of_container = self.robot_pose
            if not self.dock_to_charging_station():
                return False

        rospy.loginfo("successfully performed action: %s", success)
        return success

    def perform_scan_action(self, action):
        """
        Performs a scan action.

        @param action: scan action to be performed
        @return: whether the scan action was completed successfully
        """
        rospy.loginfo("performing action %s at pose: %s", action.name, str(self.robot_pose.pose))
        self.action_info_pub.publish("performing action " + str(action.name) + " at pose: " + str(self.robot_pose.pose))
        self.publish_state_of_ongoing_operation("scanning")
        rospy.loginfo("start scanning procedure..")
        action_goal = ScanGoal()
        self.scan_client.wait_for_server()
        self.scan_client.send_goal(action_goal)
        rospy.loginfo("goal sent, waiting for completion..")
        success = self.scan_client.wait_for_result()
        rospy.loginfo("successfully performed action: %s", success)
        rospy.loginfo(self.scan_client.get_result())
        if self.scan_client.get_result() == "scanning failed":
            # sleep for monitoring -> afterwards, low-level failure
            rospy.sleep(config.PREEMPTION_SLEEP_TIME)
            return False
        return success

    def perform_wait_in_shelter_action(self, action):
        """
        Performs a 'wait in shelter' action.

        @param action: wait in shelter action to be performed
        @return: whether the action was completed successfully
        """
        rospy.loginfo("performing action %s", action.name)
        self.action_info_pub.publish("performing action " + str(action.name))
        self.publish_state_of_ongoing_operation("waiting")
        self.waiting = True
        while self.waiting:
            rospy.sleep(config.WAIT_SLEEP_TIME)
        return self.undock_from_charging_station(self.pose_in_front_of_container)

    def perform_charge_action(self, action, next_action):
        """
        Performs a charge action.

        @param action: charge action to be performed
        @param next_action: action that follows the charge action in the plan
        @return: whether the charge action was completed successfully
        """
        rospy.loginfo("performing action %s", action.name)
        self.action_info_pub.publish("performing action " + str(action.name))
        self.publish_state_of_ongoing_operation("charging")
        # notify charge failure monitoring that charging starts
        self.charge_action_pub.publish("")

        if self.sim_charge_fail:
            rospy.loginfo("simulating charging failure..")
            self.sim_info_pub.publish("simulating charging failure")
            self.sim_charge_fail = False
            # just sleep for some time to trigger charge fail
            rospy.sleep(config.CHARGING_FAILURE_TIME + config.ERROR_SLEEP_TIME)
            return False
        else:
            rospy.set_param("charging_mode", True)
            charge_mode = rospy.get_param("charging_mode")

        while charge_mode:
            charge_mode = rospy.get_param("charging_mode")
            rospy.loginfo("charging battery..")
            rospy.sleep(config.CHARGE_SLEEP_TIME)

        if not charge_mode:
            rospy.loginfo("battery charged..")
            self.fully_charged_pub.publish("")
            self.robot_info_pub.publish("battery charged..")
            # if the next action is to wait in the container, the robot shouldn't undock
            if next_action:
                return True if next_action.name == "wait_in_shelter" else self.undock_from_charging_station(
                    self.pose_in_front_of_container)
            return self.undock_from_charging_station(self.pose_in_front_of_container)

    def perform_action(self, action, next_action):
        """
        Executes an action of the currently pursued plan of the robot.

        @param action: action to be performed
        @param next_action: action to be performed after the currently considered one
        """
        if action.name == "drive_to" or action.name == "return_to_base":
            return self.perform_moving_action(action)
        elif action.name == "scan":
            return self.perform_scan_action(action)
        elif action.name == "wait_in_shelter":
            return self.perform_wait_in_shelter_action(action)
        elif action.name == "charge":
            return self.perform_charge_action(action, next_action)
        else:
            rospy.loginfo("error - unknown action: %s", action.name)
        return False

    def dock_to_charging_station(self):
        """
        Docks the robot to the inductive charging station inside the mobile container.

        @return: whether the docking was completed successfully
        """
        self.publish_state_of_ongoing_operation("docking")
        goal = DockGoal()
        goal.goal = "custom goal"
        self.docking_client.wait_for_server()
        rospy.loginfo("START DOCKING PROCEDURE..")
        self.robot_info_pub.publish("start docking procedure")
        self.docking_client.send_goal(goal)
        rospy.loginfo("goal sent, waiting for completion..")

        # possibility to consider simulated localization failures
        rospy.sleep(config.WAIT_BEFORE_DEACTIVATING_LOC_MON)
        # localization monitoring has to be deactivated here, as it does not make sense inside of the container
        self.deactivate_localization_pub.publish("")
        # success just means that the smach execution has been successful, not the docking itself
        success = self.docking_client.wait_for_result()

        if success:
            rospy.loginfo("SMACH execution terminated successfully")
            rospy.loginfo("DOCKING PROCEDURE FINISHED: %s", self.docking_client.get_result().result_state)

            if self.docking_client.get_result().result_state == "success":
                rospy.loginfo("successfully docked to charging station..")
                self.robot_info_pub.publish("successfully docked to charging station..")
                return True
            else:
                rospy.loginfo("docking failure..")
                self.charge_fail_pub.publish(config.CHARGING_FAILURES[0])
                # sleep to let monitoring detect the problem
                rospy.sleep(config.ERROR_SLEEP_TIME)
        else:
            rospy.loginfo("SMACH execution failed: %s", self.docking_client.get_goal_status_text())
            self.charge_fail_pub.publish(config.CHARGING_FAILURES[0])
            # sleep to let monitoring detect the problem
            rospy.sleep(config.ERROR_SLEEP_TIME)
        return False

    def undock_from_charging_station(self, pose_in_front_of_container):
        """
        Undocks the robot from the inductive charging station inside the mobile container.

        @param pose_in_front_of_container: pose the robot should take in front of the container
        @return: whether the undocking was completed successfully
        """
        self.publish_state_of_ongoing_operation("undocking")
        goal = UndockGoal()
        goal.ramp_alignment_pose = pose_in_front_of_container
        self.undocking_client.wait_for_server()
        rospy.loginfo("START UNDOCKING PROCEDURE..")
        self.robot_info_pub.publish("start undocking procedure")
        self.undocking_client.send_goal(goal)
        rospy.loginfo("goal sent, waiting for completion")
        success = self.undocking_client.wait_for_result()
        if success:
            rospy.loginfo("SMACH execution terminated successfully")
            rospy.loginfo("UNDOCKING PROCEDURE FINISHED: %s", self.undocking_client.get_result().result_state)

            if self.undocking_client.get_result().result_state == "success":
                rospy.loginfo("successfully undocked from charging station..")
                self.robot_info_pub.publish("successfully undocked from charging station..")
                # after undocking is completed, reactivate localization monitoring
                self.loc_pub.publish(True)
                # clear costmaps due to ramp movement
                self.robot_info_pub.publish("clearing costmaps..")
                util.clear_costmaps()
                return True
            else:
                rospy.loginfo("undocking failed..")
                self.charge_fail_pub.publish(config.CHARGING_FAILURES[1])
                rospy.sleep(config.ERROR_SLEEP_TIME)
        else:
            rospy.loginfo("SMACH execution failed: %s", self.undocking_client.get_goal_status_text())
            self.charge_fail_pub.publish(config.CHARGING_FAILURES[1])
            rospy.sleep(config.ERROR_SLEEP_TIME)
        return False

    def intermediate_recharge_goal(self, userdata):
        """
        Introduces an intermediate recharge goal.

        @param userdata: I/O of the 'EXECUTE_PLAN' state.
        """
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

    def intermediate_shelter_goal(self, userdata):
        """
        Introduces an intermediate shelter goal.

        @param userdata: I/O of the 'EXECUTE_PLAN' state.
        """
        rospy.loginfo("introducing intermediate shelter goal [return_to_base, charge, wait]")
        self.robot_info_pub.publish("introducing intermediate shelter goal [return_to_base, charge, wait]")
        self.introduce_shelter_goal = False
        # first return to base, then charge, then wait
        a = action()
        a.name = "wait_in_shelter"
        userdata.plan.insert(0, a)
        a = action()
        a.name = "charge"
        userdata.plan.insert(0, a)
        a = action()
        a.name = "return_to_base"
        userdata.plan.insert(0, a)

    def intermediate_nav_goal(self, userdata):
        """
        Introduces an intermediate navigation goal.

        @param userdata: I/O of the 'EXECUTE_PLAN' state.
        """
        rospy.loginfo("introducing intermediate nav goal..")
        self.robot_info_pub.publish("introducing intermediate nav goal..")
        self.introduce_nav_goal = False
        a = action()
        a.name = "drive_to"
        a.pose = self.intermediate_nav_goal_pose
        userdata.plan.insert(0, a)

    def execute(self, userdata):
        """
        Execution of 'EXECUTE_PLAN' state - processing of a given plan.

        @param userdata: input of the state
        @return: outcome of the state ("action_completed", "plan_completed", "soft_failure", "hard_failure",
                                       "external_problem")
        """
        rospy.loginfo("executing EXECUTE_PLAN state..")
        self.remaining_tasks = len(userdata.plan)

        if len(userdata.plan) == 0:
            rospy.loginfo("plan successfully executed..")
            return "plan_completed"
        else:
            rospy.loginfo("executing plan - remaining actions: %s", len(userdata.plan))
            # potential introduction of intermediate goals
            if self.introduce_recharge_goal:
                self.intermediate_recharge_goal(userdata)
            elif self.introduce_shelter_goal:
                self.intermediate_shelter_goal(userdata)

            a = userdata.plan.pop(0)
            next_action = userdata.plan[0] if len(userdata.plan) > 0 else None

            if a.name == "return_to_base" and next_action and next_action.name == "charge":
                # skip all of them when battery is already charged and the next action is not "wait_in_shelter"
                sec_next_action = userdata.plan[1] if len(userdata.plan) > 1 else None
                if sec_next_action and sec_next_action.name != "wait_in_shelter" \
                        and self.battery_charge > config.ALREADY_CHARGED_THRESH:
                    userdata.plan.pop(0)  # remove charge action
                    return "action_completed"  # and don't perform the redundant "return_to_base"

            # when the robot is interrupted during scanning, the "drive_to" action before also needs to be reattached
            drive_to_before_scan = None
            if a.name == "scan":
                drive_to_before_scan = self.latest_action

            self.latest_action = a
            if not self.battery_discharged:
                action_successfully_performed = self.perform_action(a, next_action)
            else:
                rospy.loginfo("hard failure..")
                self.robot_info_pub.publish("hard failure -- battery discharged")
                self.publish_state_of_ongoing_operation("catastrophe")
                return "hard_failure"

            if self.preempt_requested():
                rospy.loginfo("external problem detected by monitoring procedures - preempting normal operation..")
                self.robot_info_pub.publish(
                    "external problem detected by monitoring procedures - preempting normal operation..")
                # last action should be repeated
                userdata.plan.insert(0, self.latest_action)
                # if the preempted action was a "scan" action, the "drive_to" before is also required to be repeated
                if drive_to_before_scan:
                    userdata.plan.insert(0, drive_to_before_scan)
                self.publish_state_of_ongoing_operation("contingency")
                self.service_preempt()
                return 'external_problem'

            if action_successfully_performed:
                rospy.loginfo("%s action successfully completed - executing rest of plan..", a.name)
                self.robot_info_pub.publish("action " + a.name + " successfully completed - executing rest of plan..")
                rospy.sleep(config.SHORT_DELAY)
                if self.introduce_nav_goal and self.intermediate_nav_goal_pose is not None:
                    self.intermediate_nav_goal(userdata)
                self.completed_tasks += 1
                return "action_completed"
            else:
                rospy.loginfo("soft failure..")
                self.robot_info_pub.publish("soft failure -- action not successfully performed")
                self.publish_state_of_ongoing_operation("contingency")
                return "soft_failure"


class OperationStateMachine(smach.StateMachine):
    """
    Low-level operational state machine responsible for 'acting', i.e., execution of a given plan.
    """

    def __init__(self):
        super(OperationStateMachine, self).__init__(
            outcomes=['minor_complication', 'critical_complication', 'end_of_episode', 'preempted'],
            input_keys=[],
            output_keys=[]
        )

        self.userdata.sm_input = []

        # defines states and transitions of the low-level acting SMACH
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
