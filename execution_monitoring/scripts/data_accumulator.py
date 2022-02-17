#!/usr/bin/env python
import time
from datetime import datetime
import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from geometry_msgs.msg import Pose, Point, Quaternion
from arox_performance_parameters.msg import arox_operational_param
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import json


class DataAccumulator:

    def __init__(self):
        self.msg_store = MessageStoreProxy()

        self.operation_start_time = datetime.now()
        self.op_info = None

        rospy.Subscriber('/show_db_entries', String, self.show_db_entries, queue_size=1)
        rospy.Subscriber('/contingency_preemption', String, self.contingency_callback, queue_size=1)
        rospy.Subscriber('/catastrophe_preemption', String, self.catastrophe_callback, queue_size=1)
        rospy.Subscriber('/arox/ongoing_operation', arox_operational_param, self.operation_callback, queue_size=1)
        rospy.Subscriber('/robot_info', String, self.info_callback, queue_size=1)
        rospy.Subscriber('/sim_info', String, self.sim_info_callback, queue_size=1)
        rospy.Subscriber('/action_info', String, self.action_info_callback, queue_size=1)
        rospy.Subscriber('/operator_communication', String, self.operator_communication_callback, queue_size=1)
        rospy.Subscriber('/resolution', String, self.resolution_callback, queue_size=1)

    def operation_callback(self, msg):
        self.op_info = msg

    def resolution_callback(self, msg):
        rospy.loginfo("saving resolution data in DB..")
        try:
            self.msg_store.insert_named("resolution", msg)
        except rospy.ServiceException as e:
            rospy.loginfo("service call failed: %s", e)

    def operator_communication_callback(self, msg):
        rospy.loginfo("saving operator communication data in DB..")
        try:
            self.msg_store.insert_named("operator_communication", msg)
        except rospy.ServiceException as e:
            rospy.loginfo("service call failed: %s", e)

    def action_info_callback(self, msg):
        rospy.loginfo("saving action info data in DB..")
        try:
            self.msg_store.insert_named("action_info", msg)
        except rospy.ServiceException as e:
            rospy.loginfo("service call failed: %s", e)

    def sim_info_callback(self, msg):
        rospy.loginfo("saving sim info data in DB..")
        try:
            self.msg_store.insert_named("sim_info", msg)
        except rospy.ServiceException as e:
            rospy.loginfo("service call failed: %s", e)

    def log_failure_circumstances(self):
        rospy.loginfo("saving failure circumstances in DB..")

        nav_sat_fix = None
        try:
            nav_sat_fix = rospy.wait_for_message('/fix', NavSatFix, timeout=10)
        except rospy.ROSException as e:
            rospy.loginfo("problem retrieving GNSS fix: %s", e)

        # only meta information that are not already published by the monitoring procedures
        # TODO: can be arbitrarily extended
        circumstances = {}
        circumstances['robot_pose'] = "lat: " + str(nav_sat_fix.latitude) + ", lng: "  + str(nav_sat_fix.longitude)
        circumstances['completed_tasks'] = self.op_info.rewards_gained
        circumstances['operation_time'] = str((datetime.now() - self.operation_start_time).total_seconds()) + "s"

        try:
            self.msg_store.insert_named("failure_circumstances", String(json.dumps(circumstances)))
        except rospy.ServiceException as e:
            rospy.loginfo("service call failed: %s", e)

    def contingency_callback(self, msg):
        rospy.loginfo("saving contingency data in DB..")
        try:
            self.msg_store.insert_named("contingency", msg)
            self.log_failure_circumstances()
        except rospy.ServiceException as e:
            rospy.loginfo("service call failed: %s", e)

    def catastrophe_callback(self, msg):
        rospy.loginfo("saving catastrophe data in DB..")
        try:
            self.msg_store.insert_named("catastrophe", msg)
            self.log_failure_circumstances()
        except rospy.ServiceException as e:
            rospy.loginfo("service call failed: %s", e)

    def info_callback(self, msg):
        rospy.loginfo("saving robot info data in DB..")
        try:
            self.msg_store.insert_named("robot_info", msg)
        except rospy.ServiceException as e:
            rospy.loginfo("service call failed: %s", e)

    def show_db_entries(self, msg):
        rospy.loginfo("all DB entries of type String:")

        all_poses = self.msg_store.query(String._type)
        for i in all_poses:
            data, meta = i
            rospy.loginfo("data: %s", data.data)
            rospy.loginfo("name: %s, inserted_at: %s", meta['name'], meta['inserted_at'])
            rospy.loginfo("------------------------------")

def node():
    rospy.init_node('data_accumulator')
    rospy.loginfo("launch LTA data accumulator node..")
    DataAccumulator()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
