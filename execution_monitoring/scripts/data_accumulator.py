#!/usr/bin/env python
import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String


class DataAccumulator:

    def __init__(self):
        self.msg_store = MessageStoreProxy()

        rospy.Subscriber('/show_db_entries', String, self.show_db_entries, queue_size=1)

        rospy.Subscriber('/contingency_preemption', String, self.contingency_callback, queue_size=1)
        rospy.Subscriber('/catastrophe_preemption', String, self.catastrophe_callback, queue_size=1)
        rospy.Subscriber('/robot_info', String, self.info_callback, queue_size=1)

    def contingency_callback(self, msg):
        rospy.loginfo("saving contingency data in DB..")
        try:
            self.msg_store.insert_named("contingency", msg)
        except rospy.ServiceException as e:
            rospy.loginfo("service call failed: %s", e)

    def catastrophe_callback(self, msg):
        rospy.loginfo("saving catastrophe data in DB..")
        try:
            self.msg_store.insert_named("catastrophe", msg)
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
