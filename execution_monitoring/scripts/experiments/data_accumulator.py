#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @author Tim Bohne

import errno
import json
from datetime import datetime
from signal import signal, SIGPIPE, SIG_DFL

import rospy
from arox_performance_parameters.msg import arox_operational_param, arox_battery_params
from mongodb_store.message_store import MessageStoreProxy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from execution_monitoring import config


class DataAccumulator:
    """
    Captures the relevant data during LTA missions and stores it in a MongoDB database.
    To achieve systematic and reusable logging and general data acquisition, the `mongodb_store` is used.
    """

    def __init__(self):
        # prevents broken pipe issue when logging during LTA experiments
        signal(SIGPIPE, SIG_DFL)
        self.msg_store = self.operation_start_time = self.op_info = self.charge_info = None
        self.init()

    def init(self):
        """
        (Re)initializes the data accumulator.
        """
        self.msg_store = MessageStoreProxy()
        self.operation_start_time = datetime.now()
        self.op_info = None
        self.charge_info = None

        rospy.Subscriber('/show_db_entries', String, self.show_db_entries, queue_size=1)
        rospy.Subscriber('/contingency_preemption', String, self.contingency_callback, queue_size=1)
        rospy.Subscriber('/catastrophe_preemption', String, self.catastrophe_callback, queue_size=1)
        rospy.Subscriber(config.OPERATION_TOPIC, arox_operational_param, self.operation_callback, queue_size=1)
        rospy.Subscriber('/robot_info', String, self.info_callback, queue_size=1)
        rospy.Subscriber('/sim_info', String, self.sim_info_callback, queue_size=1)
        rospy.Subscriber('/action_info', String, self.action_info_callback, queue_size=1)
        rospy.Subscriber('/operator_communication', String, self.operator_communication_callback, queue_size=1)
        rospy.Subscriber('/resolution', String, self.resolution_callback, queue_size=1)
        rospy.Subscriber(config.BATTERY_TOPIC, arox_battery_params, self.battery_callback, queue_size=1)

    def battery_callback(self, msg):
        """
        Called whenever new information about the battery parameters arrive.

        @param msg: callback message - battery parameters
        """
        self.charge_info = msg

    def operation_callback(self, msg):
        """
        Called whenever new information about the ongoing operation arrive.

        @param msg: callback message - operational parameters
        """
        self.op_info = msg
        mission_info = {'operation_mode': msg.operation_mode, 'remaining_tasks': msg.total_tasks,
                        'ongoing_task': msg.ongoing_task, 'completed_tasks': msg.rewards_gained,
                        'operation_time': str((datetime.now() - self.operation_start_time).total_seconds()) + "s"}
        try:
            self.msg_store.insert_named("mission_info", String(json.dumps(mission_info)))
        except IOError as e:
            if e.errno == errno.EPIPE:
                if config.VERBOSE_LOGGING:
                    rospy.loginfo("DB entry error -- re-initializing data accumulator")
                self.init()
        except rospy.ServiceException as e:
            if config.VERBOSE_LOGGING:
                rospy.loginfo("log mission info -- DB entry failed - trying again: %s", e)
            self.operation_callback(msg)
        except AttributeError as e:
            if config.VERBOSE_LOGGING:
                rospy.loginfo("ERROR: %s", e)
                rospy.loginfo("entered: %s - trying again", String(json.dumps(mission_info)))
            self.operation_callback(msg)

    def resolution_callback(self, msg):
        """
        Called whenever a failure resolution takes place.

        @param msg: callback message - resolution info
        """
        try:
            self.msg_store.insert_named("resolution", msg)
        except IOError as e:
            if e.errno == errno.EPIPE:
                if config.VERBOSE_LOGGING:
                    rospy.loginfo("DB entry error -- re-initializing data accumulator")
                self.init()
        except rospy.ServiceException as e:
            if config.VERBOSE_LOGGING:
                rospy.loginfo("resolution callback -- DB entry failed - trying again: %s, msg: %s", e, msg)
            self.resolution_callback(msg)
        except AttributeError as e:
            if config.VERBOSE_LOGGING:
                rospy.loginfo("ERROR: %s", e)
                rospy.loginfo("entered: %s - trying again", msg)
            self.resolution_callback(msg)

    def operator_communication_callback(self, msg):
        """
        Called whenever operator communication takes place.

        @param msg: callback message - robot-human communication info
        """
        try:
            self.msg_store.insert_named("operator_communication", msg)
        except IOError as e:
            if e.errno == errno.EPIPE:
                if config.VERBOSE_LOGGING:
                    rospy.loginfo("DB entry error -- re-initializing data accumulator")
                self.init()
        except rospy.ServiceException as e:
            if config.VERBOSE_LOGGING:
                rospy.loginfo("operator communication callback -- DB entry failed - trying again: %s, msg: %s", e, msg)
            self.operator_communication_callback(msg)
        except AttributeError as e:
            if config.VERBOSE_LOGGING:
                rospy.loginfo("ERROR: %s", e)
                rospy.loginfo("entered: %s - trying again", msg)
            self.operator_communication_callback(msg)

    def action_info_callback(self, msg):
        """
        Called whenever an action is performed.

        @param msg: callback message - action info
        """
        try:
            self.msg_store.insert_named("action_info", msg)
        except IOError as e:
            if e.errno == errno.EPIPE:
                if config.VERBOSE_LOGGING:
                    rospy.loginfo("DB entry error -- re-initializing data accumulator")
                self.init()
        except rospy.ServiceException as e:
            if config.VERBOSE_LOGGING:
                rospy.loginfo("action info callback -- DB entry failed - trying again: %s, msg: %s", e, msg)
            self.action_info_callback(msg)
        except AttributeError as e:
            if config.VERBOSE_LOGGING:
                rospy.loginfo("ERROR: %s", e)
                rospy.loginfo("entered: %s - trying again", String(msg))
            self.action_info_callback(msg)

    def sim_info_callback(self, msg):
        """
        Called whenever new failure situations are simulated.

        @param msg: callback message - simulation info
        """
        try:
            self.msg_store.insert_named("sim_info", msg)
        except IOError as e:
            if e.errno == errno.EPIPE:
                if config.VERBOSE_LOGGING:
                    rospy.loginfo("DB entry error -- re-initializing data accumulator")
                self.init()
        except rospy.ServiceException as e:
            if config.VERBOSE_LOGGING:
                rospy.loginfo("sim info callback -- DB entry failed - trying again: %s, msg: %s", e, msg)
            self.sim_info_callback(msg)
        except AttributeError as e:
            if config.VERBOSE_LOGGING:
                rospy.loginfo("ERROR: %s", e)
                rospy.loginfo("entered: %s - trying again", msg)
            self.sim_info_callback(msg)

    def log_failure_circumstances(self):
        """
        Logs the circumstances of failure situations, as these provide valuable information to learn from.
        """
        nav_sat_fix = None
        try:
            nav_sat_fix = rospy.wait_for_message('/fix', NavSatFix, timeout=10)
        except IOError as e:
            if e.errno == errno.EPIPE:
                if config.VERBOSE_LOGGING:
                    rospy.loginfo("DB entry error -- re-initializing data accumulator")
                self.init()
        except rospy.ROSException as e:
            rospy.loginfo("problem retrieving GNSS fix: %s", e)

        # only meta information that are not already published by the monitoring procedures
        circumstances = {}
        if nav_sat_fix:
            circumstances['robot_pose'] = "lat: " + str(nav_sat_fix.latitude) + ", lng: " + str(nav_sat_fix.longitude)
        else:
            circumstances['robot_pose'] = "---"
        circumstances['completed_tasks'] = self.op_info.rewards_gained
        circumstances['operation_time'] = str((datetime.now() - self.operation_start_time).total_seconds()) + "s"
        circumstances['charge'] = self.charge_info.charge
        circumstances['charge_cycle'] = self.charge_info.charging_cycle

        try:
            self.msg_store.insert_named("failure_circumstances", String(json.dumps(circumstances)))
        except IOError as e:
            if e.errno == errno.EPIPE:
                if config.VERBOSE_LOGGING:
                    rospy.loginfo("DB entry error -- re-initializing data accumulator")
                self.init()
        except rospy.ServiceException as e:
            if config.VERBOSE_LOGGING:
                rospy.loginfo("log failure circumstances -- DB entry failed - trying again: %s", e)
            self.log_failure_circumstances()
        except AttributeError as e:
            if config.VERBOSE_LOGGING:
                rospy.loginfo("ERROR: %s - trying again", e)
            self.log_failure_circumstances()

    def contingency_callback(self, msg):
        """
        Called in contingency situations.

        @param msg: callback message - reason for contingency
        """
        try:
            self.msg_store.insert_named("contingency", msg)
            self.log_failure_circumstances()
        except IOError as e:
            if e.errno == errno.EPIPE:
                if config.VERBOSE_LOGGING:
                    rospy.loginfo("DB entry error -- re-initializing data accumulator")
                self.init()
        except rospy.ServiceException as e:
            if config.VERBOSE_LOGGING:
                rospy.loginfo("contingency callback -- DB entry failed - trying again: %s, msg: %s", e, msg)
            self.contingency_callback(msg)
        except AttributeError as e:
            if config.VERBOSE_LOGGING:
                rospy.loginfo("ERROR: %s", e)
                rospy.loginfo("entered: %s - trying again", msg)
            self.contingency_callback(msg)

    def catastrophe_callback(self, msg):
        """
        Called in catastrophe situations.

        @param msg: callback message - reason for catastrophe
        """
        try:
            self.msg_store.insert_named("catastrophe", msg)
            self.log_failure_circumstances()
        except IOError as e:
            if e.errno == errno.EPIPE:
                if config.VERBOSE_LOGGING:
                    rospy.loginfo("DB entry error -- re-initializing data accumulator")
                self.init()
        except rospy.ServiceException as e:
            if config.VERBOSE_LOGGING:
                rospy.loginfo("catastrophe callback -- DB entry failed - trying again: %s, msg: %s", e, msg)
            self.catastrophe_callback(msg)
        except AttributeError as e:
            if config.VERBOSE_LOGGING:
                rospy.loginfo("ERROR: %s", e)
                rospy.loginfo("entered: %s - trying again", msg)
            self.catastrophe_callback(msg)

    def info_callback(self, msg):
        """
        Called whenever new status information about the robot and its environment arrive.

        @param msg: callback message - robot / environment status information
        """
        try:
            self.msg_store.insert_named("robot_info", msg)
        except IOError as e:
            if e.errno == errno.EPIPE:
                if config.VERBOSE_LOGGING:
                    rospy.loginfo("DB entry error -- re-initializing data accumulator")
                self.init()
        except rospy.ServiceException as e:
            if config.VERBOSE_LOGGING:
                rospy.loginfo("info callback -- DB entry failed - trying again: %s, msg: %s", e, msg)
            self.info_callback(msg)
        except AttributeError as e:
            if config.VERBOSE_LOGGING:
                rospy.loginfo("ERROR: %s", e)
                rospy.loginfo("entered: %s - trying again", msg)
            self.info_callback(msg)

    def show_db_entries(self, msg):
        """
        Shows entries currently stored in the database.

        @param msg: callback message - specifying category to be displayed
        """
        # no category specified -> query entire database
        if msg.data == "complete":
            rospy.loginfo("showing database entries:")
            entries = self.msg_store.query(String._type)
        # query database with the specified category
        else:
            rospy.loginfo("showing database entries for category: %s", msg.data)
            entries = [(data, meta) for data, meta in self.msg_store.query(String._type) if meta['name'] == msg.data]

        for entry in entries:
            data, meta = entry
            rospy.loginfo("data: %s", data.data)
            rospy.loginfo("name: %s, inserted_at: %s", meta['name'], meta['inserted_at'])
            rospy.loginfo("------------------------------")


def node():
    """
    Data accumulation node.
    """
    rospy.init_node('data_accumulator')
    rospy.loginfo("launch LTA data accumulator node..")
    DataAccumulator()
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
