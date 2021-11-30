#!/usr/bin/env python

# SCAN FAILURES
SENSOR_FAILURE_ONE = "total sensor failure"
SENSOR_FAILURE_TWO = "empty list of range values"
SENSOR_FAILURE_THREE = "predominantly infeasible range values (inf)"
SENSOR_FAILURE_FOUR = "repeated scan"

# CONNECTION FAILURES
CONNECTION_FAILURE_ONE = "bad wifi link"
CONNECTION_FAILURE_TWO = "bad wifi signal"
CONNECTION_FAILURE_THREE = "bad wifi bitrate"
CONNECTION_FAILURE_FOUR = "wifi disconnect"
CONNECTION_FAILURE_FIVE = "internet disconnect"
CONNECTION_FAILURE_SIX = "bad download speed"
CONNECTION_FAILURE_SEVEN = "bad upload speed"
BAD_WIFI_LINK_QUALITY = 2
BAD_WIFI_SIGNAL_LEVEL = -90
BAD_WIFI_BIT_RATE = 0.1
WIFI_DISCONNECT = 0
BAD_DOWNLOAD = 0.5
BAD_UPLOAD = 0.5

# DATA MANAGEMENT FAILURES
DATA_MANAGEMENT_FAILURE_ONE = "full memory"
DATA_MANAGEMENT_FAILURE_TWO = "scan not logged correctly"

# DATA MANAGEMENT SETTINGS
MONITOR_DRIVE = "/"
ENABLE_SPECIFIC_LASER_SCAN_CHECK = True

# SCAN SETTINGS
SCAN_PATH = MONITOR_DRIVE + "/home/docker/catkin_ws/src/execution_monitoring/execution_monitoring/scans/"
SCAN_TIME_LIMIT = 60
SCAN_FILE_EXTENSION = ".txt"
SCAN_VALUES_LB_PERCENTAGE = 5

# WIFI MONITORING
WIFI_INTERFACE = "wlx3c1e045678a2"
