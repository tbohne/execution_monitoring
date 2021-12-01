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
CONNECTION_FAILURE_SIX = "internet connection: bad download speed"
CONNECTION_FAILURE_SEVEN = "internet connection: bad upload speed"
BAD_WIFI_LINK_QUALITY = 2
BAD_WIFI_SIGNAL_LEVEL = -90
BAD_WIFI_BIT_RATE = 0.1
WIFI_DISCONNECT = 0
BAD_DOWNLOAD = 0.5
BAD_UPLOAD = 0.5
GPS_TIMEOUT = 60

# GNSS STATUS OPTIONS
GNSS_STATUS_NO_FIX = -1  # unable to find position
GNSS_STATUS_FIX = 0      # unaugmented fix - found a location, using solely GPS/GLONASS/etc.
GNSS_STATUS_SBAS_FIX = 1 # with satellite-based augmentation - fix with assistance of such networks as StarFire
GNSS_STATUS_GBAS_FIX = 2 # with ground-based augmentation - fix with assistance of such networks as DGPS or GBAS
# GNSS SERVICE OPTIONS
GNSS_SERVICE_GPS = 1
GNSS_SERVICE_GLONASS = 2
GNSS_SERVICE_COMPASS = 4
GNSS_SERVICE_GALILEO = 8
# GNSS COVARIANCE TYPES
GNSS_COVARIANCE_TYPE_UNKNOWN = 0
GNSS_COVARIANCE_TYPE_APPROXIMATED = 1
GNSS_COVARIANCE_TYPE_DIAGONAL_KNOWN = 2
GNSS_COVARIANCE_TYPE_KNOWN = 3
# user config
GNSS_STATUS = GNSS_STATUS_FIX
GNSS_SERVICE = GNSS_SERVICE_GPS
GNSS_COVARIANCES = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
GNSS_COVARIANCE_TYPE = GNSS_COVARIANCE_TYPE_UNKNOWN

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
