#!/usr/bin/env python

#############################################################################
################################### SETUP ###################################
#############################################################################
VERBOSE_LOGGING = False
DOCKING = True  # otherwise the simplified charging patch scenario is used

# target pose (lat, lng, theta) on the charging patch
BASE_POSE = [52.3203191407, 8.153625154949, 270]
# target pose (lat, lng, theta) in front of the base station
DOCKING_BASE_POSE = [52.3203930281, 8.15361381961, 180]

# maximum idle time without error in seconds
MISSION_IDLE_LIMIT = 900
# frequency at which new simulations are considered (sleep time in experiments loop)
EXPERIMENTS_CHECK_FREQ = 120
# used to avoid timing issues
SHORT_DELAY = 2  # in seconds

# directory in which the experimental results are stored
EXP_PATH = "/home/docker/catkin_ws/src/execution_monitoring/execution_monitoring/experiments/"
MISSION_FAIL_MSG = "MISSION FAIL: exceeded idle time limit during plan execution"

# crucial topics
OPERATION_TOPIC = "/arox/ongoing_operation"
GOAL_STATUS_TOPIC = "/move_base_flex/exe_path/status"
#############################################################################
#############################################################################
#############################################################################

#############################################################################
################################# RESOLUTION ################################
#############################################################################
# all in seconds
# ----------------
# frequency with which it is checked whether a problem has been solved
RESOLUTION_CHECK_FREQ = 5
# time after which a previous failure is considered independent of the new one
# when counting failures in recovery procedures
FAIL_OUTDATED_THRESH = 300
# sleep time introduced to give the high-level monitoring the chance to detect
# issues before the low-level SMACH
ERROR_SLEEP_TIME = 5
# sleep time introduced to not transition to a new state before a contingency
# is launched and initiated
PREEMPTION_SLEEP_TIME = 10
# frequency with which it is checked whether the robot should stop waiting
# in the shelter
WAIT_SLEEP_TIME = 5
#############################################################################
#############################################################################
#############################################################################

#############################################################################
############################## OBSTACLE SPAWNER #############################
#############################################################################
# height of the barrier models to be spawned as static obstacles
BARRIER_HEIGHT = 0.833558  # in meters
# distance of the barrier models to the robot
DIST_TO_ROBOT = 2.0  # in meters

STOP_SIGN_MODEL = "/home/docker/catkin_ws/src/execution_monitoring/models/stop_sign/model.sdf"
BARRIER_MODEL = "/home/docker/catkin_ws/src/execution_monitoring/models/jersey_barrier/model.sdf"

STOP_SIGN_POSES_SCENE_ONE = [
    [30.702585, -23.646406, 0.671698, 0.0, 0.0, 0.619839],
    [31.195600, -23.287600, 0.671698, 0.0, 0.0, 0.619839],
    [31.688100, -22.930900, 0.671698, 0.0, 0.0, 0.619839],
    [32.183200, -22.573900, 0.671698, 0.0, 0.0, 0.619839],
    [32.677800, -22.216500, 0.671698, 0.0, 0.0, 0.619839],
    [33.175500, -21.848200, 0.671698, 0.0, 0.0, 0.619839]
]
STOP_SIGN_POSES_SCENE_TWO = [
    [35.508858, -2.809350, 0.671698, 0.0, 0.0, 0.693925],
    [35.988500, -2.413980, 0.671698, 0.0, 0.0, 0.693925],
    [36.459300, -2.011220, 0.671698, 0.0, 0.0, 0.693925],
    [36.957800, -1.602300, 0.671698, 0.0, 0.0, 0.693925]
]
STOP_SIGN_POSES_SCENE_THREE = [
    [20.724546, -4.331711, 0.671698, 0.0, 0.0, 1.311019]
]
STOP_SIGN_POSES_SCENE_FOUR = [
    [0.382557, -2.998463, 1.468006, 0.0, 0.0, 2.896465],
    [-0.218533, -2.857140, 1.468010, 0.0, 0.0, 2.896465],
    [-0.821764, -2.707880, 1.468010, 0.0, 0.0, 2.896465],
    [-1.425310, -2.558510, 1.468010, 0.0, 0.0, 2.896465]
]
BARRIER_POSES_SCENE_ONE = [
    [28.351700, -23.712500, 0.782270, 0.0, 0.0, 0.0],
    [33.595978, -19.534710, 0.833558, 0.0, 0.0, 1.393767]
]
BARRIER_POSES_SCENE_TWO = [
    [33.112300, -3.033610, 0.833558, 0.0, 0.0, 0.0],
    [37.379513, 0.590976, 0.833558, 0.0, 0.0, -1.769417]
]
BARRIER_POSES_SCENE_THREE = [
    [20.170605, -6.542426, 0.833558, 0.0, 0.0, 1.377083]
]
BARRIER_POSES_SCENE_FOUR = [
    [2.577835, -4.190211, 1.411332, 0.0, 0.0, -0.519839],
    [-3.797018, -2.632295, 1.411332, 0.0, 0.0, 0.041027]
]
#############################################################################
#############################################################################
#############################################################################

#############################################################################
####################### POWER MANAGEMENT MONITORING #########################
#############################################################################
# contingency and catastrophe signals expected from the battery watchdog
CONTINGENCY_MSG = "CONT"
CATASTROPHE_MSG = "CATO"

# delay of the battery watchdog in seconds at startup of the system
# -- not really reliable at startup
INITIAL_SLEEP_TIME = 30
# frequency with which the charge level is checked during charging (in seconds)
CHARGE_SLEEP_TIME = 2
# battery charge percentage that is considered sufficient for skipping a
# charge action
ALREADY_CHARGED_THRESH = 95
# default discharge rate of the robot's battery
NORMAL_DISCHARGE_RATE = 0.03
# increased discharge rate used to trigger contingencies
CONTINGENCY_DISCHARGE_RATE = 0.35
# increased discharge rate used to trigger catastrophes
CATASTROPHE_DISCHARGE_RATE = 0.59

POWER_MANAGEMENT_FAILURES = {
    0: "immediate return to the base station required in order to still be able to reach it"
}
POWER_MANAGEMENT_CATA = "no longer possible to reach the base station based on battery charge level"
#############################################################################
#############################################################################
#############################################################################

#############################################################################
########################## NAVIGATION MONITORING ############################
#############################################################################
# max number of repeated recoveries without causing a sustained recovery case
RECOVERY_LIMIT = 30
# minimum distance (km) to be traveled during recovery to be considered progress
RECOVERY_PROGRESS_MIN_DIST_THRESH = 0.001
# frequency in seconds with which the monitoring for navigation failures takes place
NAV_MON_FREQ = 5

NAVIGATION_FAILURES = {
    0: "sustained recovery - navigation cannot recover",
    1: "sustained recovery - but still making progress - continuing for now",
    2: "explicit nav failure reported by low-level operation state machine"
}
NAV_CATA = "nav resolution failed -- catastrophe"

# scenario-specific waypoints
RECOVERY_POINT_ONE = [52.32056824755535, 8.153337579568582, 270]
STREET = [52.320786493558508, 8.153624127240558, 270]
FIELD = [52.32042638488258, 8.153084460244127, 270]
#############################################################################
#############################################################################
#############################################################################

#############################################################################
###################### CHARGING FAILURE MONITORING ##########################
#############################################################################
# invalid base pose to simulate docking failures
DOCKING_BASE_POSE_FAIL = [52.32059819361085, 8.153113603063638, 90]

# thresholds for (un)docking repetitions before calling human operator
DOCKING_FAIL_THRESH = 1
UNDOCKING_FAIL_THRESH = 1
# time in seconds after which charging without an increase in the charge level
# is considered a failure
CHARGING_FAILURE_TIME = 10  # in seconds
WAIT_BEFORE_DEACTIVATING_LOC_MON = 5  # in seconds

CHARGING_FAILURES = {
    0: "docking failure - explicit docking smach failure",
    1: "undocking failure - explicit undocking smach failure",
    2: "battery not charging although docked"
}
CHARGING_CATA = "charging resolution failed -- catastrophe"
#############################################################################
#############################################################################
#############################################################################

#############################################################################
######################## PLAN DEPLOYMENT MONITORING #########################
#############################################################################
# frequency in seconds with which the monitoring for plan failures takes place
PLAN_MON_FREQ = 10
# max time without executing an action that does not lead to a contingency
IDLE_THRESH = 300
# frequency in seconds with which plans are requested in idle state
PLAN_CHECK_FREQ = 10

# thresholds for plan retrieval repetitions before calling human operator
UNAVAIL_PLAN_SERVICE_THRESH = 1
INVALID_PLAN_THRESH = 1

PLAN_DEPLOYMENT_FAILURES = {
    0: "robot idle for an extended period of time",
    1: "plan retrieval service unavailable",
    2: "deployed plan empty",
    3: "deployed plan corrupted / infeasible",
    4: "unspecified plan deployment failure"
}
PLAN_DEPLOYMENT_CATA = "plan deployment resolution failed -- catastrophe"

# list of feasible actions in the considered scenario
FEASIBLE_ACTIONS = ["drive_to", "return_to_base", "charge", "scan"]

# plan deployment error codes
PLAN_RETRIEVAL_TIMEOUT_CODE = 0
EMPTY_PLAN_CODE = 1
INFEASIBLE_PLAN_CODE = 2
#############################################################################
#############################################################################
#############################################################################

#############################################################################
######################## DATA MANAGEMENT MONITORING #########################
#############################################################################
# threshold for scan repetitions before calling human operator
REPEAT_SCAN_THRESH = 1

# contingency case
FULL_MEMORY_THRESH = 99.0
# info cases
ALMOST_FULL_MEMORY_THRESH_ONE = 90.0
ALMOST_FULL_MEMORY_THRESH_TWO = 95.0

# drive to be monitored
MONITOR_DRIVE = "/"
# path under which a full USB drive should be mounted during experiments
FULL_DRIVE = "/mnt/usb"
# whether only general capacity monitoring should take place
# (or also specific scan checks)
ENABLE_SPECIFIC_LASER_SCAN_CHECK = True

DATA_MANAGEMENT_FAILURES = {
    0: "full memory",
    1: "scan not logged correctly"
}
DATA_MANAGEMENT_CATA = "data management resolution failed -- catastrophe"
#############################################################################
#############################################################################
#############################################################################

#############################################################################
############################# SENSOR MONITORING #############################
#############################################################################
# time it takes to generate a simulated scan (simulated scanning time)
SCAN_TIME = 4
# topic on which the simulated scans are expected to arrive
SCAN_TOPIC = "/RIEGL"
# path to the directory where the recorded scans should be saved
SCAN_PATH = MONITOR_DRIVE + "home/docker/catkin_ws/src/execution_monitoring/execution_monitoring/scans/"
# max scanning time without causing a contingency
SCAN_TIME_LIMIT = 30
# file extension of the saved scan logs
SCAN_FILE_EXTENSION = ".txt"
# minimum percentage of feasible range values in a recorded scan
# without causing a contingency
SCAN_VALUES_LB_PERCENTAGE = 5

SENSOR_FAILURES = {
    0: "total sensor failure",
    1: "empty list of range values",
    2: "predominantly infeasible range values (inf)",
    3: "repeated scan"
}
SENSOR_CATA = "sensor resolution failed -- catastrophe"
#############################################################################
#############################################################################
#############################################################################

#############################################################################
########################## LOCALIZATION MONITORING ##########################
#############################################################################
# monitoring delay in seconds after a transition from one navigational status
# to another (e.g. ACTIVE -> SUCCEEDED)
STATUS_SWITCH_DELAY = 4

# thresholds in meters for different categories of estimated distance
# divergences between GNSS and odometry
ODOMETRY_GNSS_DIST_HEAVY_DIV_THRESH = 2.0  # causing contingency
ODOMETRY_GNSS_DIST_DIV_CONTINGENCY_THRESH = 1.5  # causing contingency
ODOMETRY_GNSS_DIST_SLIGHT_DIV_THRESH = 1.0  # causing status info

# maximum angular velocity in rad/sec at standstill
NOT_MOVING_ANG_VELO_UB = 0.05
# maximum linear acceleration in m/s^2 at standstill
NOT_MOVING_LIN_ACC_UB = 3.5

# minimum factor between linear acceleration values recorded in active vs.
# passive state (navigation) -> active / passive ratio
ACTIVE_PASSIVE_RATIO_LB = 1.2

# maximum linear twist at standstill
NOT_MOVING_LINEAR_TWIST_UB = 0.5
# maximum angular twist at standstill
NOT_MOVING_ANGULAR_TWIST_UB = 0.5
# minimum linear twist during movement
MOVING_LINEAR_TWIST_LB = 0.3

# minimum distance in meters between two consecutive GNSS
# position estimates to allow yaw angle interpolation
DIST_THRESH_FOR_INTERPOLATION_BETWEEN_GNSS_POS = 0.05

# length of the list that keeps the latest linear acceleration values
LIN_ACC_HISTORY_LEN = 30

# maximum divergence between yaw angle (z component) estimates without contingency
Z_COMP_DIFF_UB = 0.3

# length of the list that keeps the latest IMU entries
IMU_ENTRIES = 1500
# fraction of the largest absolute linear acceleration values
# of the IMU entries to be considered
IMU_PERCENTAGE = .1

# IMU standard deviation thresholds
IMU_ORIENTATION_STD_DEV_UB = 10  # quaternion
IMU_ANGULAR_VELO_STD_DEV_UB = 10  # rad/sec
IMU_LIN_ACC_STD_DEV_UB = 10  # m/s^2

# odometry standard deviation thresholds
ODOM_POSE_STD_DEV_UB = 3
ODOM_TWIST_STD_DEV_UB = 3

# frequency at which localization monitoring takes place
LOCALIZATION_MON_FREQ = 0.5

LOCALIZATION_FAILURES = {
    0: "GNSS (initial-current) and odometry (initial-current) distances are diverging quite heavily "
       + "-> indicator for localization issue",
    1: "GNSS (initial-current) and odometry (initial-current) distances are diverging quite a bit "
       + "-> indicator for localization issue",
    2: "GNSS (initial-current) and odometry (initial-current) distances are slightly diverging "
       + "-> indicator for minor localization issue",
    3: "yaw diff between GNSS interpolation and IMU too high",
    4: "yaw diff between GNSS interpolation and filtered odometry too high",
    5: "IMU standard deviations too high",
    6: "IMU angular velocity too high for passive state",
    7: "linear acceleration too high for passive state",
    8: "linear acceleration during movement not considerably higher compared to standing still",
    9: "linear twist too high for passive state",
    10: "angular twist too high for passive state",
    11: "odometry pose standard deviation too high",
    12: "odometry twist standard deviation too high"
}
LOCALIZATION_CATA = "localization failure resolution not successful -- catastrophe"
#############################################################################
#############################################################################
#############################################################################

#############################################################################
############################ WEATHER MONITORING #############################
#############################################################################
FEASIBLE_MAX_TEMP = 40  # in deg. C
FEASIBLE_MIN_TEMP = -5  # in deg. C
MIN_DIST_TO_SUNSET = 15  # in minutes
WEATHER_MONITORING_FREQUENCY = 15  # in seconds
WEATHER_LOG_FREQ = 50  # every n monitoring iterations


WEATHER_FAILURES = {
    0: "moderate rain - continuing work",
    1: "heavy rain - interrupting work, seeking shelter",
    2: "moderate snow - continuing work",
    3: "heavy snow - interrupting work, seeking shelter",
    4: "strong breeze -> large branches in continuous motion etc. - continuing work",
    5: "gale -> whole trees in motion; inconvenience felt when walking against the wind; wind breaks twigs and"
       + " small branches - continuing work, but it begins getting critical",
    6: "strong gale -> risk for structural damage - interrupting work, seeking shelter",
    7: "storm force -> very high risk for structural damage; larger trees blown over and uprooted "
       + "- interrupting work, seeking shelter",
    8: "hurricane -> very high risk for severe and extensive structural damage - interrupting work, seeking shelter",
    9: "very high temperature; sensor damage possible - interrupting work, seeking shelter",
    10: "very low temperature; battery and sensor damage expected - interrupting work, seeking shelter",
    11: "thunderstorm - interrupting work, seeking shelter",
    12: "tornado - interrupting work, seeking shelter",
    13: "perception may be impaired by mist, smoke or fog - interrupting work, continuing later",
    14: "before sunrise - interrupting work until then",
    15: "after sunset - interrupting work until sunrise",
    16: "sunset in a few minutes - interrupting work and driving back to base"
}
WEATHER_CATA = "drastic weather change resolution failed -- catastrophe"

# WEATHER CONDITION CODES
# Group 2xx: Thunderstorm
THUNDERSTORM_WITH_LIGHT_RAIN = 200
THUNDERSTORM_WITH_RAIN = 201
THUNDERSTORM_WITH_HEAVY_RAIN = 202
LIGHT_THUNDERSTORM = 210
THUNDERSTORM = 211
HEAVY_THUNDERSTORM = 212
RAGGED_THUNDERSTORM = 221
THUNDERSTORM_WITH_LIGHT_DRIZZLE = 230
THUNDERSTORM_WITH_DRIZZLE = 231
THUNDERSTORM_WITH_HEAVY_DRIZZLE = 232
# Group 3xx: Drizzle
LIGHT_INTENSITY_DRIZZLE = 300
DRIZZLE = 301
HEAVY_INTENSITY_DRIZZLE = 302
LIGHT_INTENSITY_DRIZZLE_RAIN = 310
DRIZZLE_RAIN = 311
HEAVY_INTENSITY_DRIZZLE_RAIN = 312
SHOWER_RAIN_AND_DRIZZLE = 313
HEAVY_SHOWER_RAIN_AND_DRIZZLE = 314
SHOWER_DRIZZLE = 321
# Group 5xx: Rain
LIGHT_RAIN = 500
MODERATE_RAIN = 501
HEAVY_INTENSITY_RAIN = 502
VERY_HEAVY_RAIN = 503
EXTREME_RAIN = 504
FREEZING_RAIN = 511
LIGHT_INTENSITY_SHOWER_RAIN = 520
SHOWER_RAIN = 521
HEAVY_INTENSITY_SHOWER_RAIN = 522
RAGGED_SHOWER_RAIN = 531
# Group 6xx: Snow
LIGHT_SNOW = 600
SNOW = 601
HEAVY_SNOW = 602
SLEET = 611
LIGHT_SHOWER_SLEET = 612
SHOWER_SLEET = 613
LIGHT_RAIN_AND_SNOW = 615
RAIN_AND_SNOW = 616
LIGHT_SHOWER_SNOW = 620
SHOWER_SNOW = 621
HEAVY_SHOWER_SNOW = 622
# Group 7xx: Atmosphere
MIST = 701
SMOKE = 711
HAZE = 721
SAND_DUST_WHIRLS = 731
FOG = 741
SAND = 751
DUST = 761
VOLCANIC_ASH = 762
SQUALLS = 771
TORNADO = 781
# Group 800: Clear
CLEAR_SKY = 800
# Group 80x: Clouds
FEW_CLOUDS = 801
SCATTERED_CLOUDS = 802
BROKEN_CLOUDS = 803
OVERCAST_CLOUDS = 804

RAIN_SIM_VAL = 8  # mm/h
SNOW_SIM_VAL = 4  # mm/h
WIND_SIM_VAL = 27  # m/s
TEMP_MIN_SIM = -9  # deg. C
TEMP_MAX_SIM = -2  # deg. C
TEMP_SIM = -5  # deg. C
SIM_WEATHER_CODE = 221
#############################################################################
#############################################################################
#############################################################################

#############################################################################
########################### CONNECTION MONITORING ###########################
#############################################################################
# identifier of the WiFi interface (e.g. available via `ifconfig`)
WIFI_INTERFACE = "wlx3c1e045678a2"

# frequency at which connection timeout monitoring takes place (in seconds)
TIMEOUT_MON_FREQ = 5
# frequency at which the internet connection is monitored (in seconds)
INTERNET_MON_FREQ = 10
# number of reconnections before calling a human operator for help
REPEAT_CONNECTION_CHECK_THRESH = 1

WIFI_FAILURES = {
    0: "bad wifi link",
    1: "bad wifi signal",
    2: "bad wifi bitrate",
    3: "wifi disconnect",
    4: "wifi timeout"
}
INTERNET_FAILURES = {
    0: "internet disconnect",
    1: "internet connection: bad download speed",
    2: "internet connection: bad upload speed",
    3: "internet timeout"
}
GNSS_FAILURES = {
    0: "GNSS timeout",
    1: "unknown GNSS status",
    2: "GNSS unable to find position - no fix",
    3: "using pure GNSS, no RTK available",
    4: "unknown GNSS service",
    5: "GNSS - latitude not present or infeasible",
    6: "GNSS - longitude not present or infeasible",
    7: "GNSS - unknown covariance type",
    8: "GNSS - critically high standard deviations",
    9: "GNSS - critically high approximated standard deviations",
    10: "GNSS - standard deviation progression issue (increasingly higher)"
}
CONNECTION_CATA = "connection resolution failed -- catastrophe"

# thresholds
CRITICALLY_BAD_WIFI_LINK_THRESH = 5  # in percent
BAD_WIFI_LINK_THRESH = 25  # in percent
BELOW_AVG_WIFI_LINK = 50  # in percent

CRITICALLY_BAD_WIFI_SIGNAL_THRESH = -90  # in dBm
VERY_LOW_WIFI_SIGNAL_THRESH = -80  # in dBm
LOW_WIFI_SIGNAL_THRESH = -75  # in dBm

CRITICALLY_BAD_WIFI_BIT_RATE_THRESH = 1  # in Mb/s
RATHER_LOW_WIFI_BIT_RATE_THRESH = 20  # in Mb/s

CRITICALLY_BAD_DOWNLOAD_SPEED_THRESH = 1  # in Mb/s
RATHER_LOW_DOWNLOAD_SPEED_THRESH = 10  # in Mb/s

CRITICALLY_BAD_UPLOAD_SPEED_THRESH = 1  # in Mb/s
RATHER_LOW_UPLOAD_SPEED = 10  # in Mb/s

# limits
LAT_LB = -90  # latitude lower bound
LAT_UB = 90  # latitude upper bound
LNG_LB = -180  # longitude lower bound
LNG_UB = 180  # longitude upper bound
STD_DEVIATION_UB = 10  # standard deviations upper bound (in meters)
COVARIANCE_HISTORY_LENGTH = 5  # number of tracked covariance matrices
SIGNIFICANT_DEVIATION_INCREASE = 5  # what is considered a significant increase in meters

# problematic values used in the simulation of error events
BAD_WIFI_LINK_QUALITY = 2  # in percent
BAD_WIFI_SIGNAL_LEVEL = -90  # in dBm
BAD_WIFI_BIT_RATE = 0.1  # in Mb/s
WIFI_DISCONNECT = 0  # in Mb/s
BAD_DOWNLOAD = 0.5  # in Mb/s
BAD_UPLOAD = 0.5  # in Mb/s

# timeouts
GPS_TIMEOUT = 30  # in s
WIFI_TIMEOUT = 120  # in s
INTERNET_TIMEOUT = 120  # in s

# monitoring frequencies
WIFI_MONITORING_FREQ = 10  # in s
WIFI_QUALITY_ESTIMATION_FREQ = 60  # in s

# GNSS STATUS OPTIONS
GNSS_STATUS_NO_FIX = -1  # unable to find position
GNSS_STATUS_FIX = 0  # unaugmented fix - found a location, using solely GPS/GLONASS/etc.
GNSS_STATUS_SBAS_FIX = 1  # with satellite-based augmentation - fix with assistance of such networks as StarFire
GNSS_STATUS_GBAS_FIX = 2  # with ground-based augmentation - fix with assistance of such networks as DGPS or GBAS
# GNSS SERVICE OPTIONS
GNSS_SERVICE_INVALID = 0
GNSS_SERVICE_GPS = 1
GNSS_SERVICE_GLONASS = 2
GNSS_SERVICE_COMPASS = 4
GNSS_SERVICE_GALILEO = 8
# GNSS COVARIANCE TYPES
GNSS_COVARIANCE_TYPE_UNKNOWN = 0
GNSS_COVARIANCE_TYPE_APPROXIMATED = 1
GNSS_COVARIANCE_TYPE_DIAGONAL_KNOWN = 2
GNSS_COVARIANCE_TYPE_KNOWN = 3
#############################################################################
#############################################################################
#############################################################################
