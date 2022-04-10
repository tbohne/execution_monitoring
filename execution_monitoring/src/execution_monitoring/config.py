#!/usr/bin/env python

#############################################################################
################################### SETUP ###################################
#############################################################################

VERBOSE_LOGGING = False

BASE_POSE = [52.3203191407, 8.153625154949, 270]
DOCKING_BASE_POSE = [52.3203930281, 8.15361381961, 180]


MBF_FAILURE = 50
MBF_PAT_EXCEEDED = 103

DOCKING = True

MISSION_IDLE_LIMIT = 900

EXPERIMENTS_CHECK_FREQ = 120

ERROR_SLEEP_TIME = 5
PREEMPTION_SLEEP_TIME = 10

WAIT_SLEEP_TIME = 5

PLAN_CHECK_FREQ = 10

EXP_PATH = "/home/docker/catkin_ws/src/execution_monitoring/execution_monitoring/experiments/"

MISSION_FAIL_MSG =  "MISSION FAIL: exceeded idle time limit during plan execution"
#############################################################################
#############################################################################
#############################################################################

#############################################################################
############################## OBSTACLE SPAWNER #############################
#############################################################################
BARRIER_HEIGHT = 0.833558
DIST_TO_ROBOT = 2.0

#############################################################################
#############################################################################
#############################################################################

#############################################################################
####################### POWER MANAGEMENT MONITORING #########################
#############################################################################
CONTINGENCY_MSG = "CONT"
CATASTROPHE_MSG = "CATO"

INITIAL_SLEEP_TIME = 30

CHARGE_SLEEP_TIME = 2
ALREADY_CHARGED_THRESH = 95

NORMAL_DISCHARGE_RATE = 0.03
CONTINGENCY_DISCHARGE_RATE = 0.35
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
RECOVERY_LIMIT = 30
GOAL_STATUS_TOPIC = "/move_base_flex/exe_path/status"

RECOVERY_PROGRESS_MIN_DIST_THRESH = 0.001

NAV_MON_FREQ = 5

GOAL_STATUS_ACTIVE = 1
GOAL_STATUS_PREEMPTED = 2
GOAL_STATUS_SUCCEEDED = 3
GOAL_STATUS_ABORTED = 4

NAVIGATION_FAILURES = {
    0: "sustained recovery - mbf cannot recover",
    1: "sustained recover - but still making progress - continuing for now",
    2: "explicit nav failure reported by low-level operation state machine"
}

NAV_CATA = "nav resolution failed -- catastrophe"

RECOVERY_POINT_ONE = [52.32056824755535, 8.153337579568582, 270]

STREET = [52.320786493558508, 8.153624127240558, 270]
FIELD = [52.32042638488258,  8.153084460244127, 270]
#############################################################################
#############################################################################
#############################################################################

#############################################################################
###################### CHARGING FAILURE MONITORING ##########################
#############################################################################
DOCKING_BASE_POSE_FAIL = [52.32059819361085, 8.153113603063638, 90]

CHARGING_FAILURE_TIME = 10

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
MON_FREQ = 10
IDLE_THRESH = 300
OPERATION_TOPIC = "arox/ongoing_operation"

PLAN_DEPLOYMENT_FAILURES = {
    0: "robot idle for an extended period of time",
    1: "plan retrieval service unavailable",
    2: "deployed plan empty",
    3: "deployed plan corrupted / infeasible",
    4: "unspecified plan deployment failure"
}

PLAN_DEPLOYMENT_CATA = "plan deployment resolution failed -- catastrophe"

FEASIBLE_ACTIONS = ["drive_to", "return_to_base", "charge", "scan"]

PLAN_RETRIEVAL_TIMEOUT_CODE = 0
EMPTY_PLAN_CODE = 1
INFEASIBLE_PLAN_CODE = 2
#############################################################################
#############################################################################
#############################################################################

#############################################################################
######################## DATA MANAGEMENT MONITORING #########################
#############################################################################

DATA_MANAGEMENT_FAILURES = {
    0: "full memory",
    1: "scan not logged correctly"
}

DATA_MANAGEMENT_CATA = "data management resolution failed -- catastrophe"

# DATA MANAGEMENT SETTINGS
FULL_MEMORY_THRESH = 99.0
ALMOST_FULL_MEMORY_THRESH_TWO = 95.0
ALMOST_FULL_MEMORY_THRESH_ONE = 90.0

MONITOR_DRIVE = "/"
FULL_DRIVE = "/mnt/usb"
ENABLE_SPECIFIC_LASER_SCAN_CHECK = True
#############################################################################
#############################################################################
#############################################################################

#############################################################################
############################# SENSOR MONITORING #############################
#############################################################################

SENSOR_FAILURES = {
    0: "total sensor failure",
    1: "empty list of range values",
    2: "predominantly infeasible range values (inf)",
    3: "repeated scan"
}
SENSOR_CATA = "sensor resolution failed -- catastrophe"

# SCAN SETTINGS
SCAN_TIME = 4
SCAN_TOPIC = "/RIEGL"
SCAN_PATH = MONITOR_DRIVE + "home/docker/catkin_ws/src/execution_monitoring/execution_monitoring/scans/"
SCAN_TIME_LIMIT = 30
SCAN_FILE_EXTENSION = ".txt"
SCAN_VALUES_LB_PERCENTAGE = 5
#############################################################################
#############################################################################
#############################################################################

#############################################################################
########################## LOCALIZATION MONITORING ##########################
#############################################################################

STATUS_SWITCH_DELAY = 4

ODOMETRY_GNSS_DIST_HEAVY_DIV_THRESH = 2.0
ODOMETRY_GNSS_DIST_DIV_CONTINGENCY_THRESH = 1.5
ODOMETRY_GNSS_DIST_SLIGHT_DIV_THRESH = 1.0

# TODO: should be checked - arbitrarily selected
NOT_MOVING_ANG_VELO_UB = 0.05
# TODO: not sure -- check later
NOT_MOVING_LIN_ACC_UB = 3.5

ACTIVE_PASSIVE_FACTOR_LB = 1.2

NOT_MOVING_LINEAR_TWIST_UB = 0.5
NOT_MOVING_ANGULAR_TWIST_UB = 0.5
MOVING_LINEAR_TWIST_LB = 0.3

DIST_THRESH_FOR_INTERPOLATION_BETWEEN_GNSS_POS = 0.05

LIN_ACC_HISTORY_LEN = 30

Z_COMP_DIFF_UB = 0.3

IMU_ENTRIES = 1500
IMU_PERCENTAGE = .1

IMU_ORIENTATION_STD_DEV_UB = 10  # quaternion
IMU_ANGULAR_VELO_STD_DEV_UB = 10  # rad/sec
IMU_LIN_ACC_STD_DEV_UB = 10  # m/s^2

ODOM_POSE_STD_DEV_UB = 3
ODOM_TWIST_STD_DEV_UB = 3

LOCALIZATION_MON_FREQ = 0.5

# LOCALIZATION FAILURES / INFOS

LOCALIZATION_FAILURES = {
    0: "GNSS (initial-current) and odometry (initial-current) distances are diverging quite heavily -> indicator for localization issue",
    1: "GNSS (initial-current) and odometry (initial-current) distances are diverging quite a bit -> indicator for localization issue",
    2: "GNSS (initial-current) and odometry (initial-current) distances are slightly diverging -> indicator for minor localization issue",
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
# WEATHER FAILURES / INFOS

WEATHER_FAILURES = {
    0: "moderate rain - continuing work",
    1: "heavy rain - interrupting work, seeking shelter",
    2: "moderate snow - continuing work",
    3: "heavy snow - interrupting work, seeking shelter",
    4: "strong breeze -> large branches in continuous motion etc. - continuing work",
    5: "gale -> whole trees in motion; inconvenience felt when walking against the wind; wind breaks twigs and small branches - continuing work, but it begins getting critical",
    6: "strong gale -> risk for structural damage - interrupting work, seeking shelter",
    7: "storm force -> very high risk for structural damage; larger trees blown over and uprooted - interrupting work, seeking shelter",
    8: "hurricane -> very high risk for severe and extensive structural damage - interrupting work, seeking shelter",
    9: "very high temperature (> 40 deg. C); sensor damage possible - interrupting work, seeking shelter",
    10: "very low temperature (< -5 deg. C); battery and sensor damage expected - interrupting work, seeking shelter",
    11: "thunderstorm - interrupting work, seeking shelter",
    12: "tornado - interrupting work, seeking shelter",
    13: "perception may be impaired by mist, smoke or fog - interrupting work, continuing later",
    14: "before sunrise - interrupting work until then",
    15: "after sunset - interrupting work until sunrise",
    16: "sunset in a few minutes - interrupting work and driving back to base"
}

WEATHER_CATA = "drastic weather change resolution failed -- catastrophe"

WEATHER_MONITORING_FREQUENCY = 15  # seconds

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
#############################################################################
#############################################################################
#############################################################################

#############################################################################
########################### CONNECTION MONITORING ###########################
#############################################################################
TIMEOUT_MON_FREQ = 5
INTERNET_MON_FREQ = 10

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
BAD_WIFI_LINK_QUALITY = 2
CRITICALLY_BAD_WIFI_LINK_THRESH = 5
BAD_WIFI_LINK_THRESH = 25
BELOW_AVG_WIFI_LINK = 50

CRITICALLY_BAD_WIFI_SIGNAL_THRESH = -90
VERY_LOW_WIFI_SIGNAL_THRESH = -80
LOW_WIFI_SIGNAL_THRESH = -75

CRITICALLY_BAD_WIFI_BIT_RATE_THRESH = 1
RATHER_LOW_WIFI_BIT_RATE_THRESH = 20

CRITICALLY_BAD_DOWNLOAD_SPEED_THRESH = 1
RATHER_LOW_DOWNLOAD_SPEED_THRESH = 10

CRITICALLY_BAD_UPLOAD_SPEED_THRESH = 1
RATHER_LOW_UPLOAD_SPEED = 10

BAD_WIFI_SIGNAL_LEVEL = -90
BAD_WIFI_BIT_RATE = 0.1
WIFI_DISCONNECT = 0
BAD_DOWNLOAD = 0.5
BAD_UPLOAD = 0.5
GPS_TIMEOUT = 30
WIFI_TIMEOUT = 120
WIFI_MONITORING_FREQ = 10
WIFI_QUALITY_ESTIMATION_FREQ = 60
INTERNET_TIMEOUT = 120

# GNSS STATUS OPTIONS
GNSS_STATUS_NO_FIX = -1  # unable to find position
GNSS_STATUS_FIX = 0      # unaugmented fix - found a location, using solely GPS/GLONASS/etc.
GNSS_STATUS_SBAS_FIX = 1 # with satellite-based augmentation - fix with assistance of such networks as StarFire
GNSS_STATUS_GBAS_FIX = 2 # with ground-based augmentation - fix with assistance of such networks as DGPS or GBAS
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
# other GNSS conf
LAT_LB = -90
LAT_UB = 90
LNG_LB = -180
LNG_UB = 180
# GNSS covariance config
STD_DEVIATION_UB = 10
COVARIANCE_HISTORY_LENGTH = 5
SIGNIFICANT_DEVIATION_INCREASE = 5

# WIFI MONITORING
WIFI_INTERFACE = "wlx3c1e045678a2"
#############################################################################
#############################################################################
#############################################################################
