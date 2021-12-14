#!/usr/bin/env python

#############################################################################
################################# MOVEMENT ##################################
#############################################################################
BASE_POSE = [52.3203191407, 8.153625154949, 270]
MBF_FAILURE = 50
MBF_PAT_EXCEEDED = 103
#############################################################################
#############################################################################
#############################################################################

#############################################################################
######################## DATA MANAGEMENT MONITORING #########################
#############################################################################
DATA_MANAGEMENT_FAILURE_ONE = "full memory"
DATA_MANAGEMENT_FAILURE_TWO = "scan not logged correctly"

# DATA MANAGEMENT SETTINGS
MONITOR_DRIVE = "/"
ENABLE_SPECIFIC_LASER_SCAN_CHECK = True
#############################################################################
#############################################################################
#############################################################################

#############################################################################
############################# SENSOR MONITORING #############################
#############################################################################
SENSOR_FAILURE_ONE = "total sensor failure"
SENSOR_FAILURE_TWO = "empty list of range values"
SENSOR_FAILURE_THREE = "predominantly infeasible range values (inf)"
SENSOR_FAILURE_FOUR = "repeated scan"

# SCAN SETTINGS
SCAN_PATH = MONITOR_DRIVE + "/home/docker/catkin_ws/src/execution_monitoring/execution_monitoring/scans/"
SCAN_TIME_LIMIT = 60
SCAN_FILE_EXTENSION = ".txt"
SCAN_VALUES_LB_PERCENTAGE = 5
#############################################################################
#############################################################################
#############################################################################

#############################################################################
########################## LOCALIZATION MONITORING ##########################
#############################################################################
# TODO: should be checked - arbitrarily selected
NOT_MOVING_ANG_VELO_UB = 0.05
NOT_MOVING_LIN_ACC_UB = 0.3

NOT_MOVING_LINEAR_TWIST_UB = 0.01
NOT_MOVING_ANGULAR_TWIST_UB = 0.01
MOVING_LINEAR_TWIST_LB = 0.3

DIST_THRESH_FOR_INTERPOLATION_BETWEEN_GNSS_POS = 0.01

Z_COMP_DIFF_UB = 0.03

IMU_ENTRIES = 1500
IMU_PERCENTAGE = .1

IMU_ORIENTATION_COV_UB = 10
IMU_ANGULAR_VELO_COV_UB = 10
IMU_LINEAR_ACC_COV_UB = 10
ODOM_POSE_COV_UP = 20
ODOM_TWIST_COV_UP = 20
#############################################################################
#############################################################################
#############################################################################

#############################################################################
############################ WEATHER MONITORING #############################
#############################################################################
# WEATHER FAILURES / INFOS
WEATHER_FAILURE_ONE = "moderate rain - continuing work"
WEATHER_FAILURE_TWO = "heavy rain - interrupting work, seeking shelter"
WEATHER_FAILURE_FOUR = "moderate snow - continuing work"
WEATHER_FAILURE_FIVE = "heavy snow - interrupting work, seeking shelter"
WEATHER_FAILURE_SIX = "strong breeze -> large branches in continuous motion etc. - continuing work"
WEATHER_FAILURE_SEVEN = "gale -> whole trees in motion; inconvenience felt when walking against the wind; wind breaks twigs and small branches - continuing work, but it begins getting critical"
WEATHER_FAILURE_EIGHT = "strong gale -> risk for structural damage - interrupting work, seeking shelter"
WEATHER_FAILURE_NINE = "storm force -> very high risk for structural damage; larger trees blown over and uprooted - interrupting work, seeking shelter"
WEATHER_FAILURE_TEN = "hurricane -> very high risk for severe and extensive structural damage - interrupting work, seeking shelter"
WEATHER_FAILURE_ELEVEN = "very high temperature (> 40 deg. C); sensor damage possible - interrupting work, seeking shelter"
WEATHER_FAILURE_TWELVE = "very low temperature (< -5 deg. C); battery and sensor damage expected - interrupting work, seeking shelter"
WEATHER_FAILURE_THIRTEEN = "thunderstorm - interrupting work, seeking shelter"
WEATHER_FAILURE_FOURTEEN = "tornado - interrupting work, seeking shelter"
WEATHER_FAILURE_FIFTEEN = "perception may be impaired by mist, smoke or fog - interrupting work, continuing later"
WEATHER_FAILURE_SIXTEEN = "before sunrise - interrupting work until then"
WEATHER_FAILURE_SEVENTEEN = "after sunset - interrupting work until sunrise"
WEATHER_FAILURE_EIGHTEEN = "sunset in a few minutes - interrupting work and driving back to base"

WEATHER_MONITORING_FREQUENCY = 5

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
CONNECTION_FAILURE_ONE = "bad wifi link"
CONNECTION_FAILURE_TWO = "bad wifi signal"
CONNECTION_FAILURE_THREE = "bad wifi bitrate"
CONNECTION_FAILURE_FOUR = "wifi disconnect"
CONNECTION_FAILURE_FIVE = "internet disconnect"
CONNECTION_FAILURE_SIX = "internet connection: bad download speed"
CONNECTION_FAILURE_SEVEN = "internet connection: bad upload speed"
CONNECTION_FAILURE_EIGHT = "GNSS timeout"
CONNECTION_FAILURE_NINE = "wifi timeout"
CONNECTION_FAILURE_TEN = "internet timeout"
CONNECTION_FAILURE_ELEVEN = "unknown GNSS status"
CONNECTION_FAILURE_TWELVE = "GNSS unable to find position - no fix"
CONNECTION_FAILURE_THIRTEEN = "using pure GNSS, no RTK available"
CONNECTION_FAILURE_FOURTEEN = "unknown GNSS service"
CONNECTION_FAILURE_FIFTEEN = "GNSS - latitude not present or infeasible"
CONNECTION_FAILURE_SIXTEEN = "GNSS - longitude not present or infeasible"
CONNECTION_FAILURE_SEVENTEEN = "GNSS - unknown covariance type"
CONNECTION_FAILURE_EIGHTEEN = "GNSS - critically high covariance"
CONNECTION_FAILURE_NINETEEN = "GNSS - critically high approximated covariance"
CONNECTION_FAILURE_TWENTY = "GNSS - covariance progression issue (increasingly higher)"
BAD_WIFI_LINK_QUALITY = 2
BAD_WIFI_SIGNAL_LEVEL = -90
BAD_WIFI_BIT_RATE = 0.1
WIFI_DISCONNECT = 0
BAD_DOWNLOAD = 0.5
BAD_UPLOAD = 0.5
GPS_TIMEOUT = 60
WIFI_TIMEOUT = 120
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
HIGH_AREA_COVARIANCE = 50
GOOD_VARIANCE_UB = 20
COVARIANCE_HISTORY_LENGTH = 5
SIGNIFICANT_COVARIANCE_INCREASE = 15

# WIFI MONITORING
WIFI_INTERFACE = "wlx3c1e045678a2"
#############################################################################
#############################################################################
#############################################################################
