#!/usr/bin/env python
import rospy
import collections
from datetime import datetime
from std_msgs.msg import String, Bool
from execution_monitoring.msg import WiFi, Internet
from sensor_msgs.msg import NavSatFix
from execution_monitoring import config
import math


class ConnectionMonitoring:
    """
    Provides monitoring solutions for connection failures (WiFi, Internet, GNSS).
    """

    def __init__(self):
        self.active_monitoring = True
        self.last_gnss_msg_time = datetime.now()
        self.last_wifi_msg_time = datetime.now()
        self.last_internet_msg_time = datetime.now()
        self.gnss_covariance_history = collections.deque([], config.COVARIANCE_HISTORY_LENGTH)
        self.gnss_info_time = datetime.now()

        self.contingency_pub = rospy.Publisher('/contingency_preemption', String, queue_size=1)
        self.aggravate_pub = rospy.Publisher('/aggravate', String, queue_size=1)
        self.robot_info_pub = rospy.Publisher('/robot_info', String, queue_size=1)
        self.interrupt_reason_pub = rospy.Publisher('/interrupt_reason', String, queue_size=1)

        rospy.Subscriber('/wifi_connectivity_info', WiFi, self.wifi_callback, queue_size=1)
        rospy.Subscriber('/internet_connectivity_info', Internet, self.internet_callback, queue_size=1)
        rospy.Subscriber('/fix', NavSatFix, self.gnss_callback, queue_size=1)
        rospy.Subscriber('/resolve_connection_failure_success', Bool, self.resolve_callback, queue_size=1)

        self.timeout_monitoring()

    def timeout_monitoring(self):
        """
        Continuously running procedure that checks the time since the last message from either connection. If a
        user-specified time limit for one of the connections is exceeded, a contingency due to a disconnection is
        initiated.
        """

        while not rospy.is_shutdown():
            now = datetime.now()

            if self.active_monitoring:
                if self.last_gnss_msg_time:
                    time_since_update = (now - self.last_gnss_msg_time).total_seconds()
                    if time_since_update > config.GPS_TIMEOUT:
                        rospy.loginfo("detected GNSS timeout - no new update since %s s", time_since_update)
                        self.contingency_pub.publish(config.CONNECTION_FAILURE_EIGHT)
                        self.active_monitoring = False
                        self.last_gnss_msg_time = datetime.now()

                if self.last_wifi_msg_time:
                    time_since_update = (now - self.last_wifi_msg_time).total_seconds()
                    if time_since_update > config.WIFI_TIMEOUT:
                        rospy.loginfo("detected wifi timeout - no new update since %s s", time_since_update)
                        self.contingency_pub.publish(config.CONNECTION_FAILURE_NINE)
                        self.active_monitoring = False

                if self.last_internet_msg_time:
                    time_since_update = (now - self.last_internet_msg_time).total_seconds()
                    if time_since_update > config.INTERNET_TIMEOUT:
                        rospy.loginfo("detected internet timeout - no new update since %s s", time_since_update)
                        self.contingency_pub.publish(config.CONNECTION_FAILURE_TEN)
                        self.active_monitoring = False

            rospy.sleep(config.TIMEOUT_MON_FREQ)

    @staticmethod
    def feasible_standard_deviations(nav_sat_fix):
        """
        Checks each GNSS standard deviation (square root of the diagonal values of the covariance matrix) against a
        user-configurable upper bound.

        @param nav_sat_fix: `NavSatFix` message that contains the standard deviations to be checked
        @return: whether or not the standard deviations satisfy the upper bound
        """
        for i in range(0, len(nav_sat_fix.position_covariance), 4):
            if nav_sat_fix.position_covariance[i] != float('nan') and math.sqrt(
                    nav_sat_fix.position_covariance[i]) > config.STD_DEVIATION_UB:
                return False
        return True

    def estimate_gnss_quality(self, nav_sat_fix):
        """
        Qualitative assessment of GNSS links that work in principle based on the information in the `NavSatFix` message.
        If the connection is not perfect, the mission is not immediately interrupted, but it is at least documented.
        ----------------------------------------------------------------------------------------------------------------
        Quality ratings:
        - good (status GBAS_FIX, cov type >= DIAGONAL_KNOWN, each standard deviation <= STD_DEVIATION_UB)
        - medium (status >= FIX, cov type >= APPROXIMATED, each standard deviation <= STD_DEVIATION_UB)
        - low (status >= FIX, cov type == UNKNOWN)
        ----------------------------------------------------------------------------------------------------------------

        @param nav_sat_fix: `NavSatFix` message that provides the information for the quality assessment
        """
        estimation_str = "Quality estimation of currently provided GNSS data: "

        if (datetime.now() - self.gnss_info_time).total_seconds() > config.WIFI_QUALITY_ESTIMATION_FREQ:
            self.gnss_info_time = datetime.now()
            # GOOD
            if nav_sat_fix.status.status == config.GNSS_STATUS_GBAS_FIX and nav_sat_fix.position_covariance_type \
                    >= config.GNSS_COVARIANCE_TYPE_DIAGONAL_KNOWN and self.feasible_standard_deviations(nav_sat_fix):
                if config.VERBOSE_LOGGING:
                    self.robot_info_pub.publish(estimation_str + "good")
            # MEDIUM
            elif nav_sat_fix.status.status >= config.GNSS_STATUS_FIX and nav_sat_fix.position_covariance_type \
                    >= config.GNSS_COVARIANCE_TYPE_APPROXIMATED and self.feasible_standard_deviations(nav_sat_fix):
                self.robot_info_pub.publish(estimation_str + "medium")
            # LOW
            elif nav_sat_fix.status.status >= config.GNSS_STATUS_FIX and nav_sat_fix.position_covariance_type \
                    == config.GNSS_COVARIANCE_TYPE_UNKNOWN:
                self.robot_info_pub.publish(estimation_str + "low")

    def status_monitoring(self, nav_sat_fix):
        """
        Monitoring for the GNSS status.

        @param nav_sat_fix: `NavSatFix` message to obtain status information from
        """
        if nav_sat_fix.status.status not in [config.GNSS_STATUS_NO_FIX, config.GNSS_STATUS_FIX,
                                             config.GNSS_STATUS_SBAS_FIX, config.GNSS_STATUS_GBAS_FIX]:
            self.contingency_pub.publish(config.CONNECTION_FAILURE_ELEVEN)
        elif nav_sat_fix.status.status == config.GNSS_STATUS_NO_FIX:
            self.contingency_pub.publish(config.CONNECTION_FAILURE_TWELVE)
        elif nav_sat_fix.status.status == config.GNSS_STATUS_FIX:
            self.robot_info_pub.publish(config.CONNECTION_FAILURE_THIRTEEN)

    def service_monitoring(self, nav_sat_fix):
        """
        Monitoring for the GNSS service.

        @param nav_sat_fix: `NavSatFix` message to obtain service information from
        """
        if nav_sat_fix.status.service not in [config.GNSS_SERVICE_GPS, config.GNSS_SERVICE_GLONASS,
                                              config.GNSS_SERVICE_COMPASS, config.GNSS_SERVICE_GALILEO]:
            self.robot_info_pub.publish(config.CONNECTION_FAILURE_FOURTEEN)

    def belief_state_monitoring(self, nav_sat_fix):
        """
        Monitoring for the current latitude / longitude belief state.

        @param nav_sat_fix: `NavSatFix` message to obtain lat / lng information from
        """
        if nav_sat_fix.latitude < config.LAT_LB or nav_sat_fix.latitude > config.LAT_UB:
            # lat (degrees): pos -> north of equator, neg -> south of equator
            self.contingency_pub.publish(config.CONNECTION_FAILURE_FIFTEEN)
        elif nav_sat_fix.longitude < config.LNG_LB or nav_sat_fix.longitude > config.LNG_UB:
            # lng (degrees): pos -> east of prime meridian, neg -> west of prime meridian
            self.contingency_pub.publish(config.CONNECTION_FAILURE_SIXTEEN)

    def covariance_monitoring(self, nav_sat_fix):
        """
        Monitoring for the GNSS covariance values.

        -- in m^2 defined relative to a tangential plane through the currently believed position
        -- components are "east", "north" and "up" (ENU coordinate system)

        -------------------------------------------
        rated aspects:
         - absolute values (good / bad)
         - progression over time
        -------------------------------------------

        @param nav_sat_fix: `NavSatFix` message to obtain covariance information from
        """

        if nav_sat_fix.position_covariance_type not in [config.GNSS_COVARIANCE_TYPE_UNKNOWN,
                                                        config.GNSS_COVARIANCE_TYPE_APPROXIMATED,
                                                        config.GNSS_COVARIANCE_TYPE_DIAGONAL_KNOWN,
                                                        config.GNSS_COVARIANCE_TYPE_KNOWN]:
            self.robot_info_pub.publish(config.CONNECTION_FAILURE_SEVENTEEN)

        # only monitoring the progression for the diagonal, as these are the most important values
        # --> appending to history when >= DIAGONAL_KNOWN

        if nav_sat_fix.position_covariance_type >= config.GNSS_COVARIANCE_TYPE_DIAGONAL_KNOWN:
            self.gnss_covariance_history.appendleft(nav_sat_fix.position_covariance)

        if self.feasible_covariance_history():
            if nav_sat_fix.position_covariance_type >= config.GNSS_COVARIANCE_TYPE_DIAGONAL_KNOWN:
                # GNSS receiver provides at least the variance of each measurement
                # -> diagonal can be used for quality assessment
                if not self.feasible_standard_deviations(nav_sat_fix):
                    self.contingency_pub.publish(config.CONNECTION_FAILURE_EIGHTEEN)
            elif nav_sat_fix.position_covariance_type == config.GNSS_COVARIANCE_TYPE_APPROXIMATED:
                # can be considered, but with caution, without putting too much weight on it -> only approximated
                if not self.feasible_standard_deviations(nav_sat_fix):
                    self.robot_info_pub.publish(config.CONNECTION_FAILURE_NINETEEN)

    def gnss_callback(self, nav_sat_fix):
        """
        Called when new GNSS information arrives - triggers monitoring.

        @param nav_sat_fix: `NavSatFix` message containing GNSS information to be monitored
        """
        self.last_gnss_msg_time = datetime.now()
        self.estimate_gnss_quality(nav_sat_fix)
        self.status_monitoring(nav_sat_fix)
        self.service_monitoring(nav_sat_fix)
        self.belief_state_monitoring(nav_sat_fix)
        self.covariance_monitoring(nav_sat_fix)

    @staticmethod
    def increasing_values_only(components):
        """
        Checks whether the specified components are increasing only.

        @param components: components to check
        @return: whether the values are increasing only
        """
        for i in range(1, len(components)):
            # both values feasible measures
            if components[i] != float('nan') and components[i - 1] != float('nan'):
                if components[i] <= components[i - 1]:
                    return False
        return True

    @staticmethod
    def get_oldest(components):
        """
        Returns the oldest feasible value from the specified components.

        @param components: components to retrieve oldest value from
        @return: oldest value
        """
        for i in components:
            if i != float('nan'):
                return i

    @staticmethod
    def get_latest(components):
        """
        Returns the latest feasible value from the specified components.

        @param components: components to retrieve latest value from
        @return: latest value
        """
        for i in components[::-1]:
            if i != float('nan'):
                return i

    def feasible_covariance_history(self):
        """
        Analyzes the GNSS covariance history in terms of feasibility.

        @return: whether the current covariance history is feasible based on user-configurable standards
        """
        if len(self.gnss_covariance_history) == config.COVARIANCE_HISTORY_LENGTH:
            east_deviations = [math.sqrt(cov[0]) for cov in self.gnss_covariance_history][::-1]
            north_deviations = [math.sqrt(cov[4]) for cov in self.gnss_covariance_history][::-1]
            up_deviations = [math.sqrt(cov[8]) for cov in self.gnss_covariance_history][::-1]

            # definition for getting worse:
            #  - only increases
            #  - total increase between oldest and latest larger than SIGNIFICANT_DEVIATION_INCREASE (configurable)

            east_fail = (self.increasing_values_only(east_deviations) and self.get_latest(
                east_deviations) - self.get_oldest(east_deviations) > config.SIGNIFICANT_DEVIATION_INCREASE)

            north_fail = (self.increasing_values_only(north_deviations) and self.get_latest(
                north_deviations) - self.get_oldest(north_deviations) > config.SIGNIFICANT_DEVIATION_INCREASE)

            up_fail = (self.increasing_values_only(up_deviations) and self.get_latest(up_deviations) - self.get_oldest(
                up_deviations) > config.SIGNIFICANT_DEVIATION_INCREASE)

            if east_fail or north_fail or up_fail:
                self.contingency_pub.publish(config.CONNECTION_FAILURE_TWENTY)
                return False
        return True

    def resolve_callback(self, msg):
        """
        Resolver communication callback - determines whether resolution attempt was successful.
        Aggravates to catastrophe in case of failure.

        @param msg: callback message - whether resolution of connection failure was successful
        """
        if msg.data:
            # reset all counters -- new run
            self.last_gnss_msg_time = datetime.now()
            self.last_wifi_msg_time = datetime.now()
            self.last_internet_msg_time = datetime.now()
            self.active_monitoring = True
        else:
            self.interrupt_reason_pub.publish(config.CONNECTION_CATA)
            self.aggravate_pub.publish(config.CONNECTION_CATA)

    def check_wifi_disconnect(self, wifi_info):
        """
        Checks for WiFi disconnects.

        @param wifi_info: information about the current WiFi connection
        @return: whether WiFi disconnect has been recognized
        """
        if wifi_info.link_quality == wifi_info.signal_level == wifi_info.bit_rate == 0:
            rospy.loginfo("detected wifi disconnect..")
            self.contingency_pub.publish(config.CONNECTION_FAILURE_FOUR)
            self.active_monitoring = False
            return True
        return False

    def check_internet_disconnect(self, internet_info):
        """
        Checks for internet disconnects.

        @param internet_info: information about the current internet connection
        @return: whether internet disconnect has been recognized
        """
        if internet_info.download == internet_info.upload == 0:
            rospy.loginfo("detected internet disconnect..")
            self.contingency_pub.publish(config.CONNECTION_FAILURE_FIVE)
            self.active_monitoring = False
            return True
        return False

    def check_link_quality(self, link_quality):
        """
        Checks the WiFi link quality.

        @param link_quality: WiFi link quality in percent
        """
        if link_quality < config.CRITICALLY_BAD_WIFI_LINK_THRESH:
            rospy.loginfo("detected critically bad wifi link of %s%% - should be fixed..", link_quality)
            self.contingency_pub.publish(config.CONNECTION_FAILURE_ONE)
            self.active_monitoring = False
        elif link_quality < config.BAD_WIFI_LINK_THRESH:
            rospy.loginfo("detected bad wifi link of %s%%..", link_quality)
            self.robot_info_pub.publish("detected bad wifi link of " + str(link_quality) + "%")
        elif link_quality < config.BELOW_AVG_WIFI_LINK:
            rospy.loginfo("detected below-average wifi link quality of %s%%..", link_quality)
            self.robot_info_pub.publish("detected below-average wifi link quality of " + str(link_quality) + "%")

    def check_signal_level(self, signal_level):
        """
        Checks the WiFi signal level.

        @param signal_level: WiFi signal level in dBm
        """
        if signal_level <= config.CRITICALLY_BAD_WIFI_SIGNAL_THRESH:
            rospy.loginfo("detected critically bad wifi signal level of %s dBm - no signal..", signal_level)
            self.contingency_pub.publish(config.CONNECTION_FAILURE_TWO)
            self.active_monitoring = False
        elif signal_level <= config.VERY_LOW_WIFI_SIGNAL_THRESH:
            rospy.loginfo("detected very low wifi signal level of %s dBm", signal_level)
            self.robot_info_pub.publish("detected very low wifi signal level of " + str(signal_level) + "dBm")
        elif signal_level < config.LOW_WIFI_SIGNAL_THRESH:
            rospy.loginfo("detected low wifi signal level of %s dBm", signal_level)
            self.robot_info_pub.publish("detected low wifi signal level of " + str(signal_level) + "dBm")

    def check_bit_rate(self, bit_rate):
        """
        Checks the WiFi bit rate.

        @param bit_rate: WiFi bit rate in Mb/s
        """
        if bit_rate < config.CRITICALLY_BAD_WIFI_BIT_RATE_THRESH:
            rospy.loginfo("detected critically bad wifi bit rate of %s Mb/s", bit_rate)
            self.contingency_pub.publish(config.CONNECTION_FAILURE_THREE)
            self.active_monitoring = False
        elif bit_rate < config.RATHER_LOW_WIFI_BIT_RATE_THRESH:
            rospy.loginfo("detected rather low wifi bit rate of %s Mb/s", bit_rate)
            self.robot_info_pub.publish("detected rather low wifi bit rate of " + str(bit_rate) + "Mb/s")

    def wifi_callback(self, wifi_info):
        """
        Called whenever new WiFi information are published - initiates WiFi connection monitoring.

        @param wifi_info: information about the current WiFi connection
        """
        self.last_wifi_msg_time = datetime.now()
        if self.active_monitoring:
            if config.VERBOSE_LOGGING:
                rospy.loginfo("link quality: %s%%, signal level: %s dBm, bit rate: %s Mb/s",
                              "{:4.2f}".format(wifi_info.link_quality), "{:4.2f}".format(wifi_info.signal_level),
                              "{:4.2f}".format(wifi_info.bit_rate))
            if not self.check_wifi_disconnect(wifi_info):
                self.check_link_quality(wifi_info.link_quality)
                self.check_signal_level(wifi_info.signal_level)
                self.check_bit_rate(wifi_info.bit_rate)

    def check_download_speed(self, download):
        """
        Checks the current download speed of the internet connection.

        @param download: current download speed in Mb/s
        """
        if download < config.CRITICALLY_BAD_DOWNLOAD_SPEED_THRESH:
            rospy.loginfo("detected critically bad download speed: %s Mb/s", download)
            self.contingency_pub.publish(config.CONNECTION_FAILURE_SIX)
            self.active_monitoring = False
        elif download < config.RATHER_LOW_DOWNLOAD_SPEED_THRESH:
            rospy.loginfo("detected rather low download speed: %s Mb/s", download)
            self.robot_info_pub.publish("detected rather low download speed of " + str(download) + " Mb/s")

    def check_upload_speed(self, upload):
        """
        Checks the current upload speed of the internet connection.

        @param upload: current upload speed in Mb/s
        """
        if upload < config.CRITICALLY_BAD_UPLOAD_SPEED_THRESH:
            rospy.loginfo("detected critically bad upload speed: %s Mb/s", upload)
            self.contingency_pub.publish(config.CONNECTION_FAILURE_SEVEN)
            self.active_monitoring = False
        elif upload < config.RATHER_LOW_UPLOAD_SPEED:
            rospy.loginfo("detected rather low upload speed: %s Mb/s", upload)
            self.robot_info_pub.publish("detected rather low upload speed of " + str(upload) + " Mb/s")

    def internet_callback(self, internet_info):
        """
        Called whenever new internet connectivity information are published - initiates connection monitoring.

        @param internet_info: information about the current internet connection
        @return:
        """
        self.last_internet_msg_time = datetime.now()
        if self.active_monitoring:
            if config.VERBOSE_LOGGING:
                rospy.loginfo("download: %s Mb/s, upload: %s Mb/s", internet_info.download, internet_info.upload)
            if not self.check_internet_disconnect(internet_info):
                self.check_download_speed(internet_info.download)
                self.check_upload_speed(internet_info.upload)


def node():
    """
    Connection failure monitoring node.
    """
    rospy.init_node('connection_monitoring')
    rospy.wait_for_message('SMACH_running', String)
    rospy.loginfo("launch connection monitoring..")
    ConnectionMonitoring()
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
