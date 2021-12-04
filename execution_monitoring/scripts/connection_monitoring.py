#!/usr/bin/env python
import time
import rospy
import collections
from datetime import date, datetime
from std_msgs.msg import String, Bool
from execution_monitoring.msg import WiFi, Internet
from sensor_msgs.msg import NavSatFix
from execution_monitoring import util, config

class ConnectionMonitoring:

    def __init__(self):
        rospy.Subscriber('/wifi_connectivity_info', WiFi, self.wifi_callback, queue_size=1)
        rospy.Subscriber('/internet_connectivity_info', Internet, self.internet_callback, queue_size=1)
        rospy.Subscriber('/fix', NavSatFix, self.gnss_callback, queue_size=1)
        rospy.Subscriber('/resolve_wifi_failure_success', Bool, self.resolve_callback, queue_size=1)
        rospy.Subscriber('/resolve_internet_failure_success', Bool, self.resolve_callback, queue_size=1)
        self.contingency_pub = rospy.Publisher('/contingency_preemption', String, queue_size=1)
        self.catastrophe_pub = rospy.Publisher('/catastrophe_preemption', String, queue_size=1)
        self.robot_info_pub = rospy.Publisher('/robot_info', String, queue_size=1)
        self.active_monitoring = True
        self.last_gnss_msg_time = datetime.now()
        self.last_wifi_msg_time = datetime.now()
        self.last_internet_msg_time = datetime.now()
        self.gnss_covariance_history = collections.deque([], config.COVARIANCE_HISTORY_LENGTH)
        self.gnss_info_time = datetime.now()
        self.timeout_monitoring()

    def timeout_monitoring(self):
        while not rospy.is_shutdown():
            now = datetime.now()

            if self.active_monitoring:
                if self.last_gnss_msg_time is not None:
                    time_since_update = (now - self.last_gnss_msg_time).total_seconds()
                    rospy.loginfo("time since last GNSS update: %s s", time_since_update)
                    if time_since_update > config.GPS_TIMEOUT:
                        rospy.loginfo("detected GNSS timeout - no new update since %s s", time_since_update)
                        self.contingency_pub.publish(config.CONNECTION_FAILURE_EIGHT)
                        self.active_monitoring = False

                if self.last_wifi_msg_time is not None:
                    time_since_update = (now - self.last_wifi_msg_time).total_seconds()
                    rospy.loginfo("time since last wifi update: %s s", time_since_update)
                    if time_since_update > config.WIFI_TIMEOUT:
                        rospy.loginfo("detected wifi timeout - no new update since %s s", time_since_update)
                        self.contingency_pub.publish(config.CONNECTION_FAILURE_NINE)
                        self.active_monitoring = False

                if self.last_internet_msg_time is not None:
                    time_since_update = (now - self.last_internet_msg_time).total_seconds()
                    rospy.loginfo("time since last internet update: %s s", time_since_update)
                    if time_since_update > config.INTERNET_TIMEOUT:
                        rospy.loginfo("detected internet timeout - no new update since %s s", time_since_update)
                        self.contingency_pub.publish(config.CONNECTION_FAILURE_TEN)
                        self.active_monitoring = False

            rospy.sleep(5)


    def good_variances(nav_sat_fix):
        for i in range(0, len(nav_sat_fix.position_covariance), 4):
            if nav_sat_fix.position_covariance[i] != float('nan') and nav_sat_fix.position_covariance[i] > config.GOOD_VARIANCE_UB:
                return False
        return True
    
    def estimate_gnss_quality(self, nav_sat_fix):
        # GNSS quality estimation based on provided info
        #   - good (status GNSS_STATUS_GBAS_FIX, cov type >= config.GNSS_COVARIANCE_TYPE_DIAGONAL_KNOWN, each variance value (diagonal) <= LOW_COV_UB)
        #   - medium (status >= GNSS_STATUS_FIX, cov type >= config.GNSS_COVARIANCE_TYPE_APPROXIMATED, each value (diagonal) <= LOW_COV_UB)
        #   - low (status >= GNSS_STATUS_FIX, cov type == config.GNSS_COVARIANCE_TYPE_UNKNOWN)

        estimation_str = "Quality estimation of currently provided GNSS data: "

        if (datetime.now() - self.gnss_info_time).total_seconds() > 60:
            self.gnss_info_time = datetime.now()
            # GOOD QUALITY
            if nav_sat_fix.status.status == config.GNSS_STATUS_GBAS_FIX and nav_sat_fix.position_covariance_type >= config.GNSS_COVARIANCE_TYPE_DIAGONAL_KNOWN and self.good_variances(nav_sat_fix):
                self.robot_info_pub.publish(estimation_str + "good")
            # MEDIUM QUALITY
            if nav_sat_fix.status.status >= config.GNSS_STATUS_FIX and nav_sat_fix.position_covariance_type >= config.GNSS_COVARIANCE_TYPE_APPROXIMATED and self.good_variances(nav_sat_fix):
                self.robot_info_pub.publish(estimation_str + "medium")
            # LOW QUALITY
            if nav_sat_fix.status.status >= config.GNSS_STATUS_FIX and nav_sat_fix.position_covariance_type == config.GNSS_COVARIANCE_TYPE_UNKNOWN:
                self.robot_info_pub.publish(estimation_str + "low")

    def status_monitoring(self, nav_sat_fix):
        if nav_sat_fix.status.status not in [config.GNSS_STATUS_NO_FIX, config.GNSS_STATUS_FIX, config.GNSS_STATUS_SBAS_FIX, config.GNSS_STATUS_GBAS_FIX]:
            self.contingency_pub.publish(config.CONNECTION_FAILURE_ELEVEN)
        elif nav_sat_fix.status.status == config.GNSS_STATUS_NO_FIX:
            self.contingency_pub.publish(config.CONNECTION_FAILURE_TWELVE)
        elif nav_sat_fix.status.status == config.GNSS_STATUS_FIX:
            self.robot_info_pub.publish(config.CONNECTION_FAILURE_THIRTEEN)

    def service_monitoring(self, nav_sat_fix):
        if nav_sat_fix.status.service not in [config.GNSS_SERVICE_GPS, config.GNSS_SERVICE_GLONASS, config.GNSS_SERVICE_COMPASS, config.GNSS_SERVICE_GALILEO]:
            self.robot_info_pub.publish(config.CONNECTION_FAILURE_FOURTEEN)

    def belief_state_monitoring(self, nav_sat_fix):
        # lat / lng belief state monitoring
        if not nav_sat_fix.latitude or nav_sat_fix.latitude < config.LAT_LB or nav_sat_fix.latitude > config.LAT_UB:
            # lat (degrees): pos -> north of equator, neg -> south of equator
            self.contingency_pub.publish(config.CONNECTION_FAILURE_FIFTEEN)
        if not nav_sat_fix.longitude or nav_sat_fix.longitude < config.LNG_LB or nav_sat_fix.longitude > config.LNG_UB:
            # lng (degrees): pos -> east of prime meridian, neg -> west of prime meridian
            self.contingency_pub.publish(config.CONNECTION_FAILURE_SIXTEEN)

    def covariance_monitoring(self, nav_sat_fix):
        if nav_sat_fix.position_covariance_type not in [config.GNSS_COVARIANCE_TYPE_UNKNOWN, config.GNSS_COVARIANCE_TYPE_APPROXIMATED, config.GNSS_COVARIANCE_TYPE_DIAGONAL_KNOWN, config.GNSS_COVARIANCE_TYPE_KNOWN]:
            self.robot_info_pub.publish(config.CONNECTION_FAILURE_SEVENTEEN)

        # only consider the covariance values if they are somehow reasonable
        # -- in m^2 - defined relative to tangential plane through the reported position
        # -- components are "east", "north", and "up"

        # we can basically rate two things:
        #  - absolute values (good / bad)
        #  - progression over time

        # only monitor the progression for the diagonal as these are the most important values
        # --> append to history when >= DIAGONAL_KNOWN

        if nav_sat_fix.position_covariance_type >= config.GNSS_COVARIANCE_TYPE_DIAGONAL_KNOWN:
            # at least the diagoal is known, if not all -> append to history
            self.gnss_covariance_history.appendleft(nav_sat_fix.position_covariance)
        
        if self.analyze_covariance_history():
        
            if nav_sat_fix.position_covariance_type == config.GNSS_COVARIANCE_TYPE_KNOWN:
                # covariances can be completely used for quality assessment
                for cov in nav_sat_fix.position_covariance:
                    if cov != float('nan') and cov > config.HIGH_AREA_COVARIANCE:
                        self.contingency_pub.publish(config.CONNECTION_FAILURE_EIGHTEEN)
                        break
            elif nav_sat_fix.position_covariance_type == config.GNSS_COVARIANCE_TYPE_DIAGONAL_KNOWN:
                # GNSS receiver provided the variance of each measurement -> diagonal can be used for quality assessment
                for i in range(0, len(nav_sat_fix.position_covariance), 4):
                    if nav_sat_fix.position_covariance[i] != float('nan') and nav_sat_fix.position_covariance[i] > config.HIGH_AREA_COVARIANCE:
                        self.contingency_pub.publish(config.CONNECTION_FAILURE_EIGHTEEN)
                        break
            elif nav_sat_fix.position_covariance_type == config.GNSS_COVARIANCE_TYPE_APPROXIMATED:
                # can be considered, but with caution, without putting too much weight on it -> only an approximate value
                for cov in nav_sat_fix.position_covariance:
                    if cov != float('nan') and cov > config.HIGH_AREA_COVARIANCE:
                        # at least information, but not so critial, is only approximated
                        self.robot_info_pub.publish(config.CONNECTION_FAILURE_NINETEEN)
                        break

    def gnss_callback(self, nav_sat_fix):
        rospy.loginfo("GNSS monitoring..")
        self.last_gnss_msg_time = datetime.now()
        self.estimate_gnss_quality(nav_sat_fix)
        self.status_monitoring(nav_sat_fix)
        self.service_monitoring(nav_sat_fix)
        self.belief_state_monitoring(nav_sat_fix)
        self.covariance_monitoring(nav_sat_fix)

    def increasing_values_only(self, components):
        for i in range(1, len(components)):
            # both values feasible measures
            if components[i] != float('nan') and components[i - 1] != float('nan'):
                if components[i] <= components[i - 1]:
                    return False
        return True

    def get_oldest(self, components):
        for i in components:
                if i != float('nan'):
                    return i
    
    def get_latest(self, components):
        for i in components[::-1]:
                if i != float('nan'):
                    return i
        
    def analyze_covariance_history(self):
        east_components = [cov[0] for cov in self.gnss_covariance_history]
        north_components = [cov[4] for cov in self.gnss_covariance_history]
        up_components = [cov[8] for cov in self.gnss_covariance_history]

        # what's our definition for getting worse?
        #  - only increases
        #  - total increase between oldest and latest larger than 15 (configurable)

        if self.increasing_values_only(east_components) and self.get_latest(east_components) - self.get_oldest(east_components) > config.SIGNIFICANT_COVARIANCE_INCREASE:
            self.contingency_pub.publish(config.CONNECTION_FAILURE_TWENTY)
            return False
        if self.increasing_values_only(north_components) and self.get_latest(north_components) - self.get_oldest(north_components) > config.SIGNIFICANT_COVARIANCE_INCREASE:
            self.contingency_pub.publish(config.CONNECTION_FAILURE_TWENTY)
            return False
        if self.increasing_values_only(up_components) and self.get_latest(up_components) - self.get_oldest(up_components) > config.SIGNIFICANT_COVARIANCE_INCREASE:
            self.contingency_pub.publish(config.CONNECTION_FAILURE_TWENTY)
            return False

        return True

    def resolve_callback(self, msg):
        self.active_monitoring = True

    def check_wifi_disconnect(self, wifi_info):
        if wifi_info.link_quality == wifi_info.signal_level == wifi_info.bit_rate == 0:
            rospy.loginfo("detected wifi disconnect..")
            self.contingency_pub.publish(config.CONNECTION_FAILURE_FOUR)
            self.active_monitoring = False
            return True
        return False

    def check_internet_disconnect(self, internet_info):
        if internet_info.download == internet_info.upload == 0:
            rospy.loginfo("detected internet disconnect..")
            self.contingency_pub.publish(config.CONNECTION_FAILURE_FIVE)
            self.active_monitoring = False
            return True
        return False

    def check_link_quality(self, link_quality):
        if link_quality < 5:
            rospy.loginfo("detected critically bad wifi link of %s%% - should be fixed..", link_quality)
            self.contingency_pub.publish(config.CONNECTION_FAILURE_ONE)
            self.active_monitoring = False
        elif link_quality < 25:
            rospy.loginfo("detected bad wifi link of %s%%..", link_quality)
            self.robot_info_pub.publish("detected bad wifi link of " + str(link_quality) + "%")
        elif link_quality < 50:
            rospy.loginfo("detected below-average wifi link quality of %s%%..", link_quality)
            self.robot_info_pub.publish("detected below-average wifi link quality of " + str(link_quality) + "%")

    def check_signal_level(self, signal_level):
        if signal_level <= -90:
            rospy.loginfo("detected critically bad wifi signal level of %s dBm - no signal..", signal_level)
            self.contingency_pub.publish(config.CONNECTION_FAILURE_TWO)
            self.active_monitoring = False
        elif signal_level <= -80:
            rospy.loginfo("detected very low wifi signal level of %s dBm", signal_level)
            self.robot_info_pub.publish("detected very low wifi signal level of " + str(signal_level) + "dBm")
        elif signal_level < -75:
            rospy.loginfo("detected low wifi signal level of %s dBm", signal_level)
            self.robot_info_pub.publish("detected low wifi signal level of " + str(signal_level) + "dBm")

    def check_bit_rate(self, bit_rate):
        if bit_rate < 1:
            rospy.loginfo("detected critically bad wifi bit rate of %s Mb/s", bit_rate)
            self.contingency_pub.publish(config.CONNECTION_FAILURE_THREE)
            self.active_monitoring = False
        elif bit_rate < 20:
            rospy.loginfo("detected rather low wifi bit rate of %s Mb/s", bit_rate)
            self.robot_info_pub.publish("detected rather low wifi bit rate of " + str(bit_rate) + "Mb/s")

    def wifi_callback(self, wifi_info):
        self.last_wifi_msg_time = datetime.now()
        if self.active_monitoring:
            rospy.loginfo("receiving new wifi info..")
            rospy.loginfo("link quality: %s%%, signal level: %s dBm, bit rate: %s Mb/s",  "{:4.2f}".format(wifi_info.link_quality),  "{:4.2f}".format(wifi_info.signal_level),  "{:4.2f}".format(wifi_info.bit_rate))
            if not self.check_wifi_disconnect(wifi_info):
                self.check_link_quality(wifi_info.link_quality)
                self.check_signal_level(wifi_info.signal_level)
                self.check_bit_rate(wifi_info.bit_rate)

    def check_download_speed(self, download):
        if download < 1:
            rospy.loginfo("detected critically bad download speed: %s Mb/s", download)
            self.contingency_pub.publish(config.CONNECTION_FAILURE_SIX)
            self.active_monitoring = False
        elif download < 40:
            rospy.loginfo("detected rather low download speed: %s Mb/s", download)
            self.robot_info_pub.publish("detected rather low download speed of " + str(download) + " Mb/s")

    def check_upload_speed(self, upload):
        if upload < 1:
            rospy.loginfo("detected critically bad upload speed: %s Mb/s", upload)
            self.contingency_pub.publish(config.CONNECTION_FAILURE_SEVEN)
            self.active_monitoring = False
        elif upload < 10:
            rospy.loginfo("detected rather low upload speed: %s Mb/s", upload)
            self.robot_info_pub.publish("detected rather low upload speed of " + str(upload) + " Mb/s")

    def internet_callback(self, internet_info):
        self.last_internet_msg_time = datetime.now()
        if self.active_monitoring:
            rospy.loginfo("receiving new internet connection info..")
            rospy.loginfo("download: %s Mb/s, upload: %s Mb/s", internet_info.download, internet_info.upload)
            if not self.check_internet_disconnect(internet_info):
                self.check_download_speed(internet_info.download)
                self.check_upload_speed(internet_info.upload)

def node():
    rospy.init_node('connection_monitoring')
    rospy.wait_for_message('SMACH_runnning', String)
    rospy.loginfo("launch connection monitoring..")
    ConnectionMonitoring()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
