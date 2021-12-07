#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from execution_monitoring import util, config, secret_config
from datetime import datetime

from pyowm import OWM

LOCATION = 'Osnabrueck,DE'

class WeatherMonitoring:

    def __init__(self):
        self.contingency_pub = rospy.Publisher('/contingency_preemption', String, queue_size=1)
        self.robot_info_pub = rospy.Publisher('/robot_info', String, queue_size=1)
        self.launch_weather_monitoring()

    def parse_weather_data(self, data, forecast):
        rospy.loginfo("###############################################################################")
        # just for information
        rospy.loginfo("time of observation: %s", data.get_reference_time(timeformat='iso'))
        rospy.loginfo("status: %s", data.get_detailed_status())
        rospy.loginfo("cloudiness percentage: %s", data.get_clouds())
        rospy.loginfo("humidity percentage: %s", data.get_humidity())
        rospy.loginfo("atmospheric pressure: %s", data.get_pressure())

        # for monitoring
        rospy.loginfo("rain volume for the last 3 hours: %s mm", data.get_rain())
        rospy.loginfo("snow volume for the last 3 hours: %s mm", data.get_snow())
        rospy.loginfo("wind: %s", data.get_wind())
        rospy.loginfo("temperature (degrees C): %s", data.get_temperature('celsius'))
        rospy.loginfo("OWM weather condition code: %s", data.get_weather_code())
        rospy.loginfo("weather-related icon name: %s", data.get_weather_icon_name())
        if not forecast:
            rospy.loginfo("sunrise time: %s", data.get_sunrise_time('iso'))
            rospy.loginfo("sunset time: %s", data.get_sunset_time('iso'))
        rospy.loginfo("###############################################################################")

    def launch_weather_monitoring(self):
        
        owm = OWM(secret_config.OWM_API_KEY)

        if owm.is_API_online():
            observation = owm.weather_at_place(LOCATION)
            print("monitoring weather for: " + observation.get_location().get_name())
            self.parse_weather_data(observation.get_weather(), False)

            fc = owm.three_hours_forecast(LOCATION)
            f = fc.get_forecast().get_weathers()
            rospy.loginfo("forecast for the next few hours:")
            for i in range(0, 3):
                self.parse_weather_data(f[i], True)


def node():
    rospy.init_node('weather_monitoring')
    #rospy.wait_for_message('SMACH_runnning', String)
    rospy.loginfo("launch weather monitoring..")
    WeatherMonitoring()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
