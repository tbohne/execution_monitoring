#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from execution_monitoring import util, config, secret_config
from datetime import datetime

from pyowm import OWM

class WeatherMonitoring:

    def __init__(self):
        self.contingency_pub = rospy.Publisher('/contingency_preemption', String, queue_size=1)
        self.robot_info_pub = rospy.Publisher('/robot_info', String, queue_size=1)
        self.launch_weather_monitoring()

    def launch_weather_monitoring(self):
        
        owm = OWM(secret_config.OWM_API_KEY)

        if owm.is_API_online():
            observation = owm.weather_at_place('Osnabrueck,DE')
            w = observation.get_weather()
            print("location: " + observation.get_location().get_name())
            rospy.loginfo("time of observation: %s", w.get_reference_time(timeformat='iso'))
            rospy.loginfo("status: %s", w.get_detailed_status())
            rospy.loginfo("cloudiness percentage: %s", w.get_clouds())
            rospy.loginfo("rain volume for the last 3 hours: %s mm", w.get_rain())
            rospy.loginfo("snow volume for the last 3 hours: %s mm", w.get_snow())
            rospy.loginfo("wind: %s", w.get_wind())
            rospy.loginfo("humidity percentage: %s", w.get_humidity())
            rospy.loginfo("atmospheric pressure: %s", w.get_pressure())
            rospy.loginfo("temperature (degrees C): %s", w.get_temperature('celsius'))
            rospy.loginfo("OWM weather condition code: %s", w.get_weather_code())
            rospy.loginfo("weather-related icon name: %s", w.get_weather_icon_name())
            rospy.loginfo("sunrise time: %s", w.get_sunrise_time('iso'))
            rospy.loginfo("sunset time: %s", w.get_sunset_time('iso'))

            fc = owm.three_hours_forecast('Osnabrueck,DE')
            f = fc.get_forecast().get_weathers()
            rospy.loginfo("forecast for the next few hours:")
            for i in range(0, 3):
                rospy.loginfo("%s, status: %s", f[i].get_reference_time('iso'), f[i].get_status())


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
