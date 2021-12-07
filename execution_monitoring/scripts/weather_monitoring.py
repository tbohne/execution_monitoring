#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from execution_monitoring import util, config, secret_config
from datetime import datetime

from pyowm import OWM

class WeatherData:

    def __init__(self, time, status, cloudiness, humidity, pressure, rain_vol, snow_vol, wind, temperature, condition_code, icon_name, sunrise_time, sunset_time):
        self.observation_time = time
        self.status = status
        self.cloudiness_percentage = cloudiness
        self.humidity_percentage = humidity
        self.atmospheric_pressure = pressure['press']
        self.rain_vol = rain_vol['1h'] if '1h' in rain_vol else 0
        self.snow_vol = snow_vol['1h'] if '1h' in snow_vol else 0
        self.wind_gust_speed = wind['gust']
        self.wind_speed = wind['speed']
        self.wind_direction = wind['deg']
        self.max_temp = temperature['temp_max']
        self.min_temp = temperature['temp_min']
        self.temp = temperature['temp']
        self.owm_weather_condition_code = condition_code
        self.weather_related_icon_name = icon_name
        self.sunrise_time = sunrise_time
        self.sunset_time = sunset_time

    def log_complete_info(self):
        rospy.loginfo("###############################################################################")
        # just for information
        rospy.loginfo("time of observation: %s", self.observation_time)
        rospy.loginfo("status: %s", self.status)
        rospy.loginfo("cloudiness percentage: %s", self.cloudiness_percentage)
        rospy.loginfo("humidity percentage: %s", self.humidity_percentage)
        rospy.loginfo("atmospheric pressure: %s hPa", self.atmospheric_pressure)

        # for monitoring
        rospy.loginfo("rain volume for the last 1 hour: %s mm", self.rain_vol)
        rospy.loginfo("snow volume for the last 1 hour: %s mm", self.snow_vol)
        rospy.loginfo("wind gust speed: %s m/s", self.wind_gust_speed)
        rospy.loginfo("wind speed: %s m/s", self.wind_speed)
        rospy.loginfo("wind direction: %s deg.", self.wind_direction)
        rospy.loginfo("min. temperature: %s [deg. C]", self.min_temp)
        rospy.loginfo("temperature: %s [deg. C]", self.temp)
        rospy.loginfo("max. temperature: %s [deg. C]", self.max_temp)
        rospy.loginfo("OWM weather condition code: %s", self.owm_weather_condition_code)
        rospy.loginfo("weather-related icon name: %s", self.weather_related_icon_name)
        rospy.loginfo("sunrise time: %s", self.sunrise_time)
        rospy.loginfo("sunset time: %s", self.sunset_time)
        rospy.loginfo("###############################################################################")


class WeatherMonitoring:

    def __init__(self):
        self.contingency_pub = rospy.Publisher('/contingency_preemption', String, queue_size=1)
        self.robot_info_pub = rospy.Publisher('/robot_info', String, queue_size=1)
        self.launch_weather_monitoring()

    def parse_weather_data(self, data):
        return WeatherData(data.get_reference_time(timeformat='iso'), data.get_detailed_status(), data.get_clouds(), data.get_humidity(),
            data.get_pressure(), data.get_rain(), data.get_snow(), data.get_wind(), data.get_temperature('celsius'), data.get_weather_code(),
            data.get_weather_icon_name(), data.get_sunrise_time('iso'), data.get_sunset_time('iso'))

    def monitor_weather_data(self, weather_data):

        # monitor rain volume
        # monitor snow volume
        # monitor wind (gust speed, speed, direction)
        # monitor temperature (min, temp, max)
        # monitor OWM weather condition code
        # monitor sunrise / sunset
        pass
        


    def launch_weather_monitoring(self):

        owm = OWM(secret_config.OWM_API_KEY)

        if owm.is_API_online():
            observation = owm.weather_at_place(config.LOCATION)
            print("monitoring weather for: " + observation.get_location().get_name())
            weather_data = self.parse_weather_data(observation.get_weather())
            weather_data.log_complete_info()
            self.monitor_weather_data(weather_data)

            fc = owm.three_hours_forecast(config.LOCATION)
            f = fc.get_forecast().get_weathers()
            rospy.loginfo("forecast for the next few hours:")
            forecasts = [self.parse_weather_data(f[i]) for i in range(2)]
        
        # for forecast in forecasts:
        #     forecast.log_complete_info()


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
