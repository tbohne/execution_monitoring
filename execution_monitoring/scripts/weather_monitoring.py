#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Bool
from execution_monitoring import util, config, secret_config
from arox_performance_parameters.msg import arox_operational_param
from datetime import datetime
import time
from sensor_msgs.msg import NavSatFix

from pyowm import OWM

class WeatherData:

    def __init__(self, time, status, cloudiness, humidity, pressure, rain_vol, snow_vol, wind, temperature, condition_code, icon_name, sunrise_time_sec, sunset_time_sec, sunrise_time, sunset_time):
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
        self.sunrise_time_sec = int(sunrise_time_sec)
        self.sunset_time_sec = int(sunset_time_sec)

        self.robot_info_pub = rospy.Publisher('/robot_info', String, queue_size=1)

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

        self.robot_info_pub.publish("weather_info: [time of observation: " + str(self.observation_time) + ", status: " + str(self.status) + ", cloudiness percentage: " \
            + str(self.cloudiness_percentage) + ", humidity percentage: " + str(self.humidity_percentage) + ",atmospheric pressure: " + str(self.atmospheric_pressure) \
            + "hPa, rain volume for the last 1 hour: " + str(self.rain_vol) + "mm, snow volume for the last 1 hour: " + str(self.snow_vol) + "mm, wind gust speed: " \
            + str(self.wind_gust_speed) + "m/s, wind speed: " + str(self.wind_speed) + "m/s, wind direction: " + str(self.wind_direction) + "deg., min. temperature: " \
            + str(self.min_temp) + " [deg. C], temperature: " + str(self.temp) + " [deg. C], max. temperature: " + str(self.max_temp) + " [deg. C], OWM weather condition code: " \
            + str(self.owm_weather_condition_code) + ", sunrise time: " + str(self.sunrise_time) + ", sunset time: " + str(self.sunset_time))
        rospy.loginfo("###############################################################################")


class WeatherMonitoring:

    def __init__(self):
        self.contingency_pub = rospy.Publisher('/contingency_preemption', String, queue_size=1)
        self.aggravate_pub = rospy.Publisher('/aggravate', String, queue_size=1)
        self.robot_info_pub = rospy.Publisher('/robot_info', String, queue_size=1)
        self.sim_info_pub = rospy.Publisher('/sim_info', String, queue_size=1)
        self.interrupt_reason_pub = rospy.Publisher('/interrupt_reason', String, queue_size=1)
        self.stop_waiting_pub = rospy.Publisher('/stop_waiting', String, queue_size=1)
        rospy.Subscriber('/fix', NavSatFix, self.gnss_callback, queue_size=1)
        rospy.Subscriber('/resolve_weather_failure_success', Bool, self.resolve_callback, queue_size=1)
        rospy.Subscriber('/toggle_rain_sim', String, self.rain_callback, queue_size=1)
        rospy.Subscriber('/toggle_snow_sim', String, self.snow_callback, queue_size=1)
        rospy.Subscriber('/toggle_wind_sim', String, self.wind_callback, queue_size=1)
        rospy.Subscriber('/toggle_low_temp_sim', String, self.low_temp_callback, queue_size=1)
        rospy.Subscriber('/toggle_thuderstorm_sim', String, self.thunderstorm_callback, queue_size=1)
        rospy.Subscriber('/toggle_sunset_sim', String, self.sunset_callback, queue_size=1)
        rospy.Subscriber('arox/ongoing_operation', arox_operational_param, self.operation_callback, queue_size=1)
        self.sim_rain = False
        self.sim_snow = False
        self.sim_wind = False
        self.sim_temp = False
        self.code_sim = False
        self.sunset_sim = False
        self.active_monitoring = True
        self.position = None
        self.operation_mode = None
        self.launch_weather_monitoring()

    def operation_callback(self, msg):
        self.operation_mode = msg.operation_mode

    def gnss_callback(self, nav_sat_fix):
        self.position = (nav_sat_fix.latitude, nav_sat_fix.longitude)

    def resolve_callback(self, msg):
        if not msg.data:
            self.interrupt_reason_pub.publish(config.WEATHER_CATA)
            self.aggravate_pub.publish(config.WEATHER_CATA)
    
    def rain_callback(self, msg):
        self.sim_rain = not self.sim_rain
    
    def snow_callback(self, msg):
        self.sim_snow = not self.sim_snow

    def wind_callback(self, msg):
        self.sim_wind = not self.sim_wind

    def low_temp_callback(self, msg):
        self.sim_temp = not self.sim_temp

    def thunderstorm_callback(self, msg):
        self.code_sim = not self.code_sim

    def sunset_callback(self, msg):
        self.sunset_sim = not self.sunset_sim

    def parse_weather_data(self, data):
        time = data.get_reference_time(timeformat='iso')
        status = data.get_detailed_status()
        clouds = data.get_clouds()
        humid = data.get_humidity()
        pressure = data.get_pressure()
        icon = data.get_weather_icon_name()
        sunrise_iso = data.get_sunrise_time('iso')
        sunset_iso = data.get_sunset_time('iso')

        # TODO: could perhaps separate monitoring and simulation later (node that publishes WeatherData.msg)
        # data that is monitored and therefore also simulated
        rain = data.get_rain()
        snow = data.get_snow()
        wind = data.get_wind()
        temp = data.get_temperature('celsius')
        code = data.get_weather_code()
        sunrise = data.get_sunrise_time()
        sunset = data.get_sunset_time()

        if self.sim_rain:
            self.sim_info_pub.publish("weather monitoring: sim rain")
            rain = {'1h': 8}
            self.sim_rain = False
        if self.sim_snow:
            self.sim_info_pub.publish("weather monitoring: sim snow")
            snow = {'1h': 4}
            self.sim_snow = False
        if self.sim_wind:
            self.sim_info_pub.publish("weather monitoring: sim wind")
            wind['gust'] = wind['speed'] = 27
            self.sim_wind = False
        if self.sim_temp:
            self.sim_info_pub.publish("weather monitoring: sim low temperature")
            temp = {'temp_min': -9, 'temp_max': -2, 'temp': -5}
            self.sim_temp = False
        if self.code_sim:
            self.sim_info_pub.publish("weather monitoring: sim weather code RAGGED_THUNDERSTORM")
            code = 221
            self.code_sim = False
        if self.sunset_sim:
            self.sim_info_pub.publish("weather monitoring: sim sunset")
            sunset = datetime.now().timestamp() #int((datetime.now() - datetime(1970, 1, 1)).total_seconds())
            self.sunset_sim = False

        return WeatherData(time, status, clouds, humid, pressure, rain, snow, wind, temp, code, icon, sunrise, sunset, sunrise_iso, sunset_iso)

    def monitor_rain_volume(self, rain_vol):
        """
        Monitors the current rain volume.

        :param rain_vol: rain volume per hour in mm
        :return: false if contingency, else true
        """
        # moderate rain: greater than 2.6 mm per hour, but less than 7.6 mm per hour
        if 7.6 > rain_vol > 2.6 and self.active_monitoring:
            self.robot_info_pub.publish(config.WEATHER_FAILURE_ONE)
        # heavy rain
        elif rain_vol > 7.6:
            if self.active_monitoring:
                self.contingency_pub.publish(config.WEATHER_FAILURE_TWO)
                self.active_monitoring = False
            return False
        return True

    def monitor_snow_volume(self, snow_vol):
        """
        Monitors the current snow volume.

        :param snow_vol: snow volume per hour in mm
        :return: false if contingency, else true
        """
        # moderate snow
        if 2.0 > snow_vol > 0.5 and self.active_monitoring:
            self.robot_info_pub.publish(config.WEATHER_FAILURE_FOUR)
        # heavy snow
        elif snow_vol > 2.0:
            if self.active_monitoring:
                self.contingency_pub.publish(config.WEATHER_FAILURE_FIVE)
                self.active_monitoring = False
            return False
        return True

    def monitor_wind(self, gust_speed, speed):
        """
        Monitors the current wind speed.

        :param gust_speed: gust speed in m/s
        :param speed: general wind speed in m/s
        :return: false if contingency, else true
        """
        if 14 > gust_speed > 11 or 14 > speed > 11 and self.active_monitoring:
            # strong breeze -> large branches in continuous motion
            self.robot_info_pub.publish(config.WEATHER_FAILURE_SIX)
        elif 20 > gust_speed > 14 or 20 > speed > 14 and self.active_monitoring:
            # gale -> whole trees in motion; inconvenience felt when walking against the wind; wind breaks twigs and small branches
            self.robot_info_pub.publish(config.WEATHER_FAILURE_SEVEN)
        elif 28 > gust_speed > 21 or 28 > speed > 21:
            # strong gale -> risk for structural damage
            if self.active_monitoring:
                self.contingency_pub.publish(config.WEATHER_FAILURE_EIGHT)
                self.active_monitoring = False
            return False
        elif 33 > gust_speed > 28 or 33 > speed > 28:
            # storm force -> very high risk for structural damage; larger trees blown over and uprooted
            self.contingency_pub.publish(config.WEATHER_FAILURE_NINE)
            self.active_monitoring = False
            return False
        elif gust_speed > 33 or speed > 33:
            # hurricane -> very high risk for severe and extensive structural damage
            if self.active_monitoring:
                self.contingency_pub.publish(config.WEATHER_FAILURE_TEN)
                self.active_monitoring = False
            return False
        return True

    def monitor_temperature(self, min_temp, max_temp, temp):
        """
        Monitors the current temperature.

        :param min_temp: current minimum temperature in the area [deg. C]
        :param max_temp: current maximum temperature in the area [deg. C]
        :param temp: current temperature [deg. C]
        :return: false if contingency, else true
        """
        # arbitrarily chosen, depends on sensors etc., should be configurable by the user
        if temp > 40 or max_temp > 40:
            if self.active_monitoring:
                self.contingency_pub.publish(config.WEATHER_FAILURE_ELEVEN)
                self.active_monitoring = False
            return False
        # arbitrarily chosen, depends on sensors etc., should be configurable by the user
        if temp < -5 or min_temp < -5:
            if self.active_monitoring:
                self.contingency_pub.publish(config.WEATHER_FAILURE_TWELVE)
                self.active_monitoring = False
            return False
        return True

    def monitor_owm_weather_condition_code(self, code):
        """
        Monitors the current OWM weather code.

        :param code: OWM weather code
        :return: false if contingency, else true
        """
        if code in [config.THUNDERSTORM_WITH_LIGHT_RAIN, config.THUNDERSTORM_WITH_RAIN, config.THUNDERSTORM_WITH_HEAVY_RAIN, config.LIGHT_THUNDERSTORM, config.THUNDERSTORM,
            config.HEAVY_THUNDERSTORM, config.RAGGED_THUNDERSTORM, config.THUNDERSTORM_WITH_LIGHT_DRIZZLE, config.THUNDERSTORM_WITH_DRIZZLE, config.THUNDERSTORM_WITH_HEAVY_DRIZZLE]:
            if self.active_monitoring:
                self.contingency_pub.publish(config.WEATHER_FAILURE_THIRTEEN)
                self.active_monitoring = False
            return False
        elif code in [config.HEAVY_INTENSITY_RAIN, config.VERY_HEAVY_RAIN, config.EXTREME_RAIN, config.FREEZING_RAIN, config.HEAVY_INTENSITY_SHOWER_RAIN, config.RAGGED_SHOWER_RAIN]:
            if self.active_monitoring:
                self.contingency_pub.publish(config.WEATHER_FAILURE_TWO)
                self.active_monitoring = False
            return False
        elif code in [config.SNOW, config.HEAVY_SNOW, config.RAIN_AND_SNOW, config.HEAVY_SHOWER_SNOW]:
            if self.active_monitoring:
                self.contingency_pub.publish(config.WEATHER_FAILURE_FIVE)
                self.active_monitoring = False
            return False
        elif code == config.TORNADO:
            if self.active_monitoring:
                self.contingency_pub.publish(config.WEATHER_FAILURE_FOURTEEN)
                self.active_monitoring = False
            return False
        elif code == config.SQUALLS:
            if self.active_monitoring:
                self.contingency_pub.publish(config.WEATHER_FAILURE_SEVEN)
                self.active_monitoring = False
            return False
        elif code in [config.MIST, config.SMOKE, config.FOG]:
            if self.active_monitoring:
                self.contingency_pub.publish(config.WEATHER_FAILURE_FIFTEEN)
                self.active_monitoring = False
            return False
        return True

    def monitor_sunrise_and_sunset(self, sunrise_time_sec, sunset_time_sec):
        """
        Monitors the sunrise and sunset times.

        :param sunrise_time_sec: sunrise time in seconds (UNIX timestamp)
        :param sunset_time_sec: sunset time in seconds (UNIX timestamp)
        :return: false if contingency, else true
        """
        time_in_seconds = int((datetime.now() - datetime(1970, 1, 1)).total_seconds())

        if sunrise_time_sec > time_in_seconds:
            if self.active_monitoring:
                self.contingency_pub.publish(config.WEATHER_FAILURE_SIXTEEN)
                self.active_monitoring = False
            return False
        elif time_in_seconds > sunset_time_sec:
            if self.active_monitoring:
                self.contingency_pub.publish(config.WEATHER_FAILURE_SEVENTEEN)
                self.active_monitoring = False
            return False
        else:
            time_since_sunrise = time_in_seconds - sunrise_time_sec
            time_before_sunset = sunset_time_sec - time_in_seconds
            # rospy.loginfo("minutes since sunrise: %s", time_since_sunrise / 60)
            # rospy.loginfo("minutes before sunset: %s", time_before_sunset / 60)
            if time_before_sunset / 60 < 15:
                if self.active_monitoring:
                    self.contingency_pub.publish(config.WEATHER_FAILURE_EIGHTEEN)
                    self.active_monitoring = False
                return False
        return True

    def monitor_weather_data(self, weather_data):
        rain_ok = self.monitor_rain_volume(weather_data.rain_vol)
        snow_ok = self.monitor_snow_volume(weather_data.snow_vol)
        wind_ok = self.monitor_wind(weather_data.wind_gust_speed, weather_data.wind_speed)
        temp_ok = self.monitor_temperature(weather_data.min_temp, weather_data.max_temp, weather_data.temp)
        code_ok = self.monitor_owm_weather_condition_code(weather_data.owm_weather_condition_code)
        # TODO: activate for real tests in practice
        sun_ok =  True # self.monitor_sunrise_and_sunset(weather_data.sunrise_time_sec, weather_data.sunset_time_sec)
        if self.operation_mode == "waiting" and not self.active_monitoring and rain_ok and snow_ok and wind_ok and temp_ok and code_ok and sun_ok:
            self.active_monitoring = True
            rospy.loginfo("weather moderate again..")
            self.stop_waiting_pub.publish("weather moderate again..")
            self.robot_info_pub.publish("weather is moderate again..")

    def launch_weather_monitoring(self):

        owm = OWM(secret_config.OWM_API_KEY)
        cnt = 0

        while not rospy.is_shutdown():
            if owm.is_API_online() and self.position is not None:
                observation = owm.weather_at_coords(*self.position)
                rospy.loginfo("monitoring weather for: %s", observation.get_location().get_name())
                self.robot_info_pub.publish("monitoring weather for: " + observation.get_location().get_name())
                weather_data = self.parse_weather_data(observation.get_weather())
                if cnt % 50 == 0:
                    weather_data.log_complete_info()
                self.monitor_weather_data(weather_data)
                
                # TODO: implement forecast monitoring -> seek shelter in time..
                # fc = owm.three_hours_forecast(*self.position)
                # f = fc.get_forecast().get_weathers()
                # rospy.loginfo("forecast for the next few hours:")
                # forecasts = [self.parse_weather_data(f[i]) for i in range(2)]
                # for forecast in forecasts:
                #     forecast.log_complete_info()

            rospy.sleep(config.WEATHER_MONITORING_FREQUENCY)
            cnt += 1


def node():
    rospy.init_node('weather_monitoring')
    rospy.wait_for_message('SMACH_runnning', String)
    rospy.loginfo("launch weather monitoring..")
    # necessary to wait for actual start of operation -> should be removed later
    rospy.sleep(5)
    WeatherMonitoring()
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
