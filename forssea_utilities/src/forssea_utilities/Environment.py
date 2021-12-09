import numpy as np
import rospy as rp
from forssea_msgs.cfg import EnvironmentConfig
from geographic_msgs.msg import GeoPoint
from pymap3d import geodetic2enu, geodetic2ned
from threading import Thread
from math import isnan

from forssea_utilities.DynamicParametersClient import DynamicParametersClient


class Environment(object):
    """
    Environment parameter manager. Should be used to access environment variables.
    """

    def __init__(self, namespace="/environment", in_thread=False):
        """
        Constructor
        :param str namespace: prefix for the environment parameters
        """
        self.__g = 9.81
        self.__gps0 = GeoPoint()
        self.__atm = 101325
        self.__wd = 1030
        self.__client = None
        self.__triggers = []   # list of callable
        rp.loginfo("Initial gps coordinates: {}".format(self.__gps0), logger_name="EnvironmentClass")
        if not in_thread:
            self.create_client(namespace, 15)
        else:
            t = Thread(target=self.create_client, args=(namespace, None,))
            t.start()

    def register_trigger(self, trigger_func):
        if trigger_func in self.__triggers:
            return
        self.__triggers.append(trigger_func)

    def unregister_trigger(self, trigger_func):
        if trigger_func in self.__triggers:
            self.__triggers.remove(trigger_func)

    def create_client(self, namespace, timeout):
        self.__client = DynamicParametersClient(EnvironmentConfig, name=namespace, timeout=timeout)
        self.__client.start(self.callback)
        rp.loginfo("Client to EnvironmentConfig({}) started".format(namespace))

    def callback(self, config):
        """
        Callback function for environment dynamic parameter client
        :param EnvironmentConfig config: parameter config
        """
        rp.loginfo("Environment values updated!\ng: %f\natm: %f\ngps: %f, %f, %f\nwd: %f", config["gravity"], config["zero_pressure"], config["target_latitude"], config["target_longitude"], config["target_altitude"], config["water_density"], logger_name="EnvironmentClass")
        self.__g = config["gravity"]
        self.__atm = config["zero_pressure"]
        if not isnan(config["target_latitude"]):
            self.__gps0.latitude = config["target_latitude"]
        else:
            rp.logerr("Target Latitude was NaN, ignored value !", logger_name="EnvironmentClass")
        if not isnan(config["target_longitude"]):
            self.__gps0.longitude = config["target_longitude"]
        else:
            rp.logerr("Target Longitude was NaN, ignored value !", logger_name="EnvironmentClass")
        if not isnan(config["target_altitude"]):
            self.__gps0.altitude = config["target_altitude"]
        else:
            rp.logerr("Target Altitude was NaN, ignored value !", logger_name="EnvironmentClass")
        self.__wd = config["water_density"]
        rp.logdebug("Environment variables successfully updated.")
        for trigger in self.__triggers:
            trigger()

    @property
    def g(self):
        return self.__g

    @property
    def gv(self):
        return np.array([0, 0, self.g])

    @property
    def gps0(self):
        return self.__gps0

    @gps0.setter
    def gps0(self, value):
        if not isinstance(value, GeoPoint):
            raise AttributeError("gps0 must be a GeoPoint")
        if self.__client is None:
            raise ConnectionRefusedError("environment server is not yet instantianted")

        self.__gps0 = value
        gps_config = {"target_latitude": self.__gps0.latitude,
                      "target_longitude": self.__gps0.longitude,
                      "target_altitude": self.__gps0.altitude}
        self.__client.update_configuration(gps_config)
        print ('yo')

    @property
    def wd(self):
        return self.__wd

    @property
    def atm(self):
        return self.__atm

    def local(self, gps, mode='enu'):
        """

        :type gps: GeoPoint
        """
        if mode == 'enu':
            return geodetic2enu(gps.latitude, gps.longitude, gps.altitude, self.__gps0.latitude, self.__gps0.longitude,
                                self.__gps0.altitude)
        elif mode == 'ned':
            return geodetic2ned(gps.latitude, gps.longitude, gps.altitude, self.__gps0.latitude, self.__gps0.longitude,
                                self.__gps0.altitude)
        else:
            raise ValueError("unknown mode {}".format(mode))
