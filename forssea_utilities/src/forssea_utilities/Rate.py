from threading import Lock

import rospy as rp
from forssea_msgs.cfg import RateConfig

from forssea_utilities.DynamicParametersServer import DynamicParametersServer


class Rate(object):
    def __init__(self, namespace=None):
        self.__freq = 1
        self.__r = rp.Rate(self.__freq)
        self.__mutex = Lock()
        self.__external_callback = None
        self.__server = DynamicParametersServer(RateConfig, name=namespace)
        self.__server.start(self.__callback)

    def __callback(self, config, level):
        self.__mutex.acquire()
        self.__r = rp.Rate(config["frequency"])
        self.__freq = config["frequency"]
        self.__mutex.release()
        if self.__external_callback is not None:
            self.__external_callback(self.__freq)
        return config

    def sleep(self):
        self.__mutex.acquire()
        self.__r.sleep()
        self.__mutex.release()

    def set_callback(self, callback):
        self.__external_callback = callback

    @property
    def frequency(self):
        return self.__freq
