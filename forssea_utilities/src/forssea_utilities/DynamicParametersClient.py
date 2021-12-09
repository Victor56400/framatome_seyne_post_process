import rospy
from sys import exit
from dynamic_reconfigure.client import Client

from forssea_utilities.DynamicParameters import shutdown_service_from_exception


class DynamicParametersClient(Client):
    def __init__(self, dyntype, name=None, timeout=30):
        try:
            super(DynamicParametersClient, self).__init__(name if name is not None else rospy.get_name(),
                                                          timeout=timeout)
        # Happens when a server object is destroyed and recreated in the same object
        except rospy.ServiceException as e:
            shutdown_service_from_exception(e)
            super(DynamicParametersClient, self).__init__(name if name is not None else rospy.get_name(),
                                                          timeout=timeout)
        # Happens when the server is not reachable
        except rospy.ROSException:
            rospy.logwarn("Could not initialize dynamic reconfigure client of type {}. The server might not be \
running.".format(dyntype.__name__))
        self.__dyntype = dyntype
        self.__var_names = sorted(self.__dyntype.level, key=self.__dyntype.level.get)

    def start(self, callback):
        if self.get_configuration(10) is None:
            rospy.logfatal("Could not get a configuration from the server {}.".format(self.name))
            exit(-1)
        self.set_config_callback(callback)
