#!/usr/bin/env python

import rospy
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.server import Server
from rospy.impl.registration import get_service_manager


class DDynamicReconfigure:
    """
    Dynamic reconfigure server/client wrapper for python nodes. Allows easier parameter updates in callbacks.
    """
    dyn_rec_srv = None  # type: Server
    dyn_rec_cli = None  # type: Client

    def __init__(self, dyntype, name=None, server=True, timeout=30):
        """
        Constructor
        :param dyntype: type of dynamic reconfigure message
        :param name: topic of the dynamic reconfigure server
        :param server: set to True if server, False if client
        :param timeout: timeout after which the client stops trying to reach server
        """
        rospy.logwarn(
            "The class DDynamicReconfigure is deprecated It will be removed in the future. You should now use DynamicParametersServer and DynamicParametersClient.")
        self.server = server
        self.timeout = timeout
        if name is None:
            self.name = rospy.get_name()
        else:
            self.name = name
        self.dyntype = dyntype
        self.var_names = sorted(self.dyntype.level, key=self.dyntype.level.get)
        self.dyn_rec_srv = None
        self.dyn_rec_cli = None
        self.callback = None

    def start(self, callback):
        """
        Start function for server/client
        :param callback: callback function for client/server
        """
        self.callback = callback
        if self.server:
            try:
                self.dyn_rec_srv = Server(self.dyntype, self.__cb, namespace=self.name)
            # Happens when a server object is destroyed and recreated in the same object
            except rospy.ServiceException as e:
                resolved_name = e.message[e.message.find("[") + 1:e.message.find("]")]
                service = get_service_manager().get_service(resolved_name)
                service.shutdown()
                self.start(callback)
        else:
            try:
                self.dyn_rec_cli = Client(self.name, timeout=self.timeout, config_callback=self.__cb)
            # Happens when a server object is destroyed and recreated in the same object
            except rospy.ServiceException as e:
                resolved_name = e.message[e.message.find("[") + 1:e.message.find("]")]
                service = get_service_manager().get_service(resolved_name)
                service.shutdown()
                self.start(callback)
            # Happens when the server is not reachable
            except rospy.ROSException:
                rospy.logwarn("Could not initialize dynamic reconfigure client of type {}. The server might not be \
running.".format(self.dyntype.__name__))

    def __cb(self, config, level):
        rospy.logwarn(
            "The class DDynamicReconfigure is deprecated It will be removed in the future. You should now use DynamicParametersServer and DynamicParametersClient.")
        self.callback(config, level)
        return config
