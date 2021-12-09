import copy
from datetime import datetime
from os import getenv, remove, symlink, makedirs
from os.path import join, exists, isdir
from yaml import dump, load, YAMLError, Loader

import rospy
from dynamic_reconfigure.server import Server
from forssea_msgs.srv import SetString, SetStringResponse
from std_msgs.msg import Empty
from std_srvs.srv import Trigger, TriggerResponse

from forssea_utilities.DynamicParameters import shutdown_service_from_exception


class DynamicParametersServer(Server):
    def __init__(self, dyntype, name=None):
        try:
            super(DynamicParametersServer, self).__init__(dyntype, self.__cb,
                                                          namespace=name if name is not None else rospy.get_name())
        # Happens when a server object is destroyed and recreated in the same object
        except rospy.ServiceException as e:
            shutdown_service_from_exception(e)
            super(DynamicParametersServer, self).__init__(dyntype, self.__cb,
                                                          namespace=name if name is not None else rospy.get_name())
        self.__var_names = list(self.type.level.keys())
        self.__var_levels = list(self.type.level.values())
        self.__default_path = getenv("FORSSEA_PARAMETERS_FOLDER", join(getenv("HOME"), ".forssea/parameters"))
        self.__sub = rospy.Subscriber("/save_all", Empty, self.__save_all)
        self.__save_server = self.__init_service("save_parameters", SetString, self.__save_parameters)
        self.__load_server = self.__init_service("load_parameters", SetString, self.__load_parameters)
        self.__reset_server = self.__init_service("reset_parameters", Trigger, self.__reset_parameters)
        self.__load_from_file("")

    def __init_service(self, name, type, callback):
        try:
            return rospy.Service(self.ns + name, type, callback)
        except rospy.ServiceException as e:
            shutdown_service_from_exception(e)
            return rospy.Service(self.ns + name, type, callback)

    def start(self, callback):
        self.callback = callback
        self.callback(self.config, ~0)

    def get(self, config, level):
        if level in self.__var_levels:
            name = self.__unsafe_get_name(level)
            rospy.logdebug("Parameter {} updated to {}".format(name, config[name]))
            return config[name]
        else:
            rospy.logerr("No parameter has {} as a level".format(level))
            return None

    def get_name(self, level):
        if level in self.__var_levels:
            return self.__unsafe_get_name(level)
        else:
            rospy.logerr("No parameter has {} as a level".format(level))
            return None

    def __unsafe_get_name(self, level):
        return self.__var_names[self.__var_levels.index(level)]

    def __cb(self, config, level):
        return config

    def __save_parameters(self, req):
        return SetStringResponse(self.__save_to_file(req.string), "")

    def __load_parameters(self, req):
        return SetStringResponse(self.__load_from_file(req.string), "")

    def __reset_parameters(self, req):
        self.__reset_config(self.type.defaults)
        return TriggerResponse(True, "")

    def __save_all(self, msg):
        self.__save_to_file()

    def __reset_config(self, changes):
        with self.mutex:
            new_config = copy.deepcopy(self.config)
            new_config.update(changes)
            self._clamp(new_config)
            self._change_config(new_config, -1)

    def __save_to_file(self, path=""):
        path = self.__get_path(path, True)
        if path is not None:
            params = {}
            for name in self.__var_names:
                params[name] = {"type": self.type.type[name], "value": self.config[name]}
            f_name = join(path, datetime.utcnow().isoformat())
            fl_name = join(path, "last")
            with open(f_name, "w") as f:
                dump(params, f, default_flow_style=False)
            try:
                remove(fl_name)
            except OSError:
                pass
            symlink(f_name, fl_name)
            return True
        else:
            return False

    def __load_from_file(self, path):
        path = self.__get_path(path, False)
        if path is not None:
            params = None
            changes = {}
            try:
                with open(path) as f:
                    params = load(f, Loader=Loader)
            except YAMLError as e:
                rospy.logerr("Could not load file {} : {}".format(path, e))
            for name in self.__var_names:
                try:
                    changes[name] = params[name]["value"]
                except KeyError:
                    rospy.logerr("Invalid parameters file: missing parameter {}".format(name))
            self.__reset_config(changes)
            return True
        else:
            return False

    def __get_path(self, path_in, directory=True):
        if not path_in:
            working_path = join(self.__default_path, self.ns[1:])
        else:
            working_path = path_in
        if directory:
            if not exists(working_path):
                try:
                    makedirs(working_path)
                except OSError:
                    rospy.logwarn("The provided path {} does not exist and could not be created.".format(working_path))
                    return None
            if not isdir(working_path):
                rospy.logwarn("Expected directory path, got {}".format(path_in))
                return None
        else:
            if not exists(working_path):
                rospy.logwarn("The provided path {} does not exist.".format(path_in))
                return None
            if isdir(working_path):
                working_path = join(working_path, "last")
                if not exists(working_path):
                    rospy.logwarn("Expected file, got {} and could not find {}".format(path_in, working_path))
                    return None
        return working_path
