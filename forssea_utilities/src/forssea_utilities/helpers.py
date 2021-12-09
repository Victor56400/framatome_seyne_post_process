import numpy as np
import rospy as rp

from forssea_utilities.Environment import Environment as Env


class Environment(Env):
    def __init__(self, namespace="/environment"):
        super(Environment, self).__init__(namespace)
        rp.logwarn("The environment class is deprecated. Please import it from forssea_utilities.Environment instead.")


def get_param(param_name, default_value, silent=False):
    """
    Search for the closest param name matching : private -> relative -> and goes up in the namespace hierarchy and
    returns the parameter's value if found or the default value.

    :param str param_name: name of the param to get
    :param default_value: default value to return if the parameter was not found
    :return value of param if found, default_value otherwise
    """
    p = rp.search_param(param_name)
    if p is not None:
        param = rp.get_param(p, default_value)
        rp.logdebug("Node found the parameter `{}` in the parameter server with value {}".format(param_name, param))
        return param
    else:
        if default_value is None:
            msg = "No default value set. This node might crash."
        else:
            msg = "Setting default value: {}".format(default_value)
        if silent:
            logfun = rp.logdebug
        else:
            logfun = rp.logwarn
        logfun("Node could not find the parameter `{}` in the parameter server. {}".format(param_name, msg))
        return default_value


def to_angle(vec):
    """
    Transforms a vector of values into a vector of angles between -pi and pi
    :param numpy.array vec: vector of values
    :return: vector of angles between -pi and pi
    :rtype: numpy.array
    """
    return (vec + np.pi) % (2 * np.pi) - np.pi
