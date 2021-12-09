#!/usr/bin/env python
from abc import abstractmethod
from collections import deque
from enum import Enum
from threading import Thread
from threading import Lock
import rospy as rp


class GrafcetCondition(object):
    """
    Abstract class modeling a condition to be checked before passing a transition
    """
    def __init__(self, cond_id, grafcet):
        """"
        Constructor
        """
        self._id = cond_id
        self._grafcet = grafcet

    @abstractmethod
    def eval(self):
        """
        The abstract method to be implemented by the user : actual condition expression
        """
        pass


class GrafcetTransition(object):
    """
    Class modeling a transition between two steps
    """
    def __init__(self, condition, upstream_step_ids, downstream_step_ids):
        """
        Constructor
        Note : the step ids are initialized once and for all, whereas the step will be added by the grafcet
        """
        self.__condition = condition
        self.__upstream_step_ids = upstream_step_ids
        self.__upstream_steps = []
        self.__downstream_step_ids = downstream_step_ids
        self.__downstream_steps = []

    def set_condition(self, condition):
        self.__condition = condition

    def add_upstream_step(self, step):
        self.__upstream_steps.append(step)

    def get_upstream_step_ids(self):
        return self.__upstream_step_ids

    def add_downstream_step(self, step):
        self.__downstream_steps.append(step)

    def get_downstream_step_ids(self):
        return self.__downstream_step_ids

    def get_downstream_steps(self):
        return self.__downstream_steps

    def is_enabled(self):
        """
        A transition gets enabled once all its upstream steps are active
        """
        enabled = True
        for upstream_step in self.__upstream_steps:
            enabled = upstream_step.is_active()
            if not enabled:
                break

        return enabled

    def can_be_cleared(self):
        """
        A transition can be cleared if the related condition returns True
        """
        if self.__condition is None:
            return False

        return self.__condition.eval()


class ActionMode(Enum):
    RISING_EDGE = 1,
    CONTINUE = 2,
    FALLING_EDGE = 3


class Action(object):
    """
    Class modeling an action
    """
    def __init__(self, grafcet):
        self._initialized = False
        self._grafcet = grafcet

    def is_initialized(self):
        return self.__initialized

    @abstractmethod
    def initialize(self):
        self._initialized = True
        pass

    @abstractmethod
    def execute(self):
        pass


class GrafcetStep(object):
    """
    Class modeling a step
    """
    def __init__(self, step_id, transitions, action, action_mode=ActionMode.RISING_EDGE):
        """
        Constructor
        """
        self.__id = step_id
        self.__active = False
        self.__transitions = transitions
        self.__action = action
        self.__action_mode = action_mode

    def get_id(self):
        return self.__id

    def get_transitions(self):
        return self.__transitions

    def is_active(self):
        return self.__active

    def set_active(self, active):
        """
        The actual step activation (i.e. action execution) depends on the action mode defined
        """
        if self.__active == active and active:
            if self.__action_mode == ActionMode.CONTINUE:
                self._action()
        else:
            if self.__action_mode == ActionMode.RISING_EDGE and active:
                self._action()
            elif self.__action_mode == ActionMode.FALLING_EDGE and not active:
                self._action()
            elif self.__action_mode == ActionMode.CONTINUE and active:
                self._action(True)

            self.__active = active

    def evolve(self):
        """
        Check whether a downstream transition can be cleared and return the downstream steps
        Note : Only one transition can be cleared at a time
        """
        evolved = False
        downstream_steps = []
        for transition in self.__transitions:
            if transition.is_enabled() and transition.can_be_cleared():
                downstream_steps = transition.get_downstream_steps()
                evolved = True
                break

        return evolved, downstream_steps

    def _action(self, first_activation=False):
        """
        Execute the private action
        """
        if self.__action is None:
            return False

        rp.logdebug("GrafcetStep::_action() : executing S" + str(self.get_id()))
        return self.__action.execute()


class Grafcet(Thread):
    """
    Class modeling a grafcet structure and behavior
    """
    def __init__(self, name, frequency):
        self.__name = name
        self.__frequency = frequency  # Hz
        self.__steps = []
        self.__transitions = []

        self.__active_steps = []

        self._timer = None

        # Diagnostic purpose
        self._diag_step_ids = deque([], 5)
        self._ds_lock = Lock()

        self._setup()

    @property
    def frequency(self):
        return self.__frequency

    def get_step(self, step_id):
        step_list = filter(lambda x: x.get_id() == step_id, self.__steps)
        return step_list[0]

    def _add_step(self, step):

        if step is None:
            return

        # Make sure the step is not already existing
        step_id = step.get_id()
        for existing_step in self.__steps:
            if existing_step.get_id() == step_id:
                # Can never happen except when designing the grafcet
                return

        self.__steps.append(step)

        # If the step is active, add it to the corresponding list
        if step.is_active():
            self.__active_steps.append(step)

        # Record the step's transitions
        self.__transitions.extend(step.get_transitions())

    def _setup(self):
        # Call the specific_setup method, implemented by specific grafcets
        self._specific_setup()
        # Bind upstream and downstream steps to transitions
        self.__bind_transition_to_step()

    @abstractmethod
    def _specific_setup(self):
        pass

    def run(self):

        self._timer = rp.Timer(rp.Duration(1.0 / self.__frequency), lambda x: self._evolve())

    def _evolve(self):

        completed_steps = []
        new_active_steps = []

        for active_step in self.__active_steps:
            # Can the current step evolve?
            evolved, downstream_steps = active_step.evolve()
            if evolved:
                completed_steps.append(active_step)

                new_active_steps.extend(downstream_steps)

            else:
                # The step remains an active step
                new_active_steps.append(active_step)

        # Update the step states
        for completed_step in completed_steps:
            completed_step.set_active(False)

        # Update active_steps while removing duplicated
        self.__active_steps = list(set(new_active_steps))

        for active_step in self.__active_steps:
            active_step.set_active(True)

        # Diagnostics
        self._ds_lock.acquire()
        self._diag_step_ids.append(self.__active_steps[0].get_id())
        self._ds_lock.release()

    def __bind_transition_to_step(self):

        for transition in self.__transitions:
            step_ids = transition.get_upstream_step_ids()
            for step_id in step_ids:
                step = self.get_step(step_id)
                if step is not None:
                    transition.add_upstream_step(step)

            step_ids = transition.get_downstream_step_ids()
            for step_id in step_ids:
                step = self.get_step(step_id)
                if step is not None:
                    transition.add_downstream_step(step)

    def get_diag_step_ids(self):
        self._ds_lock.acquire()
        ds_ids = list(self._diag_step_ids)
        self._ds_lock.release()
        return ds_ids

    @abstractmethod
    def shutdown(self):
        pass
