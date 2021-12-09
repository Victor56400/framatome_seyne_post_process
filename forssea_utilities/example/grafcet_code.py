#!/usr/bin/env python

from forssea_utilities.grafcet import GrafcetStep
from forssea_utilities.grafcet import GrafcetTransition
from forssea_utilities.grafcet import GrafcetCondition
from forssea_utilities.grafcet import Grafcet
from forssea_utilities.grafcet import Action

    
class DriverS0(Action):
    def __init__(self, grafcet):
        super(DriverS0, self).__init__(grafcet)
        self.initialize()

    def execute(self):
        # To be implemented
        pass

                    
class DriverS0S1Condition(GrafcetCondition):
    def __init__(self, grafcet):
        super(DriverS0S1Condition, self).__init__("DriverS0S1", grafcet)

    def eval(self):
        # To be implemented
        return False

                    
class DriverS1(Action):
    def __init__(self, grafcet):
        super(DriverS1, self).__init__(grafcet)
        self.initialize()

    def execute(self):
        # To be implemented
        pass

                    
class DriverS1S2Condition(GrafcetCondition):
    def __init__(self, grafcet):
        super(DriverS1S2Condition, self).__init__("DriverS1S2", grafcet)

    def eval(self):
        # To be implemented
        return False

                    
class DriverS2(Action):
    def __init__(self, grafcet):
        super(DriverS2, self).__init__(grafcet)
        self.initialize()

    def execute(self):
        # To be implemented
        pass

                    
class DriverS2S3Condition(GrafcetCondition):
    def __init__(self, grafcet):
        super(DriverS2S3Condition, self).__init__("DriverS2S3", grafcet)

    def eval(self):
        # To be implemented
        return False

                    
class DriverS3(Action):
    def __init__(self, grafcet):
        super(DriverS3, self).__init__(grafcet)
        self.initialize()

    def execute(self):
        # To be implemented
        pass

                    
class DriverS3S4Condition(GrafcetCondition):
    def __init__(self, grafcet):
        super(DriverS3S4Condition, self).__init__("DriverS3S4", grafcet)

    def eval(self):
        # To be implemented
        return False

                    
class DriverS4(Action):
    def __init__(self, grafcet):
        super(DriverS4, self).__init__(grafcet)
        self.initialize()

    def execute(self):
        # To be implemented
        pass

                    
class DriverS4S3Condition(GrafcetCondition):
    def __init__(self, grafcet):
        super(DriverS4S3Condition, self).__init__("DriverS4S3", grafcet)

    def eval(self):
        # To be implemented
        return False

                    
class DriverS1S0Condition(GrafcetCondition):
    def __init__(self, grafcet):
        super(DriverS1S0Condition, self).__init__("DriverS1S0", grafcet)

    def eval(self):
        # To be implemented
        return False

                    
class DriverS3S30Condition(GrafcetCondition):
    def __init__(self, grafcet):
        super(DriverS3S30Condition, self).__init__("DriverS3S30", grafcet)

    def eval(self):
        # To be implemented
        return False

                    
class DriverS30(Action):
    def __init__(self, grafcet):
        super(DriverS30, self).__init__(grafcet)
        self.initialize()

    def execute(self):
        # To be implemented
        pass

                    
class DriverS30S3Condition(GrafcetCondition):
    def __init__(self, grafcet):
        super(DriverS30S3Condition, self).__init__("DriverS30S3", grafcet)

    def eval(self):
        # To be implemented
        return False

                    
class DriverS30S300Condition(GrafcetCondition):
    def __init__(self, grafcet):
        super(DriverS30S300Condition, self).__init__("DriverS30S300", grafcet)

    def eval(self):
        # To be implemented
        return False

                    
class DriverS300(Action):
    def __init__(self, grafcet):
        super(DriverS300, self).__init__(grafcet)
        self.initialize()

    def execute(self):
        # To be implemented
        pass

                    
class DriverS3S300Condition(GrafcetCondition):
    def __init__(self, grafcet):
        super(DriverS3S300Condition, self).__init__("DriverS3S300", grafcet)

    def eval(self):
        # To be implemented
        return False

                    
class DriverS300S0Condition(GrafcetCondition):
    def __init__(self, grafcet):
        super(DriverS300S0Condition, self).__init__("DriverS300S0", grafcet)

    def eval(self):
        # To be implemented
        return False

                    
class DriverS4S40Condition(GrafcetCondition):
    def __init__(self, grafcet):
        super(DriverS4S40Condition, self).__init__("DriverS4S40", grafcet)

    def eval(self):
        # To be implemented
        return False

                    
class DriverS40(Action):
    def __init__(self, grafcet):
        super(DriverS40, self).__init__(grafcet)
        self.initialize()

    def execute(self):
        # To be implemented
        pass

                    
class DriverS40S3Condition(GrafcetCondition):
    def __init__(self, grafcet):
        super(DriverS40S3Condition, self).__init__("DriverS40S3", grafcet)

    def eval(self):
        # To be implemented
        return False

                    
class DriverS40S300Condition(GrafcetCondition):
    def __init__(self, grafcet):
        super(DriverS40S300Condition, self).__init__("DriverS40S300", grafcet)

    def eval(self):
        # To be implemented
        return False

                    
class DriverS4S41Condition(GrafcetCondition):
    def __init__(self, grafcet):
        super(DriverS4S41Condition, self).__init__("DriverS4S41", grafcet)

    def eval(self):
        # To be implemented
        return False

                    
class DriverS41(Action):
    def __init__(self, grafcet):
        super(DriverS41, self).__init__(grafcet)
        self.initialize()

    def execute(self):
        # To be implemented
        pass

                    
class DriverS41S3Condition(GrafcetCondition):
    def __init__(self, grafcet):
        super(DriverS41S3Condition, self).__init__("DriverS41S3", grafcet)

    def eval(self):
        # To be implemented
        return False

                    
class DriverS41S300Condition(GrafcetCondition):
    def __init__(self, grafcet):
        super(DriverS41S300Condition, self).__init__("DriverS41S300", grafcet)

    def eval(self):
        # To be implemented
        return False

                    
class DriverGrafcet(Grafcet):
    def __init__(self, frequency):
        super(DriverGrafcet, self).__init__("DriverGrafcet", frequency)

    def _specific_setup(self):

        # Step 0
        transitions = []
        action = DriverS0(self)
                    
        # Transition S0 / S1 Condition
        upstream_step_ids = [0]
        downstream_step_ids = [1]
        transitions.append(GrafcetTransition(DriverS0S1Condition(self), upstream_step_ids, downstream_step_ids))
                                
        step_object = GrafcetStep(0, transitions, action)
        step_object.set_active(True)
        self._add_step(step_object)

        # Step 1
        transitions = []
        action = DriverS1(self)
                    
        # Transition S1 / S2 Condition
        upstream_step_ids = [1]
        downstream_step_ids = [2]
        transitions.append(GrafcetTransition(DriverS1S2Condition(self), upstream_step_ids, downstream_step_ids))
                                
        # Transition S1 / S0 Condition
        upstream_step_ids = [1]
        downstream_step_ids = [0]
        transitions.append(GrafcetTransition(DriverS1S0Condition(self), upstream_step_ids, downstream_step_ids))
                                
        step_object = GrafcetStep(1, transitions, action)
        step_object.set_active(False)
        self._add_step(step_object)

        # Step 2
        transitions = []
        action = DriverS2(self)
                    
        # Transition S2 / S3 Condition
        upstream_step_ids = [2]
        downstream_step_ids = [3]
        transitions.append(GrafcetTransition(DriverS2S3Condition(self), upstream_step_ids, downstream_step_ids))
                                
        step_object = GrafcetStep(2, transitions, action)
        step_object.set_active(False)
        self._add_step(step_object)

        # Step 3
        transitions = []
        action = DriverS3(self)
                    
        # Transition S3 / S4 Condition
        upstream_step_ids = [3]
        downstream_step_ids = [4]
        transitions.append(GrafcetTransition(DriverS3S4Condition(self), upstream_step_ids, downstream_step_ids))
                                
        # Transition S3 / S30 Condition
        upstream_step_ids = [3]
        downstream_step_ids = [30]
        transitions.append(GrafcetTransition(DriverS3S30Condition(self), upstream_step_ids, downstream_step_ids))
                                
        # Transition S3 / S300 Condition
        upstream_step_ids = [3]
        downstream_step_ids = [300]
        transitions.append(GrafcetTransition(DriverS3S300Condition(self), upstream_step_ids, downstream_step_ids))
                                
        step_object = GrafcetStep(3, transitions, action)
        step_object.set_active(False)
        self._add_step(step_object)

        # Step 4
        transitions = []
        action = DriverS4(self)
                    
        # Transition S4 / S3 Condition
        upstream_step_ids = [4]
        downstream_step_ids = [3]
        transitions.append(GrafcetTransition(DriverS4S3Condition(self), upstream_step_ids, downstream_step_ids))
                                
        # Transition S4 / S40 Condition
        upstream_step_ids = [4]
        downstream_step_ids = [40]
        transitions.append(GrafcetTransition(DriverS4S40Condition(self), upstream_step_ids, downstream_step_ids))
                                
        # Transition S4 / S41 Condition
        upstream_step_ids = [4]
        downstream_step_ids = [41]
        transitions.append(GrafcetTransition(DriverS4S41Condition(self), upstream_step_ids, downstream_step_ids))
                                
        step_object = GrafcetStep(4, transitions, action)
        step_object.set_active(False)
        self._add_step(step_object)

        # Step 30
        transitions = []
        action = DriverS30(self)
                    
        # Transition S30 / S3 Condition
        upstream_step_ids = [30]
        downstream_step_ids = [3]
        transitions.append(GrafcetTransition(DriverS30S3Condition(self), upstream_step_ids, downstream_step_ids))
                                
        # Transition S30 / S300 Condition
        upstream_step_ids = [30]
        downstream_step_ids = [300]
        transitions.append(GrafcetTransition(DriverS30S300Condition(self), upstream_step_ids, downstream_step_ids))
                                
        step_object = GrafcetStep(30, transitions, action)
        step_object.set_active(False)
        self._add_step(step_object)

        # Step 300
        transitions = []
        action = DriverS300(self)
                    
        # Transition S300 / S0 Condition
        upstream_step_ids = [300]
        downstream_step_ids = [0]
        transitions.append(GrafcetTransition(DriverS300S0Condition(self), upstream_step_ids, downstream_step_ids))
                                
        step_object = GrafcetStep(300, transitions, action)
        step_object.set_active(False)
        self._add_step(step_object)

        # Step 40
        transitions = []
        action = DriverS40(self)
                    
        # Transition S40 / S3 Condition
        upstream_step_ids = [40]
        downstream_step_ids = [3]
        transitions.append(GrafcetTransition(DriverS40S3Condition(self), upstream_step_ids, downstream_step_ids))
                                
        # Transition S40 / S300 Condition
        upstream_step_ids = [40]
        downstream_step_ids = [300]
        transitions.append(GrafcetTransition(DriverS40S300Condition(self), upstream_step_ids, downstream_step_ids))
                                
        step_object = GrafcetStep(40, transitions, action)
        step_object.set_active(False)
        self._add_step(step_object)

        # Step 41
        transitions = []
        action = DriverS41(self)
                    
        # Transition S41 / S3 Condition
        upstream_step_ids = [41]
        downstream_step_ids = [3]
        transitions.append(GrafcetTransition(DriverS41S3Condition(self), upstream_step_ids, downstream_step_ids))
                                
        # Transition S41 / S300 Condition
        upstream_step_ids = [41]
        downstream_step_ids = [300]
        transitions.append(GrafcetTransition(DriverS41S300Condition(self), upstream_step_ids, downstream_step_ids))
                                
        step_object = GrafcetStep(41, transitions, action)
        step_object.set_active(False)
        self._add_step(step_object)
