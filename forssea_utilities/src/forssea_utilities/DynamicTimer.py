import rospy
import time



class DynamicTimer(rospy.Timer):
    """
    This class inherits from rospy.Timer, provides the same API, but also allow the user to change dynamically the period of the timer by calling the update_rate method.
    The rest of the class works the same as rospy.Timer
    """


    def __init__(self, period, callback, oneshot=False, reset=False):
        super(DynamicTimer, self).__init__(period, callback, oneshot=False, reset=False)
    

    def update_rate(self, rate):
        self.r = rospy.Rate(rate, reset=self._reset)

    def run(self): 
        self.r = rospy.Rate(1.0 / self._period.to_sec(), reset=self._reset) 
        current_expected = rospy.rostime.get_rostime() + self._period 
        last_expected, last_real, last_duration = None, None, None 
        while not rospy.core.is_shutdown() and not self._shutdown: 
            try: 
                self.r.sleep() 
            except rospy.exceptions.ROSInterruptException as e: 
                if rospy.core.is_shutdown(): 
                    break 
                raise 
            if self._shutdown: 
                break 
            current_real = rospy.rostime.get_rostime() 
            start = time.time() 
            self._callback(rospy.timer.TimerEvent(last_expected, last_real, current_expected, current_real, last_duration)) 
            if self._oneshot: 
                break 
            last_duration = time.time() - start 
            last_expected, last_real = current_expected, current_real 
            current_expected += self._period 
        
