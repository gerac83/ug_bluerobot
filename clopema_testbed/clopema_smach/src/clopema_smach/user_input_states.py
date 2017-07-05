"""
user_input_states.py

Libor Wagner on February  1, 2013
"""

import roslib; roslib.load_manifest('clopema_smach')
import rospy
import smach
import signal

from smach import State

__all__ = ['YesNoState', 'ContinueState', 'GenericAskState']

class GenericAskState(State):
    """AstState places an arbitrary questions and maps its outcome to the given answer."""

    def __init__(self, msg = "Continue? [y/n] ", answers = {"y":"succeeded", "n":"aborted"}, timeout = 0):
        State.__init__(self)

        self._answers = answers
        self._msg = msg
        self._timeout = timeout

        self.register_outcomes(self._answers.values())

        if self._timeout:
            self.register_outcomes(['interrupted'])

    def interrupted(signum, frame):
        pass

    def execute(self, ud):
        outcome = None
        if self._timeout:
            signal.signal(signal.SIGALRM, self.interrupted)
            signal.alarm(self._timeout)
        try:
            while not outcome:
                ans = raw_input("%s" % self._msg)

                if ans in self._answers:
                    outcome = self._answers[ans]
        except:
            print 'interrupted'
            outcome = 'interrupted'
        signal.alarm(0)

        return outcome


class YesNoState(GenericAskState):
    """
    YesNoState ask user to enter y or n, then the outcome is succeeded for yes
    and aborted for no.
    """
    def __init__(self, msg):
        GenericAskState.__init__(self, msg + "[y/n]? ", answers = {'y':'succeeded', 'n':'aborted'})

class ContinueState(GenericAskState):
    """
    ContinueState prompts user to hit enter to continue. When q is entered befor
    the enter, the outcome is aborted, succeeded otherwise.
    """
    def __init__(self, msg = "Hit enter to continue, q and enter to abort."):
        GenericAskState.__init__(self, msg, answers = {"":"succeeded", "q":"aborted"})

