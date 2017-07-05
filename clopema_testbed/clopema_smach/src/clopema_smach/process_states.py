"""
process_states.py

A collection of states used to start and shutdown processes.

Libor Wagner on February  1, 2013
"""

import roslib; roslib.load_manifest('clopema_smach')
import rospy
import smach
import subprocess
import os
import signal

from smach import State


__all__ = ['StartProcessState', 'KillProcessState', 'LaunchState', 'ProcessState']

class ProcessState(State):
    """
    Process state will launch a process and finis after after the process is done.
    """

    def __init__(self, cmd):
        State.__init__(self, ['succeeded', 'aborted', 'preempted'])
        self._cmd = cmd

    def execute(self, ud):
        print self._cmd
        process = subprocess.Popen(self._cmd)
        process.wait()

        ret = process.poll()
        if ret == None:
            return 'succeeded'
        else:
            return 'aborted'

class StartProcessState(State):
    """
    StartProcessState starts a process and store its pid in the user data.
    """

    def __init__(self, cmd):
        State.__init__(self, ['succeeded', 'aborted', 'preempted'], output_keys = ['pid'])
        self._cmd = cmd

    def execute(self, ud):
        process = subprocess.Popen(self._cmd)
        ud.pid = process.pid

        ret = process.poll()
        if ret == None:
            return 'succeeded'
        else:
            return 'aborted'

class KillProcessState(State):
    """
    KillProcessState kills process which pid is passed in the user data.
    """

    def __init__(self):
        State.__init__(self, ['succeeded', 'aborted', 'preempted'], input_keys = ['pid'])

    def execute(self, ud):
        os.kill(ud.pid, signal.SIGTERM)
        return 'succeeded'

class LaunchState(State):
    """ LaunchState is a wrapper for launch files."""

    def __init__(self, pkg, launch, args = {}, xterm = False):
        State.__init__(self, ['succeeded', 'aborted', 'preempted'], output_keys = ['pid'])
        self._pkg = pkg
        self._launch = launch
        self._args = args
        self._xterm = xterm

    def add_arg(self, name, value):
        """Add argument passing."""
        self._args[name] = value

    def execute(self, ud):
        args = [n+":="+str(v) for (n,v) in self._args.iteritems()]
        cmd = ['roslaunch', self._pkg, self._launch] + args

        # Open in xterm window if reqested
        if self._xterm:
            cmd = ['xterm', '-e'] + cmd

        process = subprocess.Popen(cmd)

        ud.pid = process.pid
        ret = process.poll()
        if ret == None:
            return 'succeeded'
        else:
            return 'aborted'

