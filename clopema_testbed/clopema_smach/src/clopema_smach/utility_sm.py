"""
utility_sm.py

This module contain utility functions to generate frequently used state
machines, such as plan then visualize ask user and execute.

Libor Wagner on February 19, 2013
"""
import roslib; roslib.load_manifest('clopema_smach')
import rospy
import smach

from smach         import *
from clopema_smach import *

__all__ = ['gensm_plan_vis_exec', 'gensm_grippers', 'gensm_withlaunch', 'gensm_withnode']

def gensm_plan_vis_exec(planning_state, input_keys=[], output_keys=[], visualize = True, confirm = True, execute = True, servo_off = False, sq = None, skip = False):
    """This generator will produce a state machine that will start with planning
    state. Produced trajectory will be visualized and executed after user
    confirmation."""

    sm = sq
    if not sm:
        sm = Sequence( outcomes = ['succeeded', 'aborted', 'preempted'], connector_outcome = 'succeeded')

    if skip:
        sm.register_outcomes(['skipped'])

    sm.register_input_keys(planning_state.get_registered_input_keys())
    sm.register_output_keys(planning_state.get_registered_output_keys())
    sm.register_input_keys(input_keys)
    sm.register_output_keys(output_keys)
    sm.register_outcomes(planning_state.get_registered_outcomes())

    if skip:
        qe = GenericAskState("Press enter to continue, r + enter to repeat, s to skip, and q + enter to abort.",
                answers = {'':'succeeded', 'r':'repeat', 'q':'aborted', 's':'skipped'})
    else:
        qe = GenericAskState("Press enter to continue, r + enter to repeat, and q + enter to abort.",
                answers = {'':'succeeded', 'r':'repeat', 'q':'aborted'})

    with sm:
        # Planning state
        Sequence.add("PLAN", planning_state)
        if visualize: Sequence.add("VISUALIZE", TrajectoryVisualizeState())
        if confirm and visualize:   Sequence.add("Q_EXECUTE", qe, transitions = {'repeat':'VISUALIZE'})
        if execute:   Sequence.add("EXECUTE", TrajectoryExecuteState())
        if servo_off: Sequence.add("SERVO_OFF", SetServoPowerOffState())

    return sm

def gensm_grippers(r1_state, r2_state):
    """Generates concurrent state machine that will set state to both grippers
    simultaneously."""

    cc = Concurrence(outcomes = ['succeeded', 'aborted', 'preempted'],
                 default_outcome = 'aborted',
                 outcome_map = {'succeeded':{'R1':'succeeded', 'R2':'succeeded'},
                                'preempted':{'R1':'preempted', 'R2':'preempted'}})
    with cc:
        Concurrence.add("R1", GripperState(1,r1_state))
        Concurrence.add("R2", GripperState(2,r2_state))

    return cc

def gensm_withlaunch(state, pkg, launch, args = {}, input_keys=[], output_keys=[], sq = None, **kargs):
    """Surrounds given state with launch and kill states."""

    sm = sq
    if not sm:
        sm = Sequence( outcomes = ['succeeded', 'aborted', 'preempted'], connector_outcome = 'succeeded')

    sm.register_input_keys(state.get_registered_input_keys())
    sm.register_output_keys(state.get_registered_output_keys())
    sm.register_input_keys(input_keys)
    sm.register_output_keys(output_keys)

    with sm:
        Sequence.add("LAUNCH", LaunchState(pkg, launch, args, **kargs))
        Sequence.add("EXECUTE", state, transitions={'aborted':'KILL','preempted':'KILL'})
        Sequence.add("KILL", KillProcessState())


    return sm

def gensm_withnode(state, pkg, node_type, node_name = None, args = {}, input_keys=[], output_keys=[], sq = None):
    sm = sq
    if not sm:
        sm = Sequence( outcomes = ['succeeded', 'aborted', 'preempted'], connector_outcome = 'succeeded')

    sm.register_input_keys(state.get_registered_input_keys())
    sm.register_output_keys(state.get_registered_output_keys())
    sm.register_input_keys(input_keys)
    sm.register_output_keys(output_keys)

    if node_name:
        args += [("__name", node_name)]

    rosargs = [n+":="+str(v) for (n,v) in args]
    cmd = ['rosrun', pkg, node_type] + rosargs

    with sm:
        Sequence.add("LAUNCH", StartProcessState(cmd))
        Sequence.add("EXECUTE", state, transitions={'aborted':'KILL','preempted':'KILL'})
        Sequence.add("KILL", KillProcessState())

    return sm

