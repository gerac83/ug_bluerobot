# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Nov 14, 2013

import rospy
from datetime import timedelta
import ui


class Measure(object):

    def __init__(self, robot):
        self.robot = robot
        self.meas_start = 0
        self.meas_step = 1
        self.simulation = False
        self.confirm = True
        self.display = True
        self.stop = False

    def run(self, trajs):
        self.trajs = trajs
        self.t_start = rospy.get_time()

        # Number of trajectories and measurements
        self.N_moves = len(trajs)
        self.N_meas = (self.N_moves + 1 - self.meas_start)  / self.meas_step

        # Initialise the measurement
        self._initialise(self.N_meas)

        # Go to start position i.e. the first point of the first trajectory
        pos = _get_first_point(trajs[0])
        self.robot.set_start_state_to_current_state()
        self.robot.set_joint_value_target(pos)
        traj = self.robot.plan()

        # Add movement of external axis if pressent
        if "ext_axis" in pos:
            start_state = self.robot.get_current_state()
            traj = self.robot.append_ext_axis(start_state, traj, pos["ext_axis"])

        trajs.reverse()
        trajs.append(traj)
        trajs.reverse()

        # Reset counters
        self.meas_n = 0

        for i in range(0,self.N_moves + 1):
            time = rospy.get_time()
            t_measure = timedelta(seconds=round(time - self.t_start,0))

            if i > 2:
                t_remaining = timedelta(seconds=round(((time - self.t_start) / (i - 1)) * (self.N_moves - i)))
                rospy.loginfo("T: % 2d/% 2d, M: % 2d/% 2d, TM: %s, TR: %s", i, self.N_moves, self.meas_n + 1, self.N_meas, str(t_measure), str(t_remaining))
            else:
                rospy.loginfo("T: % 2d/% 2d, M: % 2d/% 2d, TM: %s", i, self.N_moves, self.meas_n + 1, self.N_meas, str(t_measure))

            self._pre_move(i)
            if self.stop:
                break

            # Move to next measure point
            self.robot.execute(trajs[i])

            # Decide whether to measure or not
            j = i - self.meas_start
            if j >= 0 and j % self.meas_step == 0:
                self._measure(self.meas_n)
                self.meas_n += 1

        # Finalise the measurement
        return self._finalise(self.meas_n)

    def _pre_move(self, n):
        if self.display:
            self.robot.display_planned_path(self.trajs[n])

        if self.confirm:
            r = ui.ask("Would you like to continue?", {'Y':'Yes', 'n':'No'})
            if r is 'n':
                self.stop = True
                return

        # Call user deffined pre_move function
        if not self.simulation:
            fun = getattr(self, "pre_move", None)
            if callable(fun):
                fun()
            elif n == 0:
                rospy.loginfo("Pre-move function not implemented!")

    def _post_move(self, n):
        if not self.simulation:
            fun = getattr(self, "post_move", None)
            if callable(fun):
                fun()
            elif n == 0:
                rospy.loginfo("Post-move function not implemented!")

    def _initialise(self, N):
        if not self.simulation:
            initialise = getattr(self, "initialise", None)
            if callable(initialise):
                initialise(N)
            else:
                rospy.loginfo("Initialise function not implemented!")

    def _measure(self, n):
        if not self.simulation:
            measure = getattr(self, "measure", None)
            if callable(measure):
                measure(n)
            else:
                rospy.logwarn("Measure function not implemented!")

    def _finalise(self, n):
        if not self.simulation:
            finalise = getattr(self, "finalise", None)
            if callable(finalise):
                return finalise(n)
            else:
                rospy.loginfo("Finalise function not implemented!")
                return None


def _get_first_point(traj):
    """Get first point of a trajectory."""
    names = traj.joint_trajectory.joint_names
    state = traj.joint_trajectory.points[0].positions
    return dict(zip(names, state))


if __name__ == '__main__':
    import doctest
    f,_ = doctest.testmod()
    print "Failed" if f > 0 else "OK"
