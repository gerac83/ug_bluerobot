#!/usr/bin/env python

# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Oct 15, 2013

from clopema_libs.ui import ask
from clopema_robot import ClopemaRobotCommander

if __name__ == '__main__':
    group = ClopemaRobotCommander("arms")
    group.set_named_target('home_arms')
    group.plan()

    r = ask("Execute trajectory", {'Y':"Yes", "n":"No"})
    if r == 'y':
        group.go()

    group = ClopemaRobotCommander("ext")
    group.set_named_target('home_ext')
    group.plan()

    r = ask("Execute trajectory", {'Y':"Yes", "n":"No"})
    if r == 'y':
        group.go()
