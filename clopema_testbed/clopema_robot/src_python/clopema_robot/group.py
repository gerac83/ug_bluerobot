# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Apr 2, 2014

# ---- Constatns ------------------------------------------------------------- #

GROUP_FOR_EE = {"r1_ee":"r1_arm", "r2_ee":"r2_arm", "xtion1_link_ee":"r1_xtion", "xtion2_link_ee":"r2_xtion", "r1_tip_link":"r1_arm", "r2_tip_link":"r2_arm"}


# ---- Implementations ------------------------------------------------------- #

def infer_group_from_ee(ee_link):
    """Returns group name of the given link."""
    return GROUP_FOR_EE[ee_link]
