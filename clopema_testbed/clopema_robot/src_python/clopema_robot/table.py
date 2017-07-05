# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Dec 2, 2013

tables = {1: ['t1_desk', 't1_leg_1', 't1_leg_2', 't1_leg_3', 't1_leg_4'],
          2: ['t2_desk', 't2_leg_1', 't2_leg_2', 't2_leg_3', 't2_leg_4'],
          3: ['t3_desk', 't3_leg_1', 't3_leg_2', 't3_leg_3', 't3_leg_4'],
          4: ['t4_desk', 't4_leg_1', 't4_leg_2', 't4_leg_3', 't4_leg_4']}


def get_table_frames(n):
    """Get table frames by table number.

    Arguments:
        n       : {int}
                  Table number, clopema has three tables, so the possible values
                  can be 1, 2, and 3.
    Returns:
        frames  : {list of strings}
                  List of table frames starting with center i.e. 'tX_desk'.
    """
    return tables[n]
