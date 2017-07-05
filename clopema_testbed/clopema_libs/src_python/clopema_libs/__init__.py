# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Nov 19, 2013

from docopt import docopt
from convex_hull import convex_hull
from min_bounding_box import min_bounding_box

if __name__ == '__main__':
    import doctest
    f,_ = doctest.testmod()
    print "Failed" if f > 0 else "OK"
