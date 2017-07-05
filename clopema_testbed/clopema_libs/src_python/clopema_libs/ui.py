# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Nov 19, 2013

import string


def ask(message, options):
    """Ask user a question

    Arguments:
        message     : {string} Question message
        options     : {dict} Possible one letter options with description.
                      Default option should be upper case.
    Returns:
        A single lower case letter from options

    Example:
    >> ask("Would you like to continue?", {'Y':"Yes", 'n':"Now"})
    Would you like to continue? [Y,n]? y
    y
    """
    ret = None
    keys = ''.join(options.keys())
    default = keys.strip(string.lowercase)[0]
    while not ret:
        ans = raw_input(message + " [" + ",".join(options.keys()) + "]? ")

        if not ans and default is not "":
            ret = default.lower()
            break

        if ans.lower() in keys.lower():
            ret = ans

    return ret


if __name__ == '__main__':
    import doctest
    f,_ = doctest.testmod()
    print "Failed" if f > 0 else "OK"
