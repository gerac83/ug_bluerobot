#!/usr/bin/env bash

cat <<EOF
=============================== CloPeMa Error ==================================

This is a CloPeMa configuration error. The CLOPEMA_PARTNER environment variable
needs to be set.

Please set CLOPEMA_PARTNER enviroment variable to one of the partner abrevations
i.e. CERTH, CVUT, NEO, UG, UNIGE, by writting the following in yout terminal:

export CLOPEMA_PARTNER=CVUT

You will have to enter this in every terminal. Or, to make this permanent add it
to the .bashrc. By running the following command in the terminal:

echo "export CLOPEMA_PARTNER=CVUT">>~/.bashrc

================================================================================
EOF
exit 1
