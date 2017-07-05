# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Apr 2, 2014

# ---- Imports --------------------------------------------------------------- #

import yaml
import genpy
import scipy.misc
import os
import shutil
import datetime
from clopema_libs.np_utils import imgmsg_to_np


# ---- Implementations ------------------------------------------------------- #

def write_msg(msg, file_path):
    """Write single message to a file."""
    with open(file_path, "w") as f:
        f.write(str(msg))


def write_msgs(msgs, file_path):
    """Write all messages in the list."""
    with open(file_path, "w") as f:
        f.write("\n---\n".join([str(msg) for msg in msgs]))


def read_msg(file_path, message_type):
    """Read one (first) message from a file."""
    msgs = read_msgs(file_path, message_type)
    return msgs[0]


def read_msgs(file_path, message_type):
    """Read all messages from yaml file."""
    msgs = []
    with open(file_path, "r") as f:
        for s in yaml.load_all(f):
            msg = message_type()
            genpy.message.fill_message_args(msg, s)
            msgs.append(msg)
    return msgs


def write_image_msg(image_msg, file_path):
    """Write sensor_msgs/Image itn file. The format is deduced from the file
    extension

    INPUT
        image_msg       [sensor_msgs/Image]
        file_path       [String]

    OUTPUT
        success         [Bool]
    """
    image = imgmsg_to_np(image_msg)
    scipy.misc.imsave(file_path, image)


def prepare_output_directory(directory_path, force=False):
    """
    INPUT
        directory_path  [String]
        force           [Bool]
                        If true and the directory exist the data will be
                        overwritten.
    OUTPUT
        success         [Bool]
                        True if succesfull, if the directory does exist nad
                        force is set to False.
    """
    if not os.path.exists(directory_path):
        os.makedirs(directory_path)
    elif force:
        shutil.rmtree(directory_path)
        os.makedirs(directory_path)
    else:
        return False
    return True


def prepare_unique_directory(prefix):
    """Prepare unique directory based on current date and time.

    INPUT
        prefix          [String]
                        Prefix of the unique directory.
    OUTPUT
        directory_path  [String]
                        Name of the unique directory.
    """

    suffix = datetime.datetime.now().strftime("%y%m%d_%H%M%S")
    directory_path = "_".join([prefix, suffix])
    prepare_output_directory(directory_path)
    return directory_path
