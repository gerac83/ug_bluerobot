# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Dec 2, 2013


class Xtion(object):

    def __init__(self, number, name):
        self.number                 = number
        self.name                   = name
        self.rgb_image_topic_name   = 'image'
        self.depth_topic_name       = 'depth'
        self.camera_info_topic_name = 'camera_info'
        self.ik_link_id             = None
        self.calib_taget_link_id          = None
        self.calib_source_link_id   = None
        self.group                  = None


xtions = dict()

xtions[1] = Xtion(1, "xtion1")
xtions[1].rgb_topic_name = "/xtion1/rgb/image_raw"
xtions[1].depth_topic_name = "/xtion1/depth/image"
xtions[1].camera_info_topic_name = "/xtion1/rgb/camera_info"
xtions[1].ik_link_id = "xtion1_link"
xtions[1].calib_target_link_id = "r1_link_6"
xtions[1].calib_source_link_id = "r1_link_1"
xtions[1].group = "r1_xtion"


xtions[2] = Xtion(2, "xtion2")
xtions[2].rgb_topic_name = "/xtion2/rgb/image_raw"
xtions[2].depth_topic_name = "/xtion2/depth/image"
xtions[2].camera_info_topic_name = "/xtion2/rgb/camera_info"
xtions[2].ik_link_id = "xtion2_link"
xtions[2].calib_target_link_id = "r2_link_6"
xtions[2].calib_source_link_id = "r2_link_1"
xtions[2].group = "r2_xtion"


# Old definitions
xtion = dict()
xtion[1] = dict()
xtion[1]['name'] = 'xtion1'
xtion[1]['link'] = 'xtion1_link'
xtion[1]['group'] = 'r1_xtion'
xtion[1]['rgb_image'] = '/xtion1/rgb/image_raw'
xtion[1]['rgb_info'] = '/xtion1/rgb/camera_info'
xtion[1]['depth_image'] = '/xtion1/depth/image'
xtion[1]['depth_info'] = '/xtion1/depth/camera_info'

xtion[2] = dict()
xtion[2]['name'] = 'xtion2'
xtion[2]['link'] = 'xtion2_link'
xtion[2]['group'] = 'r2_xtion'
xtion[2]['rgb_image'] = '/xtion2/rgb/image_raw'
xtion[2]['rgb_info'] = '/xtion2/rgb/camera_info'
xtion[2]['depth_image'] = '/xtion2/depth/image'
xtion[2]['depth_info'] = '/xtion2/depth/camera_info'


def get_xtion(number=0):
    return xtions[number]


def get_xtion_val(n, key):
    """Get value for particular xtion number and key.

    Parameters:
        n       : {int}
                  Xtion number
        key     : {string}
                  Value key can be one of: link, group, rgb_image, rgb_info,
                  depth_image and depth_info.
    Returns:
        val     : Value for that xtion number and key.
    """
    return xtion[n][key]
