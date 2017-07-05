# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Nov 28, 2013

import rospy
import numpy as np
import tf2_ros
import cv2

from clopema_calibration.corners import refine_corners
from clopema_libs.transform import transform2T
from clopema_libs.topic import Subscriber
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError


class Filter(rospy.SubscribeListener):

    def __init__(self, topic_in, topic_out=None, lazy=True):
        """Initialise simple filter.

        Arguments:
            topic_in    : {tuple(string, topic type)}
            topic_out   : {tuple(string, topic type)}
            lazy        : {bool} optional
                          If set to False the input topic is imediately
                          subscribed, if set to True it waits for subscribers on
                          the output topic. Default is True. When output topic
                          is None the input topic is subscribed imediately
                          whatever the value is.
        """
        self.topic_in = topic_in
        self.topic_out = topic_out

        if self.topic_out is None:
            self.lazy = False
        else:
            self.lazy = lazy

    def start(self):
        # Prepare Subscriber and Publisher
        if self.lazy:
            self.sub = None
            self.pub = rospy.Publisher(self.topic_out[0], self.topic_out[1], self)
        else:
            self.sub = rospy.Subscriber(self.topic_in[0], self.topic_in[1], self.input_cb)
            if self.topic_out is not None:
                self.pub = rospy.Publisher(self.topic_out[0], self.topic_out[1])

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        if self.sub is None:
            self.sub = rospy.Subscriber(self.topic_in[0], self.topic_in[1], self.input_cb)
            rospy.loginfo("Subscribe: %s", self.sub.resolved_name)

    def peer_unsubscribe(self, topic_name, num_peers):
        if self.pub.get_num_connections() == 0 and self.sub is not None:
            rospy.loginfo("Unsubscribe: %s", self.sub.resolved_name)
            self.sub.unregister()
            self.sub = None

    def input_cb(self, msg):
        raise('input_cb not implemented!')


class ImageFilter(Filter):

    def __init__(self, **kwargs):
        """Initialise simple filter.

        Arguments:
            topic_out   : {tuple(string, topic type)} optional
            lazy        : {bool} optional
                          If set to False the input topic is imediately
                          subscribed, if set to True it waits for subscribers on
                          the output topic. Default is True. When output topic
                          is None the input topic is subscribed imediately
                          whatever the value is.

        Internal parameters:
            image_topic_name       : {string} optional, default: image
            camera_info_topic_name : {string} optional, default: camera_info

        """
        Filter.__init__(self, topic_in=(), **kwargs)
        self.image_topic_name = 'image'

        # Prepare CvBridge
        self.bridge = CvBridge()

    def start(self):
        # Prepare Subscriber and Publisher
        if self.lazy:
            self.sub = None
            self.pub = rospy.Publisher(self.topic_out[0], self.topic_out[1], self)
        else:
            self.sub = rospy.Subscriber(self.image_topic_name, Image, self.input_cb)
            if self.topic_out is not None:
                self.pub = rospy.Publisher(self.topic_out[0], self.topic_out[1])

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        if self.sub is None:
            self.sub = rospy.Subscriber(self.image_topic_name, Image, self.input_cb)
            rospy.loginfo("Subscribe: %s", self.sub.resolved_name)

    def peer_unsubscribe(self, topic_name, num_peers):
        if self.pub.get_num_connections() == 0 and self.sub is not None:
            rospy.loginfo("Unsubscribe: %s", self.sub.resolved_name)
            self.sub.unregister()
            self.sub = None

    def input_cb(self, msg):
        try:
            image = np.asarray(self.bridge.imgmsg_to_cv(msg, msg.encoding))
        except CvBridgeError, e:
            print e
            return

        # Get camera base_link transformation
        self.image_cb(msg, image)

    def image_cb(self, msg, image):
        raise('image_cb not implemented!')


class CameraFilter(Filter):

    def __init__(self, tfb=None, frame_id='base_link', **kwargs):
        """Initialise simple filter.

        Arguments:
            topic_out   : {tuple(string, topic type)} optional
            lazy        : {bool} optional
                          If set to False the input topic is imediately
                          subscribed, if set to True it waits for subscribers on
                          the output topic. Default is True. When output topic
                          is None the input topic is subscribed imediately
                          whatever the value is.
            tfb         : {tf2_ros.Buffer} optional
            frame_id    : {string} optional, default: base_link

        Internal parameters:
            image_topic_name       : {string} optional, default: image
            camera_info_topic_name : {string} optional, default: camera_info

        """
        Filter.__init__(self, topic_in=(), **kwargs)
        self.frame_id = frame_id
        self.image_topic_name = 'image'
        self.camera_info_topic_name = 'camera'

        # Prepare CvBridge
        self.bridge = CvBridge()

        # Prepare TF buffer if not given
        if tfb is None:
            self.tfb = tf2_ros.Buffer()
            tf2_ros.TransformListener(self.tfb)
        else:
            self.tfb = tfb

    def start(self):
        # Get camera info
        self.camera_info = rospy.wait_for_message(self.camera_info_topic_name, CameraInfo)
        self.P = np.array(self.camera_info.P).reshape(3,4)

        # Prepare Subscriber and Publisher
        if self.lazy:
            self.sub = None
            self.pub = rospy.Publisher(self.topic_out[0], self.topic_out[1], self)
        else:
            self.sub = rospy.Subscriber(self.image_topic_name, Image, self.input_cb)
            if self.topic_out is not None:
                self.pub = rospy.Publisher(self.topic_out[0], self.topic_out[1])

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        if self.sub is None:
            self.sub = rospy.Subscriber(self.image_topic_name, Image, self.input_cb)
            rospy.loginfo("Subscribe: %s", self.sub.resolved_name)

    def peer_unsubscribe(self, topic_name, num_peers):
        if self.pub.get_num_connections() == 0 and self.sub is not None:
            rospy.loginfo("Unsubscribe: %s", self.sub.resolved_name)
            self.sub.unregister()
            self.sub = None

    def input_cb(self, msg):
        try:
            image = np.asarray(self.bridge.imgmsg_to_cv(msg, msg.encoding))
        except CvBridgeError, e:
            print e
            return

        # Get camera base_link transformation
        rospy.sleep(0.1)  # The duration doesn't work
        tform = self.tfb.lookup_transform(msg.header.frame_id, self.frame_id, msg.header.stamp, timeout=rospy.Duration(10))
        T = transform2T(tform.transform)
        self.image_cb(msg, image, self.P, T)

    def image_cb(self, msg, image, P, T):
        raise('image_cb not implemented!')


class PolyFilter(CameraFilter):

    def __init__(self, corners, refine_corners=False, **kwargs):
        self.corners = corners
        self.refine_corners = refine_corners
        self.blur_size = (5,5)
        self.blur_sigma = 5
        self.corners_search_size = 10
        self.corners_search_scale = 0.25
        CameraFilter.__init__(self, **kwargs)

    def input_cb(self, msg):
        try:
            image = np.asarray(self.bridge.imgmsg_to_cv(msg, msg.encoding))
        except CvBridgeError, e:
            print e
            return

        # Get camera base_link transformation
        rospy.sleep(0.1)  # The duration doesn't work
        tform = self.tfb.lookup_transform(msg.header.frame_id, self.frame_id, msg.header.stamp, timeout=rospy.Duration(10))
        T = transform2T(tform.transform)

        # Apply transformation to the homogenous 3D points
        Cb = np.dot(T, self.corners)

        # Project point to the image plane
        Cc = np.dot(self.P, Cb)
        Cc = Cc / Cc[2]
        Cc = np.array(Cc[:2,:])

        # If required refine corners
        if self.refine_corners:
            gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
            gray_blured = cv2.GaussianBlur(gray, self.blur_size, self.blur_sigma)
            Cr,mask = refine_corners(gray_blured, Cc, self.corners_search_size, self.corners_search_scale)
        else:
            Cr = Cc

        self.image_cb(msg, image, self.P, T, Cr)


class XtionFilter(CameraFilter):

    def __init__(self, **kwargs):
        CameraFilter.__init__(self, **kwargs)
        self.depth_image_topic_name = 'depth'

    def start(self):
        self.dept_sub = Subscriber(self.dept_image_topic_name, Image)
        CameraFilter.start(self)

    def input_cb(self, msg):
        depth_msg = self.dept_sub.get_newer(msg.header.stamp)

        try:
            image = np.asarray(self.bridge.imgmsg_to_cv(msg, msg.encoding))
            depth = np.asarray(self.bridge.imgmsg_to_cv(depth_msg, depth_msg.encoding))
        except CvBridgeError, e:
            print e
            return

        # Get camera base_link transformation
        rospy.sleep(0.1)  # The duration doesn't work
        tform = self.tfb.lookup_transform(msg.header.frame_id, self.frame_id, msg.header.stamp, timeout=rospy.Duration(10))
        T = transform2T(tform.transform)
        self.msg = msg
        self.image = image
        self.depth = depth
        self.T = T
        self.image_cb(msg, image, depth, self.P, T)
