# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Nov 28, 2013

from clopema_libs.filter import CameraFilter, XtionFilter
from clopema_libs.depth import d2X
from clopema_libs.plane import plane_fit_svd
from clopema_libs.polygon import poly_resize
from clopema_libs.roi import extract_poly
from clopema_libs.cv.draw import draw_points
from clopema_libs.geometry import plane_line_intersection, homegenous
from clopema_libs.camera import pix2ray, point2pix
from clopema_calibration.corners import refine_corners
from clopema_libs.pose import affine2pose
from visualization_msgs.msg import Marker

import cv2
import tf
import rospy
import numpy as np


class TableCalibration(XtionFilter):

    def __init__(self, corners, **keyargs):
        self.corners  = corners
        XtionFilter.__init__(self, lazy=False, **keyargs)
        self.marker_pub = rospy.Publisher("table_calibration", Marker)
        self.blur_size = (5,5)
        self.blur_sigma = 5
        self.corners_search_size = 5
        self.corners_search_scale = 0.25
        self.depth_margin = 10
        self.text_org = (10, 455)
        self.br = tf.TransformBroadcaster()

    def _fit_plane(self, d, corners, P):
        """Fit plane to depth points inside rectangle."""
        H,W = d.shape
        XYZ = d2X(P,d)

        points = extract_poly(XYZ, corners)
        points = points[:,np.nonzero(np.isfinite(points.sum(axis=0)))[0]]

        # Fit plane to selected points
        plane = plane_fit_svd(points.transpose())

        # Check direction of the normal vector and normalize
        if plane[2] > 0:
            plane = -plane
        plane = plane / np.linalg.norm(plane[0:3])
        return plane, 0.1

    def publish_marker(self, pose):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.scale.x = 0.8
        marker.scale.y = 0.8
        marker.scale.z = 0.01
        marker.color.b = 0.5
        marker.color.a = 0.5
        marker.pose = pose
        self.marker_pub.publish(marker)

    def image_cb(self, msg, image, depth, P, T):
        # Xb    Homegenous points in 3D in base frame
        # Xc    Homegenous points in 3D in camera frame
        # Xb_   Homegenous refined points in 3D in base frame
        # Xc_   Homegenous refined points in 3D in camera frame
        # U     Points in image plane
        # U_    Refined points in 3D plane
        # pc    Plane in camera frame
        # pb    Plane in base frame

        # Convert RGB to gray
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        gray_blured = cv2.GaussianBlur(gray, self.blur_size, self.blur_sigma)

        # Image to display
        depth_show = cv2.cvtColor(depth / np.nanmax(depth), cv2.COLOR_GRAY2RGB)

        # Apply transformation to the homogenous 3D points
        Xc = np.dot(T, self.corners)

        # Project point to the image plane
        U = point2pix(Xc, P)

        # Draw corner
        draw_points(image, U, 3, (255,0,0))

        # Refine initial cornes and draw them
        U_, mask = refine_corners(gray_blured, U, self.corners_search_size, self.corners_search_scale)
        draw_points(image, U_[:, np.nonzero(mask)[0]], 3, (0,255,0))
        draw_points(depth_show, U_[:, np.nonzero(mask)[0]], 3, (0,255,0))
        cv2.polylines(depth_show, [np.int0(poly_resize(U_, self.depth_margin).T)], True, (0,255,0))

        # Fit plane
        pc, err = self._fit_plane(depth, U_, P)
        self.plane_camera = pc

        # Convert to pint in 3d in the camera frame
        Xc_ = pix2ray(U_, P)
        for i, c in enumerate(Xc_.T):
            Xc_[:,i] = plane_line_intersection(pc, np.array((0,0,0)), c)
        Xc_ = homegenous(Xc_)

        # Convert point from camera frame to baselink
        Xb_ = np.dot(np.linalg.inv(T), Xc_)
        Xb_ = np.array(Xb_)

        # Publish frames
        for i, p in enumerate(Xb_.T):
            self.br.sendTransform((p[0], p[1], p[2]),
                                  (0,1,0,0),
                                  rospy.Time.now(),
                                  "p%d" % i,
                                  self.frame_id)

        # Convert plane to base frame and normalize
        pb = np.dot(T.T, pc).reshape(4,1)
        pb = pb / np.linalg.norm(pb[:3])

        # Find center of the table
        C = np.mean(Xb_[:3, :], axis=1).reshape(3,1)

        # Find rotation from plane
        vZ = pb[:3].reshape(3,1)
        vX = np.cross(vZ.T, [0,1,0]).T
        vY = np.cross(vZ.T, vX.T).T
        vX = vX / (np.linalg.norm(vX) * np.sign(vX[0]))
        vY = vY / (np.linalg.norm(vY) * np.sign(vY[1]))
        vZ = vZ * np.sign(vZ[2])
        r = np.array(np.hstack((vX,vY,vZ)))
        r = r.T

        # Get transformation for the table center
        tform = np.append(r.T, -C,axis=1)
        tform = np.append(tform, np.array([0, 0, 0, 1]).reshape(1,4), axis=0)

        self.tform = tform
        self.publish_marker(affine2pose(np.linalg.inv(tform)))

        # Compute Distance
        d = np.sqrt(np.sum((U - U_) ** 2, axis=0))
        d[np.nonzero(~np.array(mask))[0]] = np.nan
        cv2.putText(image, "error [px]: " + ', '.join(["%5.3f" % v for v in d]), self.text_org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), thickness=2)

        cv2.imshow('RGB Image', cv2.cvtColor(image,cv2.COLOR_RGB2BGR))
        cv2.imshow('Depth Image', depth_show)
        cv2.waitKey(1)


def rot_fit(V):
    """Fit rotation matrix on a three points.

    Arguments:
        V       : {(3,3) numpy array}

    Returns:
        R       : {(3,3) numpy array}
    """
    # Get vectors
    v1 = V[:,1] - V[:,0]
    v2 = V[:,2] - V[:,0]

    # Normalize
    v1_ = v1 / np.linalg.norm(v1)
    v2_ = v2 / np.linalg.norm(v2)

    # Get othogonal vector on these two, and rotate if necesary
    vZ  = np.cross(v1_.reshape(3), v2_.reshape(3))
    vZ_ = vZ / np.linalg.norm(vZ)

    # Get orthogonal vector ont v1_ and vZ_
    vY  = np.cross(v1_.reshape(3), vZ_)
    vY_ = vY / np.linalg.norm(vY)

    # Check directions
    if v1_[0] < 0:
        v1_ = -v1_

    if vY_[1] < 0:
        vY_ = -vY_

    if vZ_[2] < 0:
        vZ_ = -vZ_

    return np.array([v1_, vY_, vZ_]).transpose()


class TableCalibrationCheck(CameraFilter):

    def __init__(self, corners, **keyargs):
        self.corners  = corners
        CameraFilter.__init__(self, lazy=False, **keyargs)
        self.blur_size = (5,5)
        self.blur_sigma = 5
        self.corners_search_size = 10
        self.corners_search_scale = 0.25
        self.text_org = (10, 455)

    def image_cb(self, msg, image, P, T):
        # Convert RGB to gray
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        gray_blured = cv2.GaussianBlur(gray, self.blur_size, self.blur_sigma)

        # Apply transformation to the homogenous 3D points
        Cb = np.dot(T, self.corners)

        # Project point to the image plane
        Cc = np.dot(P, Cb)
        Cc = Cc / Cc[2]
        Cc = np.array(Cc[:2, :])

        # Draw corner
        draw_points(image, Cc, 3, (255,0,0))
        points = Cc;
        for i in range(0, len(points.T)):
            cv2.line(image, tuple(np.int0(points.T[i-1])), tuple(np.int0(points.T[i])), (255,0,0), 2);

        # Refine initial cornes and draw them
        Cr,mask = refine_corners(gray_blured, Cc, self.corners_search_size, self.corners_search_scale)

        # Draw corner
        draw_points(image, Cr[:, np.nonzero(mask)[0]], 3, (0,255,0))

        # Compute Distance
        d = np.sqrt(np.sum((Cc - Cr) ** 2, axis=0))
        d[np.nonzero(~np.array(mask))[0]] = np.nan
        cv2.putText(image, "error [px]: " + ', '.join(["%5.3f" % v for v in d]), self.text_org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), thickness=2)

        #image = cv2.resize(image, (0,0), fx=2.0, fy=2.0)
        cv2.imshow('Table Calibration Check', cv2.cvtColor(image,cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)
