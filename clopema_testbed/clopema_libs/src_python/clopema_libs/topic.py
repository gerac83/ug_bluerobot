# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Nov 19, 2013


import rospy
import threading


class Subscriber(rospy.Subscriber):

    def __init__(self, topic_name, topic_type, callback=None):
        self.cb = callback
        self.cond = threading.Condition()
        self.msg = None
        rospy.Subscriber.__init__(self, topic_name, topic_type, self._callback)

    def _callback(self, msg):
        self.msg = msg
        self.cond.acquire()
        self.cond.notify_all()
        self.cond.release()
        if self.cb is not None:
            self.cb(msg)

    def get_last(self):
        if self.msg is None:
            self.cond.acquire()
            self.cond.wait()
            self.cond.release()

        return self.msg

    def get_next(self):
        self.cond.acquire()
        self.cond.wait()
        self.cond.release()

        return self.msg

    def get_newer(self, stamp):
        while self.msg is None or self.msg.header.stamp.to_sec() < stamp.to_sec():
            self.cond.acquire()
            self.cond.wait()
            self.cond.release()
        return self.msg


class TopicBuffer(rospy.Subscriber):

    def __init__(self, topic_name, topic_type):
        self.cond = threading.Condition()
        self.msg = None
        rospy.Subscriber.__init__(self, topic_name, topic_type, self._callback)

    def _callback(self, msg):
        self.msg = msg
        self.cond.acquire()
        self.cond.notify_all()
        self.cond.release()

    def get_last(self):
        if self.msg is None:
            self.cond.acquire()
            self.cond.wait()
            self.cond.release()

        return self.msg

    def get_newer(self, stamp=None, drop=0):
        if stamp is None:
            stamp = rospy.get_rostime()
        while self.msg is None or self.msg.header.stamp.to_sec() < stamp.to_sec() or drop > 0:
            self.cond.acquire()
            self.cond.wait()
            self.cond.release()
            drop -= 1
        return self.msg


if __name__ == '__main__':
    import doctest
    f,_ = doctest.testmod()
    print "Failed" if f > 0 else "OK"
