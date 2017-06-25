#!/usr/bin/env python

import numpy as np

import rospy
import tf
from geometry_msgs.msg import PoseStamped


class PoseToTF(object):

    def __init__(self):
        self.current_root_frame_id = rospy.get_param('~current_root_frame_id', 'camera_link')
        self.frame_id = rospy.get_param('~frame_id', 'tote_base')
        self.tf_listener = tf.listener.TransformListener()
        self.tf_broadcastor = tf.broadcaster.TransformBroadcaster()
        self.sub_pose = rospy.Subscriber('~input', PoseStamped, self._msg_cb)

        if rospy.get_param('~trust_last_pose', False):
            self.msg = None
            rate = rospy.get_param('~rate', 50)
            self.timer = rospy.Timer(rospy.Duration(1. / rate), self._timer_cb)

    def _msg_cb(self, msg):
        self.msg = msg
        self.publish(msg)

    def _timer_cb(self, event):
        if self.msg is None:
            return
        self.publish(self.msg, event.current_real)

    def publish(self, msg, stamp=None):
        stamp = msg.header.stamp if stamp is None else stamp
        rospy.loginfo('stamp: %s, pose: %s' % (stamp, msg.pose))

        # A = B \cdot C
        # A: current_root_frame_id -> frame_id
        # B: current_root_frame_id -> camera
        # C: camera -> frame_id

        try:
            T_b, R_b, = self.tf_listener.lookupTransform(
                self.current_root_frame_id,
                msg.header.frame_id,
                stamp)
        except Exception as e:
            rospy.logerr(e)
            return
        TR_b = self.tf_listener.fromTranslationRotation(T_b, R_b)

        pos = msg.pose.position
        ori = msg.pose.orientation
        T_c = (pos.x, pos.y, pos.z)
        R_c = (ori.x, ori.y, ori.z, ori.w)
        TR_c = self.tf_listener.fromTranslationRotation(T_c, R_c)

        TR_a = np.dot(TR_b, TR_c)
        TR_a_inv = tf.transformations.inverse_matrix(TR_a)
        T_a_inv = tf.transformations.translation_from_matrix(TR_a_inv)
        R_a_inv = tf.transformations.quaternion_from_matrix(TR_a_inv)

        self.tf_broadcastor.sendTransform(
            T_a_inv,
            R_a_inv,
            stamp,
            self.current_root_frame_id,
            self.frame_id,
        )


if __name__ == '__main__':
    rospy.init_node('pose_to_tf')
    app = PoseToTF()
    rospy.spin()
