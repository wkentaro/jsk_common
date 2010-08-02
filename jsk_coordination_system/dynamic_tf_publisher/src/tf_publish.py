#!/usr/bin/env python
#
# publish static tf which is set by SetDynamicTF service
# publishing tf is uniquely by child_frame_id
#
# TODO: delete target tf
#       check tf tree consistency
#       change frequency by frame_id
#
import roslib; roslib.load_manifest('dynamic_tf_publisher')
import rospy

from dynamic_tf_publisher.srv import * # SetDynamicTF
from geometry_msgs.msg import TransformStamped,Quaternion,Vector3
import tf
import tf.msg
import thread

class dynamic_tf_publisher:
    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage)
        self.cur_tf = dict()
        self.original_parent = dict()
        self.listener = tf.TransformListener()
        self.tf_sleep_time = 1.0
        self.lockobj = thread.allocate_lock()
        rospy.Service('/set_dynamic_tf', SetDynamicTF, self.set_tf)
        rospy.Service('/assoc_tf', AssocTF, self.assoc)
        rospy.Service('/dissoc_tf', DissocTF, self.dissoc)

    def publish_tf(self):
        self.lockobj.acquire()
        time = rospy.Time.now()
        tfm = tf.msg.tfMessage()
        for frame_id in self.cur_tf.keys():
            pose = self.cur_tf[frame_id]
            pose.header.stamp = time
            tfm.transforms.append(pose)
        self.pub_tf.publish(tfm)
        self.lockobj.release()

    def assoc(self,req):
        if (not self.cur_tf.has_key(req.child_frame)) or self.cur_tf.has_key[req.child_frame] == req.parent_frame:
            return AssocTFResponse()
        print "assoc %s -> %s"%(req.parent_frame, req.child_frame)
        self.listener.waitForTransform(req.parent_frame, req.child_frame, req.header.stamp, rospy.Duration(1.0))
        ts = TransformStamped()
        (trans,rot) = self.listener.lookupTransform(req.parent_frame, req.child_frame, req.header.stamp)
        ts.transform.translation = Vector3(*trans)
        ts.transform.rotation = Quaternion(*rot)
        ts.header.stamp = req.header.stamp
        ts.header.frame_id = req.parent_frame
        ts.child_frame_id = req.child_frame
        self.lockobj.acquire()
        self.original_parent[req.child_frame] = self.cur_tf[req.child_frame].header.frame_id
        self.cur_tf[req.child_frame] = ts
        self.lockobj.release()
        self.publish_tf()
        return AssocTFResponse()

    def dissoc(self,req):
        areq = None
        self.lockobj.acquire()
        if self.original_parent.has_key(req.frame_id):
            areq = AssocTFRequest()
            areq.header = req.header
            areq.child_frame = req.frame_id
            areq.parent_frame = self.original_parent[req.frame_id]
        self.lockobj.release()
        if areq:
            self.assoc(areq)
            self.original_parent.pop(req.frame_id) # remove 
        return DissocTFResponse()

    def set_tf(self,req):
        self.lockobj.acquire()
        # if not assocd
        if not self.original_parent.has_key(req.cur_tf.child_frame_id):
            self.tf_sleep_time = 1.0/req.freq
            self.cur_tf[req.cur_tf.child_frame_id] = req.cur_tf
            print "Latch [%s]/[%shz]"%(req.cur_tf.child_frame_id,req.freq)
        self.lockobj.release()
        self.publish_tf()
        return SetDynamicTFResponse()

    def publish_and_sleep(self):
        self.publish_tf()
        rospy.sleep(self.tf_sleep_time)


if __name__ == "__main__":
    rospy.init_node('tf_publish_server')
    pub = dynamic_tf_publisher()
    while not rospy.is_shutdown():
        pub.publish_and_sleep()
    print "exit"

