# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Ioan Sucan

try:
    # Try Python 2.7 behaviour first
    from StringIO import StringIO
except ImportError:
    # Use Python 3.x behaviour as fallback and choose the non-unicode version
    from io import BytesIO as StringIO

from .exception import MoveItCommanderException
from geometry_msgs.msg import Pose, PoseStamped, Transform
from moveit_msgs.msg import SamplingDistribution
import rospy
import tf


def msg_to_string(msg):
    buf = StringIO()
    msg.serialize(buf)
    return buf.getvalue()


def msg_from_string(msg, data):
    msg.deserialize(data)


def pose_to_list(pose_msg):
    pose = []
    pose.append(pose_msg.position.x)
    pose.append(pose_msg.position.y)
    pose.append(pose_msg.position.z)
    pose.append(pose_msg.orientation.x)
    pose.append(pose_msg.orientation.y)
    pose.append(pose_msg.orientation.z)
    pose.append(pose_msg.orientation.w)
    return pose


def list_to_pose(pose_list):
    pose_msg = Pose()
    if len(pose_list) == 7:
        pose_msg.position.x = pose_list[0]
        pose_msg.position.y = pose_list[1]
        pose_msg.position.z = pose_list[2]
        pose_msg.orientation.x = pose_list[3]
        pose_msg.orientation.y = pose_list[4]
        pose_msg.orientation.z = pose_list[5]
        pose_msg.orientation.w = pose_list[6]
    elif len(pose_list) == 6:
        pose_msg.position.x = pose_list[0]
        pose_msg.position.y = pose_list[1]
        pose_msg.position.z = pose_list[2]
        q = tf.transformations.quaternion_from_euler(
            pose_list[3], pose_list[4], pose_list[5]
        )
        pose_msg.orientation.x = q[0]
        pose_msg.orientation.y = q[1]
        pose_msg.orientation.z = q[2]
        pose_msg.orientation.w = q[3]
    else:
        raise MoveItCommanderException(
            "Expected either 6 or 7 elements in list: (x,y,z,r,p,y) or (x,y,z,qx,qy,qz,qw)"
        )
    return pose_msg


def list_to_pose_stamped(pose_list, target_frame):
    pose_msg = PoseStamped()
    pose_msg.pose = list_to_pose(pose_list)
    pose_msg.header.frame_id = target_frame
    pose_msg.header.stamp = rospy.Time.now()
    return pose_msg


def transform_to_list(trf_msg):
    trf = []
    trf.append(trf_msg.translation.x)
    trf.append(trf_msg.translation.y)
    trf.append(trf_msg.translation.z)
    trf.append(trf_msg.rotation.x)
    trf.append(trf_msg.rotation.y)
    trf.append(trf_msg.rotation.z)
    trf.append(trf_msg.rotation.w)
    return trf


def list_to_transform(trf_list):
    trf_msg = Transform()
    trf_msg.translation.x = trf_list[0]
    trf_msg.translation.y = trf_list[1]
    trf_msg.translation.z = trf_list[2]
    trf_msg.rotation.x = trf_list[3]
    trf_msg.rotation.y = trf_list[4]
    trf_msg.rotation.z = trf_list[5]
    trf_msg.rotation.w = trf_list[6]
    return trf_msg

def pointcloud_to_list(pointcloud):
    '''
    Convert the pointcloud to a list. The format should be [x1, y1, z1, x2, y2, z2, ...]
    '''
    pointcloud_list = []
    for p in pointcloud.points:
        pointcloud_list.append(p.x)
        pointcloud_list.append(p.y)
        pointcloud_list.append(p.z)
    return pointcloud_list

def distribution_to_list(distributions, number_of_joint):
    '''
    In this function, we convert the SamplingDistribution message to a list. The format
    should be [distribution1_mean, distribution1_variance, distribution2_mean, distribution2_variance, ...]
    we assume the length of mean should be 7.
    '''
    distribution_list = []

    for d in distributions:
        if len(d.distribution_mean) != number_of_joint:
            raise("The length of the mean does not match the number of joint of current group!!!")

        for d_value in d.distribution_mean:
            distribution_list.append(d_value)

        for d_value in d.distribution_convariance:
            distribution_list.append(d_value)

        if len(d.distribution_mean) * len(d.distribution_mean) != len(d.distribution_convariance):
            raise("The length of the convariance is not correct based on length of the mean!!!")

        if len(d.related_co_parameter_index) != len(d.related_beta) or len(d.related_co_parameter_index) != len(d.related_similarity):
            raise("The length of related co-parameter index is not equal to the length of related beta or similarity!!!")

        distribution_list.append(d.foliation_id)
        distribution_list.append(d.co_parameter_id)
        distribution_list.append(d.distribution_id)
        distribution_list.append(d.beta_ratio)
        distribution_list.append(len(d.related_co_parameter_index))
        for i in d.related_co_parameter_index:
            distribution_list.append(i)
        for rb in d.related_beta:
            distribution_list.append(rb)
        for rs in d.related_similarity:
            distribution_list.append(rs)

    return distribution_list