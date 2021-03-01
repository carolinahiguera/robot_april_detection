#!/usr/bin/env python  

import rospy
import tf2_ros
import copy
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler


if __name__ == '__main__':
    rospy.init_node('tf2_robot_footprint')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)


    broadcaster = tf2_ros.StaticTransformBroadcaster()
    new_tf = geometry_msgs.msg.TransformStamped()
    new_tf2 = geometry_msgs.msg.TransformStamped()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('world', 'apriltag_robot', rospy.Time())

            new_tf.header.stamp = rospy.Time.now()
            new_tf.header.frame_id = 'world'
            new_tf.child_frame_id = 'world_to_base_footprint'

            new_tf.transform.translation.x = trans.transform.translation.x
            new_tf.transform.translation.y = trans.transform.translation.y
            new_tf.transform.translation.z = 0.0

            q = [trans.transform.rotation.x, trans.transform.rotation.y, 
                trans.transform.rotation.z, trans.transform.rotation.w]
            (roll, pitch, yaw) = euler_from_quaternion (q)
            roll = 0.0
            pitch = 0.0
            quat = quaternion_from_euler(roll, pitch, yaw)

            new_tf.transform.rotation.x = quat[0]
            new_tf.transform.rotation.y = quat[1]
            new_tf.transform.rotation.z = quat[2]
            new_tf.transform.rotation.w = quat[3]

            broadcaster.sendTransform(new_tf)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue

        try:
            trans2 = tfBuffer.lookup_transform('apriltag_robot', 'world_to_base_footprint', rospy.Time())
            
            new_tf2.header.stamp = rospy.Time.now()
            new_tf2.header.frame_id = 'apriltag_robot'
            new_tf2.child_frame_id = 'robot_base_footprint'

            new_tf2.transform.translation.x = trans2.transform.translation.x
            new_tf2.transform.translation.y = trans2.transform.translation.y
            new_tf2.transform.translation.z = trans2.transform.translation.z

            q = [trans2.transform.rotation.x, trans2.transform.rotation.y, 
                trans2.transform.rotation.z, trans2.transform.rotation.w]
            (roll, pitch, yaw) = euler_from_quaternion (q)
            quat = quaternion_from_euler(roll, pitch, yaw)

            new_tf2.transform.rotation.x = quat[0]
            new_tf2.transform.rotation.y = quat[1]
            new_tf2.transform.rotation.z = quat[2]
            new_tf2.transform.rotation.w = quat[3]

            broadcaster.sendTransform(new_tf2)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
