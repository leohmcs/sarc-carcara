#!/usr/bin/env python

import rospy
import image_geometry
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped, PointStamped
import tf

import fire_detector

import numpy as np

class FireDetectorNode:
    def __init__(self) -> None:
        self.tf_listener = tf.TransformListener()
        self.camera_model = image_geometry.PinholeCameraModel()
        self.is_model_set = False

        self.fire_detector = fire_detector.FireDetector()
        
        camera_info_sub = rospy.Subscriber('bluefox_optflow/camera_info', CameraInfo, callback=self.camera_info_cb)
        image_sub = rospy.Subscriber('bluefox_optflow/image_raw', Image, self.image_cb)

        self.fire_pos_pub = rospy.Publisher('fire_position', PointStamped, queue_size=10) 

    def camera_info_cb(self, msg: CameraInfo):
        # set camera model parameters if not set
        if not self.is_model_set:
            self.camera_model.fromCameraInfo(msg)
            self.is_model_set = True

    def image_cb(self, msg: Image):
        if not self.is_model_set:
            rospy.loginfo('Waiting for camera info.')
        else:
            img = np.reshape(msg.data, (msg.height, msg.width))
            fire_center = self.fire_detector.locate_fire(img)
            if fire_center is None:
                return # no fire detected

            rect = self.camera_model.rectifyPoint(fire_center)
            p_c = self.camera_model.projectPixelTo3dRay(rect) # point for z = 1 in the camera frame
            fire_pos_world = self.point_in_world(self)
            print(fire_pos_world)
            self.fire_pos_pub.publish(self.point_msg(fire_pos_world))
    
    def point_in_world(self, p_c):
        pose_msg = self.pose_msg(p_c)
        
        ray_world = self.pose_msg_to_array(self.tf_listener.transformPose('world', pose_msg))
        
        (camera_pos, _) = self.tf_listener.lookupTransform('world', self.camera_model.tfFrame)
        camera_pos = np.array(camera_pos)

        p = ray_world - camera_pos
        theta = np.arcsin(np.sqrt(p[0]**2 + p[1]**2)/np.linalg.norm(p))

        p_fire_length = p[2]/np.cos(theta)

        p_fire = (p_fire_length/np.linalg.norm(p_c))*p_c
        return p_fire + camera_pos

    def set_model(self):
        self.camera_model.fromCameraInfo(self.camera_left_info, self.camera_right_info)

    def point_msg(self, p):
        msg = PointStamped()
        msg.header.frame_id = 'world'
        msg.header.stamp = rospy.Time.now()
        msg.point.x = p[0]
        msg.point.y = p[1]
        msg.point.z = p[2]
        return msg

    def pose_msg(self, p):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self.camera_model.tfFrame()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = p[0]
        pose_msg.pose.position.y = p[1]
        pose_msg.pose.position.z = p[2]
        pose_msg.pose.orientation.x = 0
        pose_msg.pose.orientation.y = 0
        pose_msg.pose.orientation.z = 0
        pose_msg.pose.orientation.w = 0
        return pose_msg
    
    def pose_msg_to_array(self, msg: PoseStamped):
        pos = msg.pose.position
        return np.array([pos.x, pos.y, pos.z])

rospy.init_node('fire_detector_node')
node = FireDetectorNode()

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    rate.sleep()
