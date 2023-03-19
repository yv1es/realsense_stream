#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header

import socket
from numpysocket import NumpySocket
import numpy as np 
from numpy_ros import to_numpy, to_message

import cv2
from cv_bridge import CvBridge


FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 30

HOST = socket.gethostname() 
PORT = 5000 


def setup_sockets():
    socket_c = NumpySocket()  
    socket_c.bind((HOST, PORT)) 
    socket_c.listen()

    return socket_c




def talker():

    color_pub = rospy.Publisher('/camera/rgb/image_rect_color', Image, queue_size=10)
    depth_pub = rospy.Publisher('/camera/depth_registered/image_raw', Image, queue_size=10)
    info_pub = rospy.Publisher('/camera/rgb/camera_info', CameraInfo, queue_size=10)


    rospy.init_node('camera_node', anonymous=True)
    rate = rospy.Rate(20) # 10hz

    camera_info = CameraInfo()
    camera_info.height = FRAME_HEIGHT
    camera_info.width = FRAME_WIDTH

    rospy.loginfo("Waiting for connection")
    socket = setup_sockets()
    conn, address = socket.accept()

    bridge = CvBridge()



    while not rospy.is_shutdown():

        rgbd_bytes_np = conn.recv()
        
        if np.size(rgbd_bytes_np) == 0:
            break

        
        color_bytes_np, depth_bytes_np = np.split(rgbd_bytes_np, [FRAME_WIDTH*FRAME_HEIGHT*3]) 
        
        color_bytes = color_bytes_np.tobytes()
        depth_bytes = depth_bytes_np.tobytes()

        color_image = np.frombuffer(color_bytes, dtype=np.uint8)
        color_image.shape = (FRAME_HEIGHT, FRAME_WIDTH, 3)
        depth_image = np.frombuffer(depth_bytes, dtype=np.uint16)
        depth_image.shape = (FRAME_HEIGHT, FRAME_WIDTH)
        
        # do whatever with color and depth
        rospy.loginfo("Got images")

        
        color_ros = bridge.cv2_to_imgmsg(color_image, encoding="rgb8")
        depth_ros = bridge.cv2_to_imgmsg(depth_image, encoding="mono16")
        

        current_time = rospy.get_time()
        header = Header(stamp=rospy.Time.from_sec(current_time), frame_id="base_link")

        color_ros.header = header
        depth_ros.header = header
        camera_info.header = header

        color_pub.publish(color_ros)
        depth_pub.publish(depth_ros)
        info_pub.publish(camera_info)
        
        # rate.sleep()

    conn.close() 
    rospy.loginfo("Disconnected")





if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
