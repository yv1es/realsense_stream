#!/usr/bin/env python3

import rospy
import socket
import numpy as np 
from numpysocket import NumpySocket
from cv_bridge import CvBridge

# import ROS messages 
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header

# Constants
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 30
HOST = socket.gethostname() 
PORT = 5000 

def setupSocket():
    socket = NumpySocket()  
    socket.bind((HOST, PORT)) 
    socket.listen()
    return socket


def setupCameraInfo():
    # information on parameters. http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
    camera_info = CameraInfo()
    camera_info.width = FRAME_WIDTH
    camera_info.height = FRAME_HEIGHT
    camera_info.distortion_model = "plumb_bob"

    camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]

    camera_info.K = [618.9998779296875,   0.0,             320.2127685546875,
                       0.0,             618.9998779296875, 239.30780029296875,
                       0.0,               0.0,               1.0] 
    
    camera_info.R = [1.0, 0.0, 0.0, 
                     0.0, 1.0, 0.0, 
                     0.0, 0.0, 1.0]

    camera_info.P = [618.9998779296875,   0.0,             320.2127685546875,   0.0,
                       0.0,             618.9998779296875, 239.30780029296875,  0.0,
                       0.0,               0.0,               1.0,               0.0]
    
    return camera_info


def decode(rgbd_bytes_np):
    color_bytes_np, depth_bytes_np = np.split(rgbd_bytes_np, [FRAME_WIDTH*FRAME_HEIGHT*3]) 
        
    color_bytes = color_bytes_np.tobytes()
    depth_bytes = depth_bytes_np.tobytes()

    color_image = np.frombuffer(color_bytes, dtype=np.uint8)
    color_image.shape = (FRAME_HEIGHT, FRAME_WIDTH, 3)
    depth_image = np.frombuffer(depth_bytes, dtype=np.uint16)
    depth_image.shape = (FRAME_HEIGHT, FRAME_WIDTH)
    return color_image, depth_image


def main():
    # initialize node and topics 
    rospy.init_node('camera_node', anonymous=True)
    color_pub = rospy.Publisher('/camera/rgb/image_rect_color', Image, queue_size=10)
    depth_pub = rospy.Publisher('/camera/depth_registered/image_raw', Image, queue_size=10)
    info_pub = rospy.Publisher('/camera/rgb/camera_info', CameraInfo, queue_size=10)

    # create camera_info and CvBridge
    camera_info = setupCameraInfo()
    bridge = CvBridge()

    rospy.loginfo("Waiting for streamer connection")
    socket = setupSocket()
    conn, address = socket.accept()
    rospy.loginfo("Streamer connected")

    # publisher loop 
    while not rospy.is_shutdown():
        # recieve frames
        rgbd_bytes_np = conn.recv()
        # quit when null 
        if np.size(rgbd_bytes_np) == 0:
            break
        # decode into color and depth images
        color_image, depth_image = decode(rgbd_bytes_np)
        
        # transform to ROS Image messages
        color_ros = bridge.cv2_to_imgmsg(color_image, encoding="rgb8")
        depth_ros = bridge.cv2_to_imgmsg(depth_image, encoding="mono16")
        
        # set headers 
        current_time = rospy.get_time()
        header = Header(stamp=rospy.Time.from_sec(current_time), frame_id="camera_link")
        color_ros.header = header
        depth_ros.header = header
        camera_info.header = header

        # publish 
        color_pub.publish(color_ros)
        depth_pub.publish(depth_ros)
        info_pub.publish(camera_info)
        
    conn.close() 
    rospy.loginfo("Streamer disconnected")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass