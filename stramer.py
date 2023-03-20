import pyrealsense2 as rs
import socket
from numpysocket import NumpySocket
import numpy as np 

# Constants
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 30
HOST = socket.gethostname() 
PORT = 5000 


def setupSocket():
    socket = NumpySocket() 
    socket.connect((HOST, PORT))  
    return socket


def setup_realsense():
    pipeline = rs.pipeline()
    config = rs.config()
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.depth, FRAME_WIDTH, FRAME_HEIGHT,  rs.format.z16, FPS)
    config.enable_stream(rs.stream.color, FRAME_WIDTH, FRAME_HEIGHT, rs.format.bgr8, FPS)

    profile = pipeline.start(config)
    return pipeline
 
def encode(color_image, depth_image):
    color_bytes = color_image.tobytes()
    depth_bytes = depth_image.tobytes()

    color_bytes_np = np.frombuffer(color_bytes, dtype=np.uint8)
    depth_bytes_np = np.frombuffer(depth_bytes, dtype=np.uint8)

    rgbd_bytes_np = np.concatenate([color_bytes_np, depth_bytes_np], dtype=np.uint8)


def main():
    print("Setting up RealSense")
    pipeline = setup_realsense()

    socket = setupSocket()
    print("Connected to publisher")
    
    # setup aligner
    align_to = rs.stream.color
    aligner = rs.align(align_to)

    try:
        while True:
            frames = pipeline.wait_for_frames()

            # Align the depth frame to color frame
            aligned_frames = aligner.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue
            
            # encode frames into one numpy array 
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            rgbd_bytes_np = encode(color_image, depth_image)
            
            # send numpy array 
            socket.sendall(rgbd_bytes_np)
    except:
        pass
    finally:
        socket.close()
        pipeline.stop()
        print("Disconnected")


if __name__=='__main__':
    main()