
# Install
Requires CUDA 11.7.1 and CuDNN 8

```
git clone https://github.com/yv1es/realsense_stream.git
cd realsense_stream/docker
docker build -t ros-noetic-gpu-img .
docker compose up
```


## Publisher.py
This script should be placed in the /src of a your ROS package. 

## Stramer.py
Run this script after the ROS node is running to start publishing Realsense frames to rtab_map

## Docker setup
### run new container 
`docker run -it --name <conatiner_name> -e DISPLAY=host.docker.internal:0.0 -p 5000:5000 <img-id>` 
By default port 5000 is used for streaming. TODO: build with GPU support
Use XLaunch Server for GUI support (https://sourceforge.net/projects/xming).

Then in the conainer install stuff setup catkin_ws and clone packet to /src. 

