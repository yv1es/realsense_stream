
FROM nvidia/cuda:11.7.1-cudnn8-runtime-ubuntu20.04

RUN apt update \
 && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
    sudo \
    curl \ 
    lsb-release \
    vim 

RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - 


RUN apt update \
 && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
    ros-noetic-desktop 

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc 
    

RUN apt update \
 && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
    python3-rosdep  \
    python3-rosinstall \ 
    python3-rosinstall-generator \  
    python3-wstool \
    python3-pip \
    build-essential \ 
    git


RUN sudo rosdep init && rosdep update

RUN mkdir -p ~/catkin_ws/src \ 
    && cd ~/catkin_ws/src \
    && git clone https://github.com/yv1es/realsense_stream.git \
    && . /opt/ros/noetic/setup.sh \
    && cd .. && catkin_make

RUN pip install numpysocket cvbridge3

RUN apt update \
 && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
    ros-noetic-rtabmap-ros