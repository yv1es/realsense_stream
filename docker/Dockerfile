
FROM nvidia/cuda:11.7.1-cudnn8-runtime-ubuntu20.04

# install basic tools
RUN apt update \
 && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
    sudo \
    curl \ 
    lsb-release \
    vim \
    git \ 
    wget \
    software-properties-common


# install ROS noetic
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - 

RUN apt update \
 && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
    ros-noetic-desktop   

RUN apt update \
 && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
    python3-rosdep  \
    python3-rosinstall \ 
    python3-rosinstall-generator \  
    python3-wstool \
    python3-pip \
    build-essential 

RUN sudo rosdep init && rosdep update

# setup catkin workspace 
RUN mkdir -p ~/catkin_ws/src \ 
    && cd ~/catkin_ws/src \
    && . /opt/ros/noetic/setup.sh \
    && cd ~/catkin_ws \
    && catkin_make

# install rtabmap ros 
RUN apt update \
 && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
    ros-noetic-rtabmap-ros


# install realsense stream packet
RUN pip install numpysocket cvbridge3
RUN cd ~/catkin_ws/src \
    && git clone https://github.com/yv1es/realsense_stream.git \
    && . /opt/ros/noetic/setup.sh \
    && cd ~/catkin_ws \
    && catkin_make

# add source commands to bashrc 
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc 
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc





