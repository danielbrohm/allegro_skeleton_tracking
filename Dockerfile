# https://hub.docker.com/r/cwaffles/openpose
FROM nvidia/cuda:11.7.0-cudnn8-devel-ubuntu20.04

#get deps
RUN apt-get update && \
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
python3-dev python3-pip python3-setuptools git g++ wget make libprotobuf-dev protobuf-compiler libopencv-dev \
libgoogle-glog-dev libboost-all-dev libhdf5-dev libatlas-base-dev libcanberra-gtk-module libcanberra-gtk3-module lsb-release 

#for python api
RUN pip3 install --upgrade pip
RUN pip3 install numpy opencv-python 

#replace cmake as old version has CUDA variable bugs
RUN wget https://github.com/Kitware/CMake/releases/download/v3.16.0/cmake-3.16.0-Linux-x86_64.tar.gz && \
tar xzf cmake-3.16.0-Linux-x86_64.tar.gz -C /opt && \
rm cmake-3.16.0-Linux-x86_64.tar.gz
ENV PATH="/opt/cmake-3.16.0-Linux-x86_64/bin:${PATH}"



#RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y sudo

#install ros
RUN DEBIAN_FRONTEND=noninteractive sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
apt-get install -y curl && \
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - &&  \
apt-get update && \
DEBIAN_FRONTEND=noninteractive apt-get install -y ros-noetic-desktop-full

#install needed ros-packages
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential git ros-noetic-cv-bridge ros-noetic-image-transport \
    libturbojpeg ros-noetic-libpcan \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends libturbojpeg

RUN rosdep init && rosdep update

# Create catkin workspace
RUN mkdir -p /catkin_ws/src

# source the ROS setup file and the catkin workspace setup file
WORKDIR /home/catkin_ws/src
RUN /bin/bash -c "cd .. && \
                  source /opt/ros/noetic/setup.bash && \
                  catkin_make && \
                  source devel/setup.bash"
                  
# Automatically source setup files and add libraries to LD_LIBRARY_PATH to solve errors                                   
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /home/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN echo "LD_LIBRARY_PATH=$LD_LIBRARY_PATH:home/openpose/build/caffe/lib:/opt/ros/noetic/lib" >> ~/.bashrc

#get openpose
WORKDIR /home/openpose
RUN git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose.git .

# install openpose using cmake
WORKDIR /home/openpose
RUN mkdir build
WORKDIR /home/openpose/build
RUN cmake -DPYBIND11_INSTALL=ON -DUSE_PYTHON_INCLUDE_DIR=ON -DGPU_MODE=CUDA -DUSE_CUDNN=OFF -DBUILD_PYTHON=ON -DBUILD_CAFFE=ON -DPYTHON_EXECUTABLE=/usr/bin/python3 ..
RUN make -j`nproc`
WORKDIR /home/openpose
RUN git checkout tags/v1.7.0
WORKDIR /home/openpose/build
RUN make install

WORKDIR /home/catkin_ws


