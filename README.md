# Allegro Skeleton Tracking

This project allows the installation of a Docker container for controlling the Allegro Hand V4 by demonstrating grasps in front of a ZED camera.
The Docker container includes the installation of Ubuntu 20.04, ROS Noetic, Openpose, Cuda 11.7 and CudNN.

# For executing the program the following steps have to be followed:
1. Install ZED SDK on host machine (https://www.stereolabs.com/developers/release/) (Version 3.8.2 worked)
	- During ZED SDK installation agree to installation of CUDA, cudNN if not already installed
2. Install Docker Desktop (https://docs.docker.com/desktop/install/linux-install/)
3. Install Nvidia Docker runtime (https://github.com/NVIDIA/nvidia-docker#quickstart)
4. Clone this git repository
5. In a terminal enter root directory of this repository (the one which contains the Dockerfile)
6. Install the docker image by running:
	sudo docker build -t allegro_skeleton_tracking .
7. Wait for installation of docker image to finish and get a coffee (Lasts around 30 minutes)
8. For allowing the docker container to open a Display call:
	xhost +local:docker 
9. Start a Docker container with the following commands including the appropriate tags (change /path/to/allegro_skeleton_tracking to the absolute path of the folder, where you cloned the repository):
	sudo docker run --gpus all -it --runtime=nvidia -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,video --mount type=bind,src=/usr/local/zed,target=/usr/local/zed --mount type=bind,src=/path/to/allegro_skeleton_tracking/ROS_Packages,target=/home/catkin_ws/src --privileged -e  DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix allegro_skeleton_tracking /bin/bash
10. Inside the Docker container run (If the following commands fail, just repeat them until they succeed): 
	catkin_make
	roslaunch allegro_hand_basic simple_launch.launch

# Interesting docker commands:
- sudo docker container prune 		-> removes all stopped containers
- sudo docker images 				-> show all images
- sudo docker rmi image_id 			-> remove image named image_id
- sudo docker container ls --all		-> show all containers, also the stopped ones
- sudo docker start -a container-id		-> enter a stopped container (-a option attaches the terminal output)
- sudo docker exec -it container-id bash	-> open a new bash of an already started docker container

# Trouble-Shooting:
	Docker trouble-shooting:
		When trying to use openpose inside the created docker:
		(OpenPose 1.7.0:32): Gtk-WARNING **: 15:12:43.538: cannot open display: :1
		solution:
			1. Leave the docker (Ctrl d)
			2. Call: xhost +local:docker
			3. Run the container with sudo docker run --gpus all --runtime=nvidia -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix openpose_cwaffles /bin/bash
		When trying to use openpose inside the created docker:
		F0306 15:14:18.876608    52 syncedmem.hpp:39] Check failed: error == cudaSuccess (700 vs. 0)  an illegal memory access was encountered
			solution:
			1. Turn face off:
			./build/examples/openpose/openpose.bin --video examples/media/video.avi -face=false --hand --write_json output_json_folder/
		Cannot see all images/containers / Other unexpected problems with docker:
			solution:
			1. Call commands with sudo privileges
	ros_openpose trouble_shooting:
		- see the provided section on the Website 
		- further trouble-shooting
			- When calling catkin_make:
			error: Could not find a package configuration file provided by "OpenPose" with any of the following names:
    				OpenPoseConfig.cmake
    				openpose-config.cmake
    			Solution:
	    			1. Go to openpose/build folder
	    			2. Run sudo make install
			- When calling catkin_make:
			error: no matching function for call to ‘op::WrapperStructPose::WrapperStructPose(<brace-enclosed initializer list>)’
			solution:
			 	1. go to root directory of openpose
			 	2. call: git checkout tags/v1.7.0
			 	3. go to openpose/build
			 	4. call: sudo make install
			-When calling roslaunch ros_openpose run.launch camera:=zed:
			error while loading shared libraries: libcaffe.so.1.0.0: cannot open shared object file: No such file or directory:
			Solution:
				1. Add the following lines to your .bashrc file (exchange your_username): 
					LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/your_username/openpose/build/caffe/lib
					export LD_LIBRARY_PATH
				2. Start a new terminal
			-When calling roslaunch ros_openpose run.launch camera:=zed:	
			error: [ INFO] [1681991405.601017605]: ZED connection -> CAMERA FAILED TO SETUP
			solution:
				Plug in Zed camera before opening the docker container.
 


