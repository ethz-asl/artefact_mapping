# artefact_mapping
Detection, tracking, and mapping of object artefacts

## Install
### ROS and system dependencies
```bash
sudo apt install software-properties-common
sudo add-apt-repository "deb http://packages.ros.org/ros/ubuntu noetic main"
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full "ros-noetic-tf2-*" "ros-noetic-camera-info-manager*" --yes

sudo rosdep init
rosdep update
echo ". /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install autotools-dev doxygen dh-autoreconf git git-lfs liblapack-dev libblas-dev libgtest-dev \
libreadline-dev libssh2-1-dev clang-format-6.0 python3-autopep8 python3-catkin-tools python3-pip python3-git \
python-setuptools python3-termcolor python3-wstool libatlas3-base python-is-python3 --yes

pip install -U requests

sudo apt install -y ccache &&\
echo 'export PATH="/usr/lib/ccache:$PATH"' | tee -a ~/.bashrc &&\
source ~/.bashrc && echo $PATH
ccache --max-size=10G
```

### Setup catkin workspace
```bash
mkdir -p artefact_mapping_ws/src
cd artefact_mapping_ws
catkin init
catkin config --merge-devel # Necessary for catkin_tools >= 0.4.
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS=-fdiagnostics-color
cd src
```

### Clone and build
```bash
git clone https://github.com/ethz-asl/artefact_mapping.git --recursive -b summer_school2021
catkin build artefact_mapping
```

## Running the node
### Source the workspace
```bash
source ~/artefact_mapping_ws/devel/setup.bash
```

### Start mapping artefacts
Adapt the smb name in the launch file below to the correct SMB number and run to start the object mapping node.
```
roslaunch artefact_mapping artefact_mapping.launch
```
Detections will be published on the `W_landmark` topic in the odometry frame. Other parameters such as the ones listed below can also be added and adapted in the launch file.

| Flag | Default | Description |
| --- | --- | --- |
| object_tracker_image_topic | /camera/color/image_raw | ROS image topic on which to perform detection and tracking |
| image_topic_buffer_size | 200 | Buffer size of ROS topic listener for incoming images |
| sensor_calibration_file | share/camchain.yaml | Camera calibration file which is used for retrieving the intrinsics |
| object_tracker_detection_period | 20 | Periodicity with which to run object detection (yolo). Frames in between will be tracked using a faster method to obtain object locations (kcf). With the default value the detector is run every 20th frame. |
| darknet_cfg_path | share/yolov3.cfg | Path to network configuration file (the tiny version will run on a CPU but will produce less good and accurate detections). See the [share](https://github.com/ethz-asl/artefact_mapping/tree/summer_school2021/artefact_mapping/share) folder for options |
| darknet_weights_path | share/yolov3.weights | Path to network weights, must match the cfg file. See the [share](https://github.com/ethz-asl/artefact_mapping/tree/summer_school2021/artefact_mapping/share) folder for options |
| darknet_classes | 0 | Comma separated list without spaces to define the tracked object classes. The association between object name and number can be found [here](https://github.com/ethz-asl/darknet_catkin/blob/master/data/coco.names) (the numbering starts from 0) |
| darknet_detection_threshold | 0.4 | Detection confidence threshold at which to start tracking an object |
| darknet_nms_threshold | 0.45 | Non maxima supression threshold, used in the elimination of duplicate detections |
| tracker_confidence_threshold | 0.8 | Confidence threshold at which the tracker still considers it is following a valid object, if the confidence drops below the threshold the track is terminated and the object pose triangulated |
| track_reassociation_iou | 0.3 | If a new detection and an existing track have an intersection over union (IoU) overlap above this threshold they are merged instead of creating a new separate track |
| object_tracker_pose_buffer_length | 600 | TF buffer length in seconds, in other words the time by which the images have to be processed or else they will no longer have a valid odometry transform associated |
| sensor_tf_frame | /blackfly_right_optical_link | Camera TF frame for triangulation |
| odom_tf_frame | /odom | Odometry TF frame for triangulation |
| publish_debug_images | false | Publish a debug topic `/artefact_mapping/debug_image` showing the detections and their tracking in real time |
| v | 0 | Verbosity output of node, increasing to 1 or 2 will give more debug messages and information on what is being detected, tracked and published |
