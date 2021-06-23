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

sudo apt install autotools-dev doxygen dh-autoreconf git git-lfs liblapack-dev libblas-dev libgtest-dev libreadline-dev libssh2-1-dev clang-format-6.0 python3-autopep8 python3-catkin-tools python3-pip python3-git python-setuptools python3-termcolor python3-wstool libatlas3-base --yes

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
