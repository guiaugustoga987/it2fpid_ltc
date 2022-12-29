# it2fpid_ltc

it2fpid_ltc is a transmission power lines tracker/follower with an UAV based on an Intervalar Type-2 fuzzy-PID controller. It allows three control modes :

- **Linear** : The UAV fixes the lateral positioning related to the power lines in hovering mode. 
- **Angular** : The UAV fixes the angular positioning related to the power lines in hovering mode. 
- **Following** : The UAV follow the power lines with a fixed forward velocity fixing lateral and angular positioning.

The project is based on Gazebo simulations with ROS framework. It uses Mavros/PX4 interfaces to control the UAV. Any suggestion/contribution is welcome.

# Installation guide :

This guide assumes you have a clean Ubuntu 20.04 (Focal) release installation on your machine. To check your ubuntu version run ```cat /etc/os-release``` on a terminal. The output should be something like this :

```
NAME="Ubuntu"
VERSION="20.04.5 LTS (Focal Fossa)"
ID=ubuntu
ID_LIKE=debian
PRETTY_NAME="Ubuntu 20.04.5 LTS"
VERSION_ID="20.04"
HOME_URL="https://www.ubuntu.com/"
SUPPORT_URL="https://help.ubuntu.com/"
BUG_REPORT_URL="https://bugs.launchpad.net/ubuntu/"
PRIVACY_POLICY_URL="https://www.ubuntu.com/legal/terms-and-policies/privacy-policy"
VERSION_CODENAME=focal
UBUNTU_CODENAME=focal
```

- Install PX4 firmware, MAVROS and ROS Noetic with Gazebo-11. Follow these steps (Adapted from https://kuat-telegenov.notion.site/How-to-setup-PX-toolchain-development-environment-for-drone-simulations-04adcf4370bf4455b374321f5d1e3bb1) :

**PX4**

```
sudo apt update
sudo apt upgrade
sudo apt install git
mkdir src
cd src
git clone https://github.com/guiaugustoga987/PX4-Autopilot.git --recursive (This is a PX4 with a custom gazebo world containing the Transmission Power Line model and a modified 3DR-Solo with a camera facing downwards for the controller tests)
cd Firmware
bash ./Tools/setup/ubuntu.sh
```
Reboot your computer !!

**ROS Noetic/MAVROS/Gazebo-11 installation :**

```
wget https://raw.githubusercontent.com/ktelegenov/scripts/main/ubuntu_sim_ros_noetic.sh
bash ubuntu_sim_ros_noetic.sh

## close the terminal and open it again
cd src/Firmware
git submodule update --init --recursive
DONT_RUN=1 make px4_sitl_default gazebo

source Tools/simulation/gazebo/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/simulation/gazebo/sitl_gazebo 
```

**Add the transmission power lines model to the ~/.gazebo folder (In case you don't find this folder go to /home/your_user and use ctrl+H to find it):**

```
cd .gazebo
git clone https://github.com/guiaugustoga987/TL.git
```

**Add the custom world with the transmission lines model to the sitl_gazebo submodule :**

```
cd src/Firmware/Tools/sitl_gazebo/worlds
wget https://github.com/guiaugustoga987/sitl_gazebo/blob/main/worlds/iris_tl.world
```

**Install PyIT2FLS (Toolkit for Interval Type 2 Fuzzy Logic Systems):**

```
pip3 install --upgrade pyit2fls
```

**Install the it2fpid_ltc package:**

```
cd ~/catkin_ws/src/
git clone https://github.com/guiaugustoga987/it2fpid_ltc.git
cd ~/catkin_ws/
catkin build
```

# Running the simulation :

**Open PX4 (New terminal)**

```
cd src/Firmware
no_sim=1 make px4_sitl_default gazebo
```

**Launch gazebo with the custom world (New terminal)**

```
roslaunch gazebo_ros empty_world.launch world_name:=$(pwd)/Tools/sitl_gazebo/worlds/iris_tl.world
```
**Open MAVROS (New terminal)**


```
roslaunch mavros sitl_px4.launch
```

**Run the controller node (New terminal)**

```
rosrun it2fpid_ltc f2_linetracking.py
```

The default mode is the lateral tracking controller. To change it, open the ```f2_linetracking.py``` script and on __init__ function uncomment the desired mode :

```
self.TEST_MODE = 'lateral' 
#self.TEST_MODE = 'angular' 
#self.TEST_MODE = 'following' 
```

The simulations variables are stored on .txt files on Results folder. Each run stores 300 samples for lateral/angular modes and 1600 samples for the following mode. The variables to be stored, number of runs and amount of samples can be changed as needed.

