# Swarm_ARIITK
Obstacle Avoidance using Swarm Robotics

<br>

# Basic Setup
## Setting up rotors_simulator via terminal
<br>

``sudo apt-get install ros-melodic-desktop-full ros-melodic-joy ros-melodic-octomap-ros ros-melodic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-melodic-control-toolbox ros-melodic-mavros ros-melodic-geographic-*``
<br>
<br>
``sudo apt-get install python-pip``
<br>
<br>
``pip install --upgrade pip``
<br>
<br>
``pip install future``
<br>
<br>
``sudo apt-get install python-future``
<br>
<br>
``cd ~``
<br>
<br>
``mkdir -p ros/rotors_ws/src ``
<br>
<br>
``cd ros/rotors_ws/``
<br>
<br>
``catkin init``
<br>
<br>
``cd src``
<br>
<br>
``wstool init``
<br>
<br>
``wget https://raw.githubusercontent.com/ethz-asl/rotors_simulator/master/rotors_hil.rosinstall``
<br>
<br>
``wstool merge rotors_hil.rosinstall``
<br>
<br>
``wstool update``
<br>
<br>
``git clone https://github.com/ethz-asl/rotors_simulator.git``
<br>
<br>
``catkin build -j4``
<br>
<br>
### Add the following line to your ~/.bashrc file

``source ~/ros/rotors_ws/devel/setup.bash``
<br>
<br>
Comment out the lines for PX-4 Autopilot from the .bashrc file if you have it installed

