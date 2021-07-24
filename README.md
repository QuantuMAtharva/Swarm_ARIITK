# Swarm_ARIITK
Obstacle Avoidance using Swarm Robotics

# Basic Setup
## Setting up rotors_simulator via terminal
<br>

```
sudo apt-get install ros-melodic-desktop-full ros-melodic-joy ros-melodic-octomap-ros ros-melodic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-melodic-control-toolbox ros-melodic-mavros ros-melodic-geographic-*
```
<br>


```
sudo apt-get install python-pip
```
<br>

```
pip install --upgrade pip
```
<br>

```
pip install future
```
<br>

```
sudo apt-get install python-future
```
<br>

```
cd ~
```
<br>

```
mkdir -p ros/rotors_ws/src
```
<br>

```
cd ros/rotors_ws/
```
<br>

```
catkin init
```
<br>

```
cd src
```
<br>

```
wstool init
```
<br>

```
wget https://raw.githubusercontent.com/ethz-asl/rotors_simulator/master/rotors_hil.rosinstall
```
<br>

```
wstool merge rotors_hil.rosinstall
```
<br>

```
wstool update
```
<br>

```
git clone https://github.com/ethz-asl/rotors_simulator.git
```
<br>

```
catkin build -j4
```
<br>

## Add the following line to your ~/.bashrc file

```
source ~/ros/rotors_ws/devel/setup.bash
```
Comment out the lines for PX-4 Autopilot from the .bashrc file if you have it installed

