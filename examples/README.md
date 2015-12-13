# Celeros Examples
Basic examples for Celeros

## Installation

### From Source

First, install [ROS Indigo release](http://wiki.ros.org/indigo/Installation/Ubuntu).

Then, make sure the ROS_DISTRO environment variable is set correctly:
```
echo $ROS_DISTRO
```

It may already be.  If not, issue this shell command:
```
$ export ROS_DISTRO=indigo
```

Then, clone the source repositories:

```
$ source /opt/ros/$ROS_DISTRO/setup.bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ wstool init src https://raw.githubusercontent.com/asmodehn/celeros/indigo-devel/examples/rosinstall/$ROS_DISTRO.rosinstall
```

Install all dependencies:
```
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y
```

Then, build everything:
```
$ catkin_make
$ source devel/setup.bash
```

This is still work in progress, so more details will be documented here when this gets finalized.