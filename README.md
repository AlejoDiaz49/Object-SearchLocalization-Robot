# Object-SearchLocalization-Robot
> ROS Simulation of a mobile robot that is capabell of recognized and localize object in the enviroment

[![Demo](https://img.youtube.com/vi/o789586vQO0/0.jpg)](https://youtu.be/o789586vQO0)

## Installation and Building

Required packages:

```sh
sudo apt-get install ros-kinetic-turtlebot
                     ros-kinetic-turtlebot-apps
                     ros-kinetic-turtlebot-interactions 
                     ros-kinetic-turtlebot-simulator 
                     ros-kinetic-kobuki-ftdi
                     ros-kinetic-ar-track-alvar-msgs
                     ros-kinetic-aruco-ros
```

Include in the .bashrc file the next line so that Gazebo will be able to find the models of the different objects

```sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/Object-SearchLocalization-Robot/labrob_gazebo/models
```

The darknet_ros package has to be build in Release mode, the rest of the packagues work as usual

```sh
catkin_make
catkin_make -DCMAKE_BUILD_TYPE=Release --pkg darknet_ros
```

## Usage example

The Simulation can be launch in different ways

### Manual control

### Automatic control

## Development setup

## Release History
