# ROS 2 Programming Assignment 4

### Dependencies
This project makes use of the ROS Humble Hawksbill distribution and is assumed to be a dependency. <br>
Find installation instructions [here](https://docs.ros.org/en/humble/Installation.html)

### Building the Code

```bash
$ source /opt/ros/humble/setup.bash
# Make your ros2 workspace
$ mkdir -p ~/ros_ws/src
# Go to the source directory of your ros2 workspace
$ cd ~/ros_ws/src
#Clone the repository
$ git clone https://github.com/Uthappa13/my_gazebo_tutorials.git
#Go back to the ws directory
$ cd ~/ros_ws
# Install rosdep dependencies before building the package
$ rosdep install -i --from-path src --rosdistro humble -y
# Build the package using colcon build
$ colcon build --packages-select walker
# After successfull build source the package
$ source ./install/setup.bash
# Run the launch file in terminal
$ ros2 launch walker walker_world.launch.py
```


### ROS2 Bag Functionality
The launch file has been modified to support ros2 bag recording. To record use the `ros2_bag_start` parameter (True/False).

```bash
$ source /opt/ros/humble/setup.bash
$ cd ~/ros_ws
$ source ./install/setup.bash
# Run the launch file in terminal with the ros2_bag_start parameter as true
$ ros2 launch walker walker_world.launch.py ros2_bag_start:=True
```

To inspect and playback the ros2 bag.
```bash
$ source /opt/ros/humble/setup.bash
$ cd ~/ros_ws
$ source ./install/setup.bash
# Inspect the ros2 bag
$  ros2 bag info walkerbag
# Play back the contents of the ros2 bag
$  ros2 bag play walkerbag
```

### Check style guidelines
```bash
#In the package directory
cd ~/ros_ws/src/walker

# cpplint
$ cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order  src/*.cpp >  results/cpplint.txt
```