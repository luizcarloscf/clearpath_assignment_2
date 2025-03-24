# Assignment 1

This repository includes a solution for the second assignment of a course presented in the [Electrical Engineering] department at the [UFES], by [PhD Ricardo Carminati de Mello] using ROS. 

> The Robot Operating System (ROS) is a set of software libraries and tools for building robot applications. From drivers and state-of-the-art algorithms to powerful developer tools, ROS has the open source tools you need for your next robotics project. 

This assigment consists of using a mobile robot with a set of sensors in a partially known environment to:

1. find a person;
2. stand next to the person;

To do so, we developed to following ROS2 packages:
* [turtlebot3_assignment_2](./turtlebot3_assignment_1/): Meta package responsible for packing the entire solution;
* [person_orientation](./person_orientation/): Package the contains a node  responsible to find a person and set the robot' final position.


## Installation

First, install the required ROS 2 packages:
```bash
sudo apt install ros-<ros2-distro>-clearpath*
sudo apt install ros-<ros2-distro>-moveit
```

Create a workspace and install `clearpath_robot` and our package from source:

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/clearpathrobotics/clearpath_robot.git
git clone https://github.com/luizcarloscf/clearpath_assignment_2.git
cd clearpath_robot/
git checkout humble
cd ~/ros2_ws/
colcon build
```

## Running 

Execute the following command to launch the simulation:
```bash
cd ~/ros2_ws/
source ./install/setup.bash
ros2 launch clearpath_gz simulation.launch.py setup_path:=$HOME/ros2_ws/src/clearpath_assignment_2/clearpath/ x:=1.5 y:=2.7 yaw:=1.570
```

Launch the navigation stack,
```bash
cd ~/ros2_ws/
source ./install/setup.bash
ros2 launch clearpath_assignment_2 nav2.launch.py setup_path:=$HOME/ros2_ws/src/clearpath_assignment_2/clearpath/ namespace:=a200_0000 use_sim_time:=true
```

Launch simultaneous localization and mapping,
```bash
cd ~/ros2_ws/
source ./install/setup.bash
ros2 launch clearpath_assignment_2 slam.launch.py setup_path:=$HOME/ros2_ws/src/clearpath_assignment_2/clearpath/ namespace:=a200_0000 use_sim_time:=true
```

Launch rviz for vizualization,
```bash
cd ~/ros2_ws/
source ./install/setup.bash
ros2 launch clearpath_viz view_navigation.launch.py namespace:=a200_0000
```

Than, launch the exploration,
```bash
cd ~/ros2_ws/
source ./install/setup.bash
ros2 launch clearpath_assignment_2 explore.launch.py namespace:=a200_0000
```

Than, to find a person,
```bash
cd ~/ros2_ws/
source ./install/setup.bash
ros2 run person_orientation person_orientation_node
```

person_orientation_node

## Notes

* Ensure that you replace `<ros2-distro>` with your installed ROS 2 distribution (e.g., `humble`, `foxy`).
* If you encounter any dependency issues, verify that all required packages are installed and sourced properly.

# Troubleshooting

* If the build fails, ensure that ROS 2 is correctly installed and sourced.

For additional support, refer to the official [ROS 2 documentation](https://docs.ros.org/en/humble/index.html).


[Electrical Engineering]: https://ele.ufes.br/
[UFES]: https://www.ufes.br/
[ROS]: https://docs.ros.org/en/humble/index.html
[PhD Ricardo Carminati de Mello]: http://lattes.cnpq.br/1569638571582691