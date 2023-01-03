# PDE4430 CW2 Gazebo Assessment World

This repository contains the launch files for Gazebo, along with some model files and nodes for the assessment.

**Prerequisite:** Make sure you have the Turtlebot3 setup done correctly.

---

## Installing the package
Follow these steps to setup the package in your ROS workspace:
1. Clone this repository in the `src` directory of your catkin workspace.
2. Recompile your workspace by going to the root directory and running `catkin_make`.
3. Restart all your terminal windows just to ensure that autocomplete works.

## Running the package
Once this is done, you should be able to open the world in Gazebo, using the following commands:
1. `roslaunch assessment_world assessment_world.launch` - This will load the World in Gazebo, which looks like the TurtleBot3 World.
2. `roslaunch assessment_world objects.launch` - This will add the spheres and wall to the World. 

### Running the package without the GUI
To run it without the Gazebo GUI, use the original command, but add the parameter `gui:=false`

`roslaunch assessment_world assessment_world.launch gui:=false`

You will also need to run the command to add the other objects, but you do not need to change that command a gazebo has already been launched without its gui

### Running the package with the TurtleBot 3
If you want to run the simulation with a Turtlebot3, to see what a robot looks like in the simulation and to see how you might control it, you can use the following command:

`roslaunch turtlebot3_gazebo turtlebot3_world.launch`

Once the World is launched with the TurtleBot spawned, run the objects launch file to add the spheres and walls:

`roslaunch assessment_world objects.launch`

---