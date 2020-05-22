## **ENPM809B: Building a Manufacturing Robot Software System: Final Project**

This is README to run the ROS package created for Final Project. 

Group Members:
1.  An Li
2.  Andre 
3.  Nakul Patel
4.  Revati Naik
5.  Sarvesh Thakur
6.  Srujan Panuganti

## **Package Name: group7_final_project**
* This package builds multiple kits for the Final Project based on the requirement.
* It builds entire kits based on the order.
* It handles the sensor blackout while still continuing to build the kit.
* It has the abilty to detect faulty parts and replace them. Also implements a smart strategy on how to deal with faulty parts if there is a sensor blackout.
* It also checks for the updates to an order.
* It identifies sudden drops of parts during pick and place. Also implements smart strategies to correct the pose of dropped parts even during sensor blackout.
* It takes smart decisions between modifying the current kit or building a whole new kit if that order is updated.
* It implements smart strategies to preserve the belt parts while updating an order.
* It takes the help of other arm to flip the parts if required, as described in the order.
* The robotic arms help each other if one of them is not able to reach a part while building a kit.
* It can build multiple orders with at most two shipments each.
* It assumes multiple waves of conveyor belt parts (at least 2 waves).

## **Minimum System Requirements**
  * ROS Melodic 
  * Gazebo >= 9.6.0 
  * Ariac 2019
  * Ubuntu Desktop 18.04 Bionic (64-bit)
  * moveit
  * gazebo_ros_pkgs 

## gazebo_ros_pkgs
gazebo_ros_pkgs package should be available in the workspace to run group7_final_project. Use the following command to 
clone the package and get it working

`cd ~/catkin_ws/src`

`git clone https://github.com/osrf/ariac-gazebo_ros_pkgs -b ariac-network-melodic`

`cd ~/catkin_ws`

`catkin_make --only-pkg-with-deps gazebo_ros_pkgs`

Read more at https://bitbucket.org/osrf/ariac/wiki/2019/tutorials/installation

## moveit_visual_tools
You should also have moveit_visual_tools installed if not installed previously. moveit_visual_tools is not installed by default using installation instructions on ariac 2019 website. Use the below command to install moveit_visual_tools for ros-melodic

  `sudo apt-get install ros-melodic-moveit-visual-tools`


## **Instructions to RUN the package group7_final_project**
1. Create and build a catkin workspace

      `mkdir -p ~/catkin_ws/src`
       
      `cd ~/catkin_ws/`

2.  Extract the package to `~/catkin_ws/src/` as below

      `~/catkin_ws/src/group7_final_project`
         
3. Source the setup.bash to add environment variables to your path to allow ROS to function

      `source /opt/ros/melodic/setup.bash`
      `source ~/catkin_ws/devel/setup.bash`

4. Build your catkin workspace

      `cd ~/catkin_ws`

      `catkin_make`

    You can alternatively use following command to build only group7_final_project package.
  
    `catkin_make --only-pkg-with-deps group7_final_project`

  Note: Always call `catkin_make` in the root of your catkin workspace. 

5. Overlay your catkin workspace on top of your environment.

     `cd ~/catkin_ws`

     `source devel/setup.bash`
 
## **Launch the Package**

To run the environment and the node, open another terminal and run the following command
Ensure that you are inside your workspace directory

In terminal 1:

	`cd ~/catkin_ws`

	`source devel/setup.bash`

	`roslaunch group7_final_project group7_final_project.launch`


In terminal 2 (to use moveit interface for arm1):

	`cd ~/catkin_ws`

	`source install/setup.bash`

	`roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm1`

In terminal 3 (to use moveit interface for arm2):

	`cd ~/catkin_ws`

	`source install/setup.bash`

	`roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm2`

In terminal 4:

	`cd ~/catkin_ws`
	 
	`source devel/setup.bash`

	`rosrun group7_final_project main_node`

## **See how it works**

<p align="center">
<img src="gifs/1.gif" alt="Pick and Place">
</p>
<p align="center">
<em>Pick and Place</em>
</p>

<p align="center">
<img src="gifs/2.gif" alt="Faulty Check">
</p>
<p align="center">
<em>Faulty Check</em>
</p>

<p align="center">
<img src="gifs/quality_check_faulty.gif" alt="Faulty Check">
</p>
<p align="center">
<em>Faulty Check During Sensor Blackout</em>
</p>

<p align="center">
<img src="gifs/3.gif" alt="In Process Order Update">
</p>
<p align="center">
<em>In Process Order Update</em>
</p>

## **Our Final Output**

<p align="center">
<img src="gifs//kit_building.gif" alt="kit Building Output">
</p>
<p align="center">
<em>Kit Building Output</em>
</p>

Access the below link to see multiple vidos of how our pipeline is working!

https://drive.google.com/drive/folders/1DF2pgMkngMvCEMyj_yyr3MpZdI1tnsCo
