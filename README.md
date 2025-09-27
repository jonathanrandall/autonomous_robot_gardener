# autonomous_robot_gardener
this is a repo for my autonomous robot gardener
coming soon...

# Bill of Materials (BOM)

This project has separate BOMs for mechanical and electronic components:

- [Chassis BOM](docs/chassis_bom.md)  
- [Electronics BOM](docs/electronics_bom.md)

# Repo Structure overview

### docs directory:
This directory contains:
1. __dot_vscode__, which has the configuration for my vscode mostly needed for include paths
2. __photos__, pictures for the repo
3. __robot_docker_files__, these are the files for my docker container build and configuration. Its recommended to use a docker container because if you upgrade your version of ROS, you don't need to upgrade your operating system. And if you pollute your environment by installing multiple versions of the same package, then you can just rebuild your container
4. __trouble_shooting_and_tips__, troulbe shooting with the gamepad (need to add input group to get ros to see the gamepad), ros bridge messages summary frrom gazebo, ros2 cli cheat sheet,esp32_mapping.txt which shows how to map an esp32 to a static port, twist_mux.yaml which is my updated file from articubot one, xarm_measurements.jpg is the mesurements for the xarm which can be used for robot kinematics if needed
5. __chassis_bom.md__, chassis bill of materials
6. __electronics_bom.md__, electronics bill of materials

### esp32_controls_diffbot_serial directory
This directory contains the code for the esp32 that drives the motors. This code interfaces with the hardware interface and uses a pid controller to set the speed of the wheels as instructed by the commands from the diff drive controller.

### ros_ws/src directory
This directory contains four ros2 packages that I have developed for this robot
1. __autonomous_robot__: this is the package for the robot chassis, and also connects the robot arm to the chassis. This is updated from Josh Newans articubot one project
2. __diffdrive_arduino__: this is the diff drive hardware interface for the esp32 and motor controllers. Its an updated version of Josh Newans diffdrive_arduino package. Josh uses an arduino and I didn't want to change the name incase I broke something. This is adapted for a four wheel drive robot.
3. __xarm_description__: this is the description package for the xarm and contains the urdf as well as launch files to view the urdf in RViZ and simulation.
4. __xarm_hardware__: ros2 control hardware interface for the hiwonder xarm with the esp32 board. 

