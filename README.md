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



