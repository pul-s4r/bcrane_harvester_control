DEPENDENCIES: 
- coppeliasim_msgs_srvs: https://github.com/mahmoud-a-ali/coppeliasim_msgs_srvs
- coppeliasim_ros_control: https://github.com/mahmoud-a-ali/coppeliasim_ros_control
- coppeliasim_ros_services: https://github.com/tud-cor/coppeliasim_ros_services
- coppeliasim_run: https://github.com/tud-cor/coppeliasim_run

git clone the above repositories into the src folder and run catkin_make to compile. 

Ensure you have CoppeliaSim 4.2.0 and ROS Noetic installed, as well as the $COPPELIASIM_ROOT workspace variable defined. 
 
Useful launch files: 
- Send robot geometry to params server: roslaunch crane_model_v02 crane_upload.launch
- Launch simulation environment: roslaunch crane_cs_sim crane_bringup.launch
- Launch crane controllers: roslaunch crane_cs_sim crane_control.launch
- Launch vision system: roslaunch crane_vision crane_vision_detection.launch
- Launch crane planner: roslaunch crane_manager crane_manager.launch
