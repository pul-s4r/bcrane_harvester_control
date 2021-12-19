search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=crane_model_v02.srdf
robot_name_in_srdf=crane_model_v02
moveit_config_pkg=crane_model_v02_moveit_config
robot_name=crane_model_v02
planning_group_name=crane_arm_serial
ikfast_plugin_pkg=crane_model_v02_crane_arm_serial_ikfast_plugin
base_link_name=base_link
eef_link_name=jt3_l
ikfast_output_path=/home/jjmoey/thesis_ws/crane_model_v02_crane_arm_serial_ikfast_plugin/src/crane_model_v02_crane_arm_serial_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
