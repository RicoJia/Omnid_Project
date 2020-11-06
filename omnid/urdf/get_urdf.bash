rosrun xacro xacro --inorder -o delta_robot.urdf delta_robot_moveit.urdf.xacro R_base:=0.18 h_base:=0.009525 M_base:=2.5 R_platform:=0.062 h_platform:=0.01 M_platform:=0.5 L_lower:=0.200 L_upper:=0.368 base_offset:=0.046375
rosrun xacro xacro --inorder -o delta_robot_pybullet.urdf delta_robot_pybullet.urdf.xacro R_base:=0.18 h_base:=0.009525 M_base:=2.5 R_platform:=0.062 h_platform:=0.01 M_platform:=0.5 L_lower:=0.200 L_upper:=0.368 base_offset:=0.046375
rosrun xacro xacro --inorder -o disk.urdf disk.urdf.xacro R_disk:=1.0 h_disk:=0.1 M_disk:=50.0


#gz sdf -p delta_robot.urdf > delta_robot.sdf
#rm delta_robot.urdf
