#!/bin/bash


#------------ kit tray 1
#blue pulley
rosrun gazebo_ros spawn_model -sdf -x 0.0 -y 0.15 -z 0.05 -R 3.14159 -P 0 -Y 0 -file `rospack find nist_gear`/models/pulley_part_blue_ariac/model.sdf -model pulley_part_blue_20 -reference_frame agv1::kit_tray_1
#green gasket
rosrun gazebo_ros spawn_model -sdf -x -0.1 -y -0.2 -z 0.05 -R 0 -P 0 -Y -0.7853 -file `rospack find nist_gear`/models/gasket_part_green_ariac/model.sdf -model gasket_part_green_20 -reference_frame agv1::kit_tray_1
#red piston
rosrun gazebo_ros spawn_model -sdf -x 0.15 -y -0.2 -z 0.05 -R 0 -P 0 -Y 0.7853 -file `rospack find nist_gear`/models/piston_rod_part_red_ariac/model.sdf -model piston_rod_part_red_20 -reference_frame agv1::kit_tray_1

#------------ kit tray 2
#blue pulley
rosrun gazebo_ros spawn_model -sdf -x 0.0 -y 0.15 -z 0.05 -R 3.14159 -P 0 -Y 0 -file `rospack find nist_gear`/models/pulley_part_blue_ariac/model.sdf -model pulley_part_blue_30 -reference_frame agv2::kit_tray_2
#green gasket
rosrun gazebo_ros spawn_model -sdf -x -0.1 -y -0.2 -z 0.05 -R 0 -P 0 -Y -0.7853 -file `rospack find nist_gear`/models/gasket_part_green_ariac/model.sdf -model gasket_part_green_30 -reference_frame agv2::kit_tray_2
#red piston
rosrun gazebo_ros spawn_model -sdf -x 0.15 -y -0.2 -z 0.05 -R 0 -P 0 -Y 0.7853 -file `rospack find nist_gear`/models/piston_rod_part_red_ariac/model.sdf -model piston_rod_part_red_30 -reference_frame agv2::kit_tray_2
