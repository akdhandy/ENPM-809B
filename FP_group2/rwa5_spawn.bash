#!/bin/bash


#------------ kit tray 1
#blue pulley
rosrun gazebo_ros spawn_model -sdf -x 0.0 -y 0.0 -z 0.0 -R 0 -P 0 -Y 0 -file `rospack find nist_gear`/models/pulley_part_blue_ariac/model.sdf -model pulley_part_blue_20 -reference_frame agv1::kit_tray_1

#------------ kit tray 2
#blue pulley
rosrun gazebo_ros spawn_model -sdf -x 0.0 -y 0.0 -z 0.0 -R 0 -P 0 -Y 0 -file `rospack find nist_gear`/models/pulley_part_blue_ariac/model.sdf -model pulley_part_blue_30 -reference_frame agv2::kit_tray_2

