# ENPM-809B
RWA-4


Build details:

1. Copy the package onto the src folder of your ariac_ws workspace.
2. execute the following commands:
cd ariac_ws/
source ~/ariac_ws/devel/setup.bash
catkin build rwa4_group2
roslaunch rwa4_group2 rwa4.launch load_moveit:=true
rosrun rwa4_group2 rwa4_node

