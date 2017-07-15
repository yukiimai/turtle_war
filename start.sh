source ../../devel/setup.bash

gnome-terminal -e "/opt/ros/indigo/roslaunch turtle_war make_field.launch"

sleep 5

gnome-terminal -e "/opt/ros/indigo/roslaunch turtle_war spawn_robot.launch"


gnome-terminal -e "/opt/ros/indigo/roslaunch turtle_war candy.launch"

