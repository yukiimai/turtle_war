source ../../devel/setup.bash

gnome-terminal -e "roslaunch turtle_war make_field.launch"

sleep 5

gnome-terminal -e "roslaunch turtle_war spawn_robot.launch"

