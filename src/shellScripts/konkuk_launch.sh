terminator -e "roscore"  &
sleep 3
terminator -e "roslaunch sk_autopilot mavros.launch"  &
sleep 3
terminator -e "roslaunch sk_autopilot waypoint_generator.launch" &
sleep 3
terminator -e "roslaunch sk_autopilot waypoint_follower.launch" &
sleep 3
