#!/usr/bin/bash

# launches a tmux session that can communicate to the aircraft via the mavlink bus
# lanunces the ros node and sends it a control message
# then listens to what it gets back on that channel

# i recommend launching this from the workspace directory
# this can be done by softlinking the file there

source install/setup.bash

tmux new -s gcs -n datalink -d

tmux send-keys 'ros2 run stardos_datalink datalink_ground --ros-args --params-file datalink.yaml' C-m

tmux neww -n heartbeat

tmux send-keys "sleep 1 && ros2 topic pub -1 /datalink/control stardos_interfaces/msg/Control 'options: >" C-m \
	'  { "heartbeat": { "pub": ["/mutt/apl_mutt_stardos_pi/raspicam/raspicam/raspicam0/heartbeat"] } }'"'" C-m

tmux send-keys "ros2 topic echo /mutt/apl_mutt_stardos_pi/raspicam/raspicam/raspicam0/heartbeat" C-m
