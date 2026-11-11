#!/bin/bash
S="source ~/ros2_ws/install/setup.bash"
gnome-terminal --window --title="1 에뮬+RViz" -- bash -c "$S; ros2 launch bungeoppang2 sim.launch.py bake:=false; exec bash" &
sleep 25
gnome-terminal --window --title="2 공정"  -- bash -c "$S; ros2 run bungeoppang2 bake_sequence --serve; exec bash" &
sleep 1
gnome-terminal --window --title="3 총괄"  -- bash -c "$S; ros2 run bungeoppang2 orchestrator; exec bash" &
sleep 1
gnome-terminal --window --title="4 음성"  -- bash -c "$S; ros2 run bungeoppang2 voice_node; exec bash" &
