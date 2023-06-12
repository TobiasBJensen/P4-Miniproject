# P4-Miniproject

To get the ros2 nodes up and working.
1. In the terminal go to the plc_ws folder and run "colcon build"
2. "source install/setup.bash"
3. "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash"

The program can be launched in 2 ways.
First is.
1. In the terminal, go to plc_ws folder
2. run "./plc_run"

If the first dose not work try.
1. Check the ip addr that will be used for the tcp connection
2. "ros2 launch plc_control plc_control ip_addr:='The ip addr found earlier'"

