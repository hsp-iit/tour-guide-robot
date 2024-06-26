yarpserver &
sleep 1;
ros2 launch gazebo_ros gazebo.launch.py extra_gazebo_args:=-slibgazebo_yarp_clock.so world:=/home/user1/tour-guide-robot/app/maps/SIM_MADAMA/madama.world &
sleep 1;
yarprun --server /console --log &
sleep 1;
yarplogger --start &
sleep 1;
yarpmanager
