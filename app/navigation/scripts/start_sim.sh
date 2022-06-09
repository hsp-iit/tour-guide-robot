roscore &
sleep 2;
yarpserver --ros &
sleep 1;
rosrun gazebo_ros gzserver -s libgazebo_yarp_clock.so /home/user1/tour-guide-robot/app/maps/SIM_GAM/GAM.world &
sleep 2;
yarprun --server /console --log &
