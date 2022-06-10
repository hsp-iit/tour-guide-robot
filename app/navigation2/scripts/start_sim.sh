yarpserver &
sleep 1;
gazebo /home/user1/tour-guide-robot/app/maps/SIM_GAM/GAM.world &
sleep 1;
yarprun --server /console --log &
sleep 1;
yarplogger --start &
sleep 1;
yarpmanager
