# Tour Guide Robot

#### The Dockerfile(s) can be found [here](https://github.com/robotology-playground/tour-guide-robot/blob/master/docker_stuff).

##### - Known problems are:
- Some applications dont behave nicely on yarpmanager (ex gazebo due to clock), so they are run seperatly with a .sh file.
- The image has to be built individually as I can't find a way to share the docker image while keeping the repository in dockerhub private.

## To launch the simulation:

### outside the docker 

#### Install docker
`sudo apt install docker`

and run post installatio steps https://docs.docker.com/engine/install/linux-postinstall/docker%20first%20installation%20guide

#### open a terminal and go to the docker_sim directory
`cd docker_stuff/docker_sim`

#### compile the docker
`./build-docker.sh`

#### launch the docker
`./start-docker-sim.sh`

### inside the docker

#### go to the correct directory
`cd ~/tour-guide-robot/app/navigation/scripts/`

#### run the simulation
`./start_sim.sh`

### in yarp manager

#### open the app **Navigation_ROS_R1_SIM**

#### run all the modules one by one in order (pay attention if navigationGUI starts correctly)

#### connect all

#### then localize twice in yarp navigation gui and check that in ros is correctly initialized

#### if in rviz you see local costmap, global costmap, lidar and voxels then you're done
