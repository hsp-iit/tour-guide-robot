# ROS Navigation

Interesting pull requests on ROS navigation stack that we could implement/look:

- https://github.com/ros-planning/navigation/issues/683
- https://github.com/ros-planning/navigation/pull/525
- https://github.com/ros-planning/navigation/pull/999



### Installation steps for R1 packages

- Create a directory (usually in ../robot/) called 'catkin_ws' and another directory inside 'catkin_ws' called 'src'
- Git clone any ros package from github inside the 'src' directory and cd to the 'catkin_ws'
- The packages we currently need are:
  - https://github.com/robotology-dependencies/r1-navigation - Modified ROS Navigation Stack with better voxel clearing
  - https://github.com/rst-tu-dortmund/costmap_prohibition_layer - Prohibition layer without rostopic support
- After downloading the sources, in the main 'catkin_ws' just run the command 'catkin_make install' (install is optional but reccomended)
- If installation is succeful the file 'setup.bash' needs to be sourced in the '/devel/' folder inside the 'catkin_ws' (source devel/setup.bash)

That should install all the necessary packages that are not already preinstalled on the robot.



### Configuration

Having installed the above, the configurations can be changed here:
  - https://github.com/robotology-playground/tour-guide-robot/tree/master/app/navigation/launch/2d_nav

, where the prohibition_layer obstacles can be defined here:
  - https://github.com/robotology-playground/tour-guide-robot/blob/master/app/navigation/launch/2d_nav/prohibition_areas.yaml
