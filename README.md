# Tour Guide Robot

#### The Dockerfile(s) can be found [here](https://github.com/robotology-playground/tour-guide-robot/blob/master/docker_stuff).

##### - Known problems are:
- Some applications dont behave nicely on yarpmanager (ex gazebo due to clock), so they are run seperatly with a .sh file.
- The image has to be built individually as I can't find a way to share the docker image while keeping the repository in dockerhub private.
