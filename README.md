# spacenav-driver

A C++ ROS package for real-time conversion of 3D motion controller events to ROS messages. In this package, button IDs are configured for 3Dconnexion's space-mice. To run this package, you need to install these beforehand:
- [ROS](http://wiki.ros.org/ROS/Installation)
- [spacenavd](https://github.com/FreeSpacenav/spacenavd)
- [libspnav](https://github.com/FreeSpacenav/libspnav)



## Run on Terminal

### Terminal #1

```sh
roscore
```

### Terminal #2

```sh
cd ./MPI_ws
catkin_make
source devel/setup.bash
rosrun spacenav_node spacenav_node
```



## Author

👤 **Aras Güngöre**

* LinkedIn: [@arasgungore](https://www.linkedin.com/in/arasgungore)
* GitHub: [@arasgungore](https://github.com/arasgungore)
