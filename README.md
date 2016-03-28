# six_point_two_eight

ROS and C++14 sample. Getting 360 degree PointCloud with TurtleBot.

![360 degree world](https://tail-island.github.io/six_point_two_eight/src/images/world_model.png)

![360 degree target](https://tail-island.github.io/six_point_two_eight/src/images/target_model.png)

## How to use

### Build

```bash
# Please setup g++-4.9. Beacuse code uses C++14.

$ cd <catkin_ws>/src
$ git clone https://github.com/tail-island/six_point_two_eight.git
$ cd <catkin_ws>
$ catkin_make
```

### Execute

```bash
$ roslaunch six_point_two_eight turtlebot_driver.launch
```

And

```bash
$ roslaunch six_point_two_eight make_world_models.launch
# PCL files will be created on /tmp. When finished, press Ctrl-c.

$ roslaunch six_point_two_eight register_world_models.launch
# Sorry, it needs long time...

$ pcl_viewer /tmp/*.pcd
```

Or

```bash
$ roslaunch six_point_two_eight make_target_models.launch
# When finished, press Ctrl-c.

$ roslaunch six_point_two_eight register_target_models.launch
$ pcl_viewer /tmp/*.pcd
```

## Document

See [ROS Programming Guide (Japanese)](https://tail-island.github.io/six_point_two_eight/).

## License

This software is licensed under the BSD License.
