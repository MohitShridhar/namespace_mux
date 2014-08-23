ROS Topic Namespace Multiplexer
=============

Topic-tools [mux](http://wiki.ros.org/topic_tools/mux) allows you to route many incoming topics into one outgoing topic. But this package allows you multiplex both many-to-one connections & one-to-many connections.

![many to one](images/many_to_one.png)

![one to many](images/one_to_many.png)

This might be useful when you want to control multiple robots using Rviz (see [multi_map_navigation](https://github.com/MohitShridhar/multi_map_navigation)).

## Dependencies & Prerequisites
[ROS Hydro](http://wiki.ros.org/hydro), [Gazebo 3.0+](http://gazebosim.org/), [Catkin](http://wiki.ros.org/catkin): [topic_tools](http://wiki.ros.org/topic_tools), [tf](http://wiki.ros.org/tf) (see [package.xml](package.xml))

## Installation
Clone and catkin_make
```bash
$ git clone https://github.com/MohitShridhar/namespace_mux.git
$ cd <catkin_ws>
$ catkin_make --pkg namespace_mux
```

## Guide

See [wiki](https://github.com/MohitShridhar/namespace_mux/wiki/User-Guide) for more details.