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

## Usage

### Rviz Namespace Mux
```xml
<node name="namespace_mux" pkg="namespace_mux" type="namespace_mux" output="screen">
    <rosparam param="/namespace_mux/robot_namespace_ref">"robot"</rosparam>
    <rosparam param="/namespace_mux/rviz_namespace">"rviz"</rosparam>

    <rosparam param="/namespace_mux/active_bots">[
      "robot0"
      ]</rosparam>

    <rosparam param="/namespace_mux/subscribed_topics">[
      "/map_store_map",
      "/odom", 
      "/laser/merged",
      "/laser/scan_back",
      "/laser/scan_front",
      "/move_base/local_costmap/obstacle_layer_footprint/footprint_stamped",
      "/move_base/TrajectoryPlannerROS/local_plan",
      "/move_base/local_costmap/costmap",
      "/move_base/TrajectoryPlannerROS/global_plan",
      "/wormhole_marker",
      "/waiting_area_marker",
      "/particlecloud"
      ]</rosparam>

    <rosparam param="/namespace_mux/published_topics">[
      "/cmd_vel",
      "/initialpose",
      "/move_base_simple/goal",
      "/clicked_point"
      ]</rosparam>
</node>
```
