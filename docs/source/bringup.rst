############
SDPP Bringup
############

*************
Launch Files
*************

Two agents
==========
launches instance with 2 human agents

.. code-block:: xml

    <arg name="open_rviz" value="true"/>
    <!-- location of static map yaml -->
    <arg name="map_file" default="$(find sdpp_gazebo)/config/maps/narrow_3_goal_10x10.yaml" />
    <!-- location of gazebo world file-->
    <arg name="world" default="$(find sdpp_gazebo)/worlds/narrow_3_goal_10x10.world"/>
    <arg name="agent_0" value="true" />
    <arg name="agent_1" value="false" />


*************
RViz Configs
*************

test_rviz
=========

basic rviz launch shows navigation plugin topics, agent locations, and static map


********
Scripts
********

timed_roslaunch
===============

bash script to allow timed launching of ros nodes and actions
