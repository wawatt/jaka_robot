# jaka dev ros2 humble
For example jaka_zu7
- [x] jaka_robot_v2.2/src/jaka_description only jaka_zu7
- [x] jaka_robot_v2.2/src/jaka_msgs
- [ ] jaka_robot_v2.2/src/jaka_driver
- [ ] jaka_robot_v2.2/src/jaka_zu7_moveit_config
## run (win robostack)
```
mamba install ros-humble-joint-state-publisher-gui
colcon build --merge-install --cmake-args  -DCMAKE_BUILD_TYPE=Release  --event-handlers console_direct+
call install/setup.bat
ros2 launch jaka_description jaka_zu7_rviz_control.launch.py
```
## SDK 2.1.11


# jaka_robot
* Latest package: jaka_robot_v2.2 (Includes documents and source code)  
* Remember to compile the first time you use it.
* ROS1 support only
