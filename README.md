## Launch 
This launches the keypoint estimator and the commander. The commander sets the initial position and the speed limit, then navigates to the goal pose. 
The keypoint estimator publishes on the statekey topic, the time to the next keypoints 
``
ros2 launch lh_demo_commander lh_demo_commander.launch.py 
``
## Other executables 
If it is undesirable to move the robot, the statekey topic can be emulated with 
``
ros2 run lh_demo_commander emulate_keypoints
``
To monitor the statekey topic run 
``
ros2 run lh_demo_commander keypoint_subscriber
``

