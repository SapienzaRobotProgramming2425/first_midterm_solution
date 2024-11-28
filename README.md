# First midterm -- Landmark scanner -- Deadline 26/11/2024
> [!CAUTION]
> It is **strictly prohibited** to alter any of the following files:
> - .eval_assignment.sh
> - src/rp_simulator/src/test_first_midterm.cpp
> - .github/workflows/classroom.yml
> 
> **The repository tracker is set to trigger any alteration to these files. Any _unexpected_ change will lead to the invalidation of the assigment.**

## Task
The task of the assignment is to implement a new type of sensor that can measure `Landmarks`.

The `LandmarkScanner` is used in the simulator node, which takes care of creating landmarks defined in the `yaml` files (`assets` directory). The task of the `LandmarkScanner` is to publish only a subset of these landmarks: those inside its `_range_max`. These landmarks must be expressed in the sensor frame.

The `LandmarkScanner` class is very similar to the `LaserScanner` one: it inherits from `WorldItem` and is ticked (it is updated in the simulation) by the `UnicyclePlatform` it is mounted on.

The `LandmarkScanner` does not subscribe to any topic, but it publishes two kinds of messages: the custom message `LandmarkArray`, and the `MarkerArray` message. The latter is used by Rviz2 for visualization purposes.

>[!TIP]
The only file you must work on is `src/rp_simulator/src/landmark_scanner.cpp`. The `LandmarkScanner` class is already implemented in this file; you just have to complete 11 TODOs.

After completing them, you can compile with:
```sh
colcon build
```
For running the simulator:
```sh
ros2 run rp_simulator simulator --ros-args -p config_file:=assets/diag_single_robot.yaml
```
For running Rviz2:
```sh
ros2 run rviz2 rviz2 -d rviz_configs/diag_single_robot.rviz
```
For running a simple teleoperation node (that you can exploit for moving the robot):
```sh
ros2 run rp_teleop teleop_node --ros-args -p robot_namespace:=robot_1
```
For testing your solution:
```sh
colcon test --ctest-args rp_simulator_test
colcon test-result --all --verbose
```

> [!IMPORTANT]
> - **The assignment is considered COMPLETE when all tests are successful**
> - **Do not modify any file except `src/rp_simulator/src/landmark_scanner.cpp`**
> - Modify only the code related to the 11 TODOs
> - You may push as many times as you want until the deadline \[**26/11/2024**\]
> - Tests are performed on every push on GitHub. Wait a bit, and you will receive a confirmation on whether your solution passed

# Additional features of the Simulator
The simulator also works with multiple robots. In this case, run the simulator with:
```sh
pixi run ros2 run rp_simulator simulator --ros-args -p config_file:=assets/diag_multi_robot.yaml
```
For running Rviz2:
```sh
pixi run ros2 run rviz2 rviz2 -d rviz_configs/diag_multi_robot.rviz
```
For running the teleoperation nodes:
```sh
pixi run ros2 run rp_teleop teleop_node --ros-args -p robot_namespace:=robot_1
pixi run ros2 run rp_teleop teleop_node --ros-args -p robot_namespace:=robot_2
```


