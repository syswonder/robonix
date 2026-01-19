# (WIP) four wheel steering controller

This package aims to implement functionality of [gazebo_ros_four_wheel_steering](https://github.com/Kettenhoax/gazebo_ros_four_wheel_steering) using the ros2 [controller_interface](https://index.ros.org/p/controller_interface/).

Features of gazebo_ros_four_wheel_steering implemented:

* [x] control using FourWheelSteeringStamped messages
* [x] publish current speed and steering angle state as FourWheelSteeringStamped messages
* [x] command timeout handling
* [ ] state noise simulation
* [ ] latency simulation

Strongly inspired by the current [diff_drive_controller](https://index.ros.org/p/diff_drive_controller/) implementation.

## License

Copyright 2021 Austrian Institute of Technology GmbH

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.