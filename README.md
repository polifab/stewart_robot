# Stewart Platform Webots Controller

This project contains a C++ Webots Controller for a Stewart Platform world, started from [stewart_platform.wbt](https://cyberbotics.com/doc/guide/samples-demos) and integrated with ROS Noetic.

## What's inside

- Inverse Pose and Rate kinematics
- Forward Kinematics
- Trapezoidal Velocity Trajectory to reach a setpoint in cartesian space
- 4 Modes of operation:
  1. Command a setpoint using a trapezoidal velocity of the End Effector
  2. Command a twist to the End Effector (it has been configured to receive inputs from joystick)
  3. Command a setpoint using just inverse kinematics and joints position control
  4. Forward kinematis guess of End Effector position
- Configurable robot parameters through YAML configuration file
- Docker image available for fast deployment

## Dependencies

The project depends on:
- [Webots 2022a](https://cyberbotics.com/)
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [webots_ros](http://wiki.ros.org/webots_ros)
- [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)

## How to build

Open a BASH shell in the location of the base directory of the project and then:

```bash
source /opt/ros/noetic/setup.bash
cd controller_ws
catkin_make
```

## How to run

Open a BASH shell in the location of the base directory of the project and then:

```bash
source /opt/ros/noetic/setup.bash
source controller_ws/devel/setup.bash
roslaunch stewart_controller stewart.launch
```
A Webots GUI showing the Stewart Platform should appear and the robot is by default set on mode 1.

## Deployment with Docker

Docker simplifies the deployment, handling all the dependencies and virtualizing the environment.
Open a BASH shell in the location of the base directory of the project and then:
```bash
docker-compose build # if newer docker compose version, use "docker compose build"
xhost + # disable access control for docker to connect to display
docker-compose up # if newer docker compose version, use "docker compose up"
```

A Webots GUI showing the Stewart Platform should appear and the robot is by default set on mode 1.

## Configuration

A YAML file describing the geometry of the robot is available in:
- Stewart/controller_ws/src/stewart_controller/config/stewart.yaml

It contains two matrices, A and B. The A matrix describes the attachment points of the joints on the base platform in world coordinate frame. The B matrix describes the attachment points of the joints to the moving platform in a solidal reference frame centered on the centroid of the cylinder which describes the moving platform.

A ROS launch file is also provided to automatise the launching of all the nodes in:
- Stewart/controller_ws/src/stewart_controller/launch/stewart.launch

It contains some parameters which are useful to the user:
```
...
        mode: It is the parameter which allows to choose among the aforementioned modes; ( 1 - Trapezoid, 2 - Twist, 3 - Pose, 4 - FK)
        trapz_acc: It is the parameter which defines the angular coefficient of the trapezoidal velocity trajectory slope.
        trapz_max_vel: It is the maximum velocity at which the trapezoidal velocity should be saturated
...
All the parameters can also be changed from ROS topics.

```

## Example of usage

After that the program has been correctly launched, if you have ROS installed natively on your system, simply open a BASH shell and type:
```bash
source /opt/ros/noetic/setup.bash
rostopic pub /stewart_controller_node/pose_trapz_setpoint geometry_msgs/Pose "position:
  x: 0.19136
  y: -0.1559
  z: 2.9
orientation:
  x: 0.073591
  y: -0.04485
  z: 0.04527
  w: 0.99524"
```
Since the robot starts in mode 1, this command will move the platform from the initial coordinates to the new required ones using the trapezoidal trajectory with the properties specified in the launch file.
To change mode:

```bash
rostopic pub /stewart_controller_node/mode std_msgs/Int32 "data: <desired_mode>"
```

In mode 2 the program will automatically detect and the parse the input from a joystick and it will be possible to command the twist of the platform using the analog controllers and the arrows.

If ROS is not natively installed and you are using Docker, it is sufficient to open a new BASH shell and type:
```bash
docker exec -it stewart_platform_container bash
```
This will open a BASH shell inside the Docker container where it will be possible to execute the steps showed before.

##
