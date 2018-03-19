# Introduction
This write-up summarizes the approach, implementation and results of the SDC Capstone project.
We first will introduce the team members, followed by a deep dive into the implementation for each component, and finally summarizing the results and give a retrospective.

Link to [Original Udacity Readme](README_Udacity.md)

# Team Gauss
## Team Members
Name | Email | Slack
--- | --- | ---
Sascha Moecker (Team Lead)  | moeckersascha@hotmail.com | @sascha
Hiroshi Ichiki  | ichy@somof.net | @hiroshi
Saina Ramyar  | saina.ramyar@gmail.com | @saina
David Obando | david.obando@gmail.com | @davidobando
William Leu | reavenk@gmail.com | @reavenk

## Slack Channel
[Link](https://carnd.slack.com/messages/G9GQ610TH/)

# Project Documentation
## Architecture

![Component Diagram](imgs/integration_component_diagram.png)

## Waypoint Updater
The waypoint updater decides which waypoints from the static map are published to the waypoint follower and adapts its velocity according to obstacles.

![Waypoint Updater](imgs/waypoint-updater-ros-graph.png)

### Basic Updater
The basic implementation of the waypoint updater subscribes to the `/base_waypoints` and `/current_pose` topics and publishes the next 200 waypoints, starting from the ego position to the topic `/final_waypoints`.
To do so, the waypoint updater node searches for the closest waypoint with respect to our current position. Once the index within the base waypoints array has been found, we slice the waypoints to a maximum of 200 and publish those using the final waypoints topic.
The actual content of each waypoint include the pose (position and orientation) and twist (linear and angular velocities).
The desired longitudinal speed is set to the target speed obtained from `/waypoint_loader/velocity` parameter which is eventually consumed by the waypoint follower (not part of the project tasks).

Since this approach ignores obstacles and traffic lights, an improvement has been developed which also takes stop points into account.

### Full Updater
The full updater enhances the basic updater by enabling the car to vary its speed and allowing to stop in front of traffic lights.
The major change is that the target velocity is not constant anymore but derived from the target stop point.
A target stop point can be a red traffic light as obtained from the traffic light detection node.
The information comes in a form of a global waypoint index at which the car is about to stop. A `-1` indicates no need for a stop (either no traffic light or a green one).

To come to a smooth and safe stop in front of a red traffic light, a target velocity trajectory is generated with a linear decrease of speed from the current waypoint velocity to zero. We define the notion of a "comfort braking" with an approximate deceleration of `1.5 m/s^2`.
This is the basis to calculate the distance at which a deceleration maneuver is started to gain a smooth braking.
In addition, a maximum allowed velocity is derived from the comfort braking deceleration value at which we actually achieve a full stop.
If the current velocity tops the maximum velocity, we allow for a "progressive braking" which decelerates faster until we reach the max allowed speed.

## Drive By Wire
The drive by wire node is responsible for turning desired velocities into actual commands, like braking and steering, the car can understand.
In addition, it needs to decide whether the DBW system shall get activated or not. This is communicated via the `/dbw_enabled` topic.
For safety reasons, the DBW system only published actuator commands when the system is intended to be active.

The controller task can be divided into two tasks: longitudinal and lateral control.

![Drive By Wire](imgs/dbw-node-ros-graph.png)

### Longitudinal Controller
There is a PID implementation used for the longitudinal control which maintains the speed of the vehicle.
The output is a periodically sent value via the `/throttle_cmd` topic and the `/brake_cmd` command, respectively.
The error used for the PID input is the difference between the current and the desired longitudinal speed.
The desired longitudinal speed is published by the waypoint follower in the `/twist_cmd` command topic, the current velocity via `/current_velocity`.

For both longitudinal commands, throttle and braking, the same PID is used.
Whenever the PID returns a positive value, meaning the current speed is lower than the desired one, we want to accelerate the car.
The controller output is scaled to comply the requirement of a throttle value between 0.0 and 1.0. Therefore, a soft scaling using a `tanh` is employed.

Accordingly, a negative controller output means breaking and the same post-processing step is applied, scaling the breaking value to a
feasible range between 0.0 and roughly 1000Nm; latter obtained from a calculation including the max brake torque, vehicle mass, wheel radius and deceleration limit.
To further smooth the breaking, a low pass filter is additionally applied.

For our implementation, the provided PID suffices our needs and was used without modifications.
Since the PID controller has a static I-component, a PID reset is triggered when DBW is not active.

### Lateral Controller
Likewise the PID controller, a yaw controller is used to create actual steering commands.
The output is a periodically sent value via the `/steering_cmd` topic.
Since the steering is inheritably more depended on the actual waypoints, deviations could be observed more evidently.
Problems during development were in particular the bad performance of the whole chain when running in the simulator.
Counter measurements were taken in the waypoint updater b y decoupling the reception of the position message from the updating of the final waypoints.

Internally the yaw controller uses the current velocity, angular velocity and linear velocity to compute an angle for the steering wheel.
This angle then translated to the steering command and was published.

To further smooth the control, a the same low pass filter as used for the braking value has been applied.
Also here, the provided yaw controller suffices our needs and was used straight away.

## Traffic Light detection

Traffic light detection is done whenever the vehicle approaches points known to have traffic lights. The purpose is simple: detect via optical analysis what the state of the traffic light is. If the traffic light is found to be red, this will be notified to other nodes so they can initiate a stopping maneuver. The output of this node is simply an integer number which will tell which of the base waypoints ahead of us has a stop sign in display. If no stop sign is ahead of us, then the special value `-1` is sent to the topic.

The vehicle, both in the simulator as well as Udacity's Carla, has a camera mounted in the front side of the dash so as to obtain color images that are fed to the traffic light detection node. The camera images are only processed when the car is in the vicinity of a known location of a traffic light. 

Traffic light locations are specified in configuration files to ROS, located in the [traffic light detection directory](./ros/src/tl_detector).
  - `sim_traffic_light_config.yaml` is used for the highway simulator,
  - `site_traffic_light_config.yaml` is used for Carla in the real world, and for the parking lot simulator.

Besides containing the traffic light location in the known map, the configuration files contains relevant data regarding camera configuration and file names for the weights to be used in our traffic light classification neural network.

Finally, the traffic light detection node could not operate without consuming streams from the following topics:
  - `/current_pose`: used in order to obtain the vehicle's current position in the map.
  - `/base_waypoints`: used in order to calculate the closest waypoints to the vehicle's current position. Also, the output of this node is an index number to the base waypoints.
  - `/image_color`: used in order to obtain a snapshot image from the vehicle's dash camera.

The traffic light detection node's computation will be output to the `/traffic_waypoint` topic.

![Traffic Light detection](imgs/tl-detector-ros-graph.png)

The problem of traffic light detection is split in two main parts:
  1. Traffic light recognition: in regards to the known data (such as traffic light locations) and the vehicle's state.
  2. Traffic light classification: in regards to the image obtained from the vehicle's dash camera.

### Traffic Light Recognition
### Traffic Light Classification

Traffic Light Classification performs on demand from the traffic light detection and returns the color state of the traffic light reflected in the camera image with a constant value in TrafficLight.
The traffic light classification solves two tasks.

  - find traffic lights as bounding boxes in the camera image
  - predict the color states of the traffic lights detected

Deep Learning is a successful technique to solve them at once. The traffic light classification applies SSD:Single Shot Multibox Detector which is one of the powerful deep learning algorithms.
To classify the color state, the traffic light classification does:

   1. resize camera image to (300, 300) for the SSD
   2. utilize the SSD and predict traffic lights with their color state, confidence and boundary box
   3. get detections with the confidence is larger than 0.6
   4. return the color state of the top of the detection list

The color state values are defined in TrafficLight class as below.

  - TrafficLight.RED
  - TrafficLight.YELLOW
  - TrafficLight.GREEN
  - TrafficLight.UNKNOWN


# Results
## Videos
Description | Link
--- | ---
Driving on the test lot simulation scenario without camera and traffic light info  | [![Video](https://img.youtube.com/vi/K93AdV7zbSs/0.jpg)](https://www.youtube.com/watch?v=K93AdV7zbSs)
Driving on the highway simulation scenario ignoring traffic lights with temporary manual override  | [![Video](https://img.youtube.com/vi/VR0Se5eRiIc/0.jpg)](https://www.youtube.com/watch?v=VR0Se5eRiIc)
Driving on the highway simulation scenario adhering to red traffic light (state directly taken from simulator)  | [![Video](https://img.youtube.com/vi/qFJfD4xo16s/0.jpg)](https://www.youtube.com/watch?v=qFJfD4xo16s)

# Retrospective
## Team Collaboration
Team communication primarily was done in our Slack channel, because it allowed for persistent and delayed communication.
We set up meeting dates for summarizing our progress. Challenging was to overcome the huge time zone differences, since team members were spread around the world (America, Europe, Asia).

We used a common code base hosted on [Github](https://github.com/Moecker/sdc_capstone) with granted rights to push directly to master for all team members, avoiding delayed integration if pull requests would have be used.
For feature development, personal or collaborative branches (i.e. for the traffic light classifier) were created and later merged into the master.

The tasks break down followed the advice given in the classroom and manifested into this Write-up structure.
We could benefit from the decoupled, component oriented architecture provided by ROS and its powerful tools for faking messages by having clear interfaces between nodes. Although every team member was assigned for a particular task as the "driver", the implementation was done collaboratively.

The task break down was chosen like so:
1. Waypoint Updater Node (Partial): Complete a partial waypoint updater.
2. DBW Node: After completing this step, the car should drive in the simulator, ignoring the traffic lights.
3. Traffic Light Detection: Detect the traffic light and its color from the /image_color.
4. Waypoint publishing: Once you have correctly identified the traffic light and determined its position, you can convert it to a waypoint index and publish it.
5. Waypoint Updater (Full): Your car should now stop at red traffic lights and move when they are green.

## Improvements
* Performance of the entire chain
* Follow the waypoints more smoothly
