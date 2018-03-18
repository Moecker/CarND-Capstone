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

![Traffic Light detection](imgs/tl-detector-ros-graph.png)

### Traffic Light Recognition
### Traffic Light Classification

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
