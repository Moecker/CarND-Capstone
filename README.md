# Introduction
This write-up summarizes the approach, implementation and results of the SDC Capstone project.

[Original Udacity Readme](README_Udacity.md)

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
## Waypoint Updater
### Basic Updater
The basic implementation of the waypoint updater subscribes to the `/base_waypoints` and `/current_pose` topics and publishes the next 200 waypoints, starting from the ego position to the topic `/final_waypoints`.

### Full Updater

## Drive By Wire
The drive by wire node is responsible for turning desired velocities into actual commands, like braking and steering, the car can understand.
In addition, it needs to decide whether the DBW system shall get activated or not. This is communicated via the `/dbw_enabled` topic.
For safety reasons, the DBW system only published actuator commands when the system is intended to be active.

The controller task can be divided into two tasks: longitudinal and lateral control.

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
### Traffic Light Recognition
### Traffic Light Classification

# Results
## Video
[![Video](https://img.youtube.com/vi/pWww_Oiqx8U/0.jpg)](https://www.youtube.com/watch?v=pWww_Oiqx8U)

# Retrospective
## Team Collaboration
## Improvements
* Performance of the entire chain
* Follow the waypoints more smoothly
