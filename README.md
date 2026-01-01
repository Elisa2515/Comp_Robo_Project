## Robo Behaviors Project

The first project for Computational Robotics was an introduction to working in ROS 2 with the Neato robot platform. In this project, we implemented a set of robot behaviors that demonstrate basic robot control, sensor processing, and autonomous decision-making. These behaviors include robot teleoperation, driving in a square, wall following, person following, and obstacle avoidance.
---
## Robot Teleop

The teleoperation mode allows a human user to directly control the robot’s motion using keyboard input. Specific keys are mapped to motion states, allowing the robot to move forward, move backward, turn left, turn right, and stop. This behavior was useful for testing basic motion commands and gaining intuition about how velocity commands affect the robot’s movement.
---
## Driving in a Square

The goal of the square path navigation behavior was to drive the Neato robot in a square pattern autonomously. To achieve this, a time-based control approach was used. The main loop repeats once for each side of the square. For each iteration, the robot drives forward for a predetermined amount of time and then performs an in-place 90-degree turn.

The turning behavior is implemented by commanding a constant angular velocity and sleeping for an amount of time proportional to the desired turn angle. This approach does not rely on sensor feedback and is therefore open-loop, meaning small errors can accumulate over time.

```python
def turn(self, degrees):
    """Turn to a specified angle"""
    angular_vel = 0.3
    self.drive(linear=0.0, angular=angular_vel)
    sleep(degrees / angular_vel)
    self.drive(linear=0.0, angular=0.0)
```
---
## Wall Following

The wall following behavior allows the robot to maintain a fixed distance from a nearby wall while driving forward. This behavior uses laser scan data to detect the closest object around the robot and treats that object as the wall.

If no valid wall is detected, the robot rotates slowly in place to search. Once a wall is found, the robot determines whether the wall is on its left or right side and attempts to keep the wall at a perpendicular angle (90° on the left or 270° on the right).
---
## Laser Processing

Invalid laser readings (infinite or NaN values) are filtered out

Each laser reading is associated with an angle using the scan’s angle increment

Angles are converted to degrees and wrapped into a 0–360° range

The closest valid laser point is selected and stored as the wall distance and angle

## Control Strategy

A proportional controller is used:

Angular control keeps the robot oriented relative to the wall

Linear control adjusts forward speed to maintain a desired distance from the wall

Speed limits are applied to prevent unsafe or unstable motion. This simple approach works well in clear environments but can be affected by noise or nearby objects.

## Person Following

The person following behavior allows the robot to follow a person using laser scan data. The robot searches for the closest object directly in front of it and assumes that object is the person.

If no object is detected, the robot slowly rotates in place to search. When a person is detected, the robot turns to face them and moves forward or backward to maintain a desired following distance.

## Laser Processing

Only laser points within a forward-facing window (approximately ±30°) are considered

Invalid readings are filtered out

The closest object within this window is selected as the target

Objects that are too close or too far away are ignored to reduce noise

## Control Strategy

A simple proportional controller is used:

Angular control keeps the robot facing the person

Linear control adjusts the robot’s speed to maintain a fixed following distance

The robot only moves forward when the person is roughly centered in front of it. Speed limits are applied to ensure smooth and safe behavior.

## Obstacle Avoidance

Obstacle avoidance allows the robot to reactively prevent collisions using laser scan data. When an obstacle is detected within a certain distance, the robot adjusts its motion to avoid it by slowing down, stopping, or turning away. This behavior demonstrates basic reactive control and safe navigation in cluttered environments.

## Summary

This project provided hands-on experience with ROS 2, sensor data processing, and basic robot control strategies. By implementing multiple behaviors, we explored both open-loop and closed-loop control, as well as the challenges of using real sensor data for autonomous decision-making.
