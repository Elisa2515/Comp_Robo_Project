# Comp_Robo_Project Repo
# Robo Behaviors Project

The first project for Computational Robotics was an introduction to working in ROS2 with the Neato robot platform. In this project, we implemented a set of behaviors that demonstrate basic robot control, sensor processing, and autonomous decision-making. These behaviors include robot teleoperation, driving in a square, wall following, person following, and obstacle avoidance.

---

## Robot Teleop

The teleoperation mode allows a human user to directly control the robot’s motion using keyboard input. Specific keys are mapped to motion states, allowing the robot to move forward, move backward, turn left, turn right, and stop. This behavior was useful for testing basic motion commands and gaining intuition about how velocity commands affect the robot’s movement.

---

## Driving in a Square

The goal of the square path navigation behavior was to drive the Neato robot in a square pattern autonomously. To achieve this, a time-based control approach was used. The main loop repeats once for each side of the square. For each iteration, the robot drives forward for a predetermined amount of time and then performs an in-place 90-degree turn.

The turning behavior is implemented by commanding a constant angular velocity and sleeping for an amount of time proportional to the desired turn angle.

```python
def turn(self, degrees):
    """Turn to a specified angle"""
    angular_vel = 0.3
    self.drive(linear=0.0, angular=angular_vel)
    sleep(degrees / angular_vel)
    self.drive(linear=0.0, angular=0.0)
