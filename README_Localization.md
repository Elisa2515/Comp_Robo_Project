## Robot Localization Exercise
## Goal 
This exercise is based on the particle filter approach described in the project instructions, with a reduced scope to highlight the core algorithmic ideas.

## Overview

The implementation uses a simplified particle filter to track a robot’s pose 
(x,y,θ)
(x,y,θ) using odometry data. A set of particles is maintained and updated as the robot moves, with noise added to model real-world uncertainty. The particle cloud is published for visualization in RViz.

## Method

The algorithm follows the standard particle filter workflow:

Initialization – Particles are initialized around a starting pose with Gaussian noise.

Motion Update – Each particle is propagated using odometry deltas with added noise.

Weighting – Particles are weighted using a simple confidence heuristic to keep the distribution coherent.

Resampling – Particles are resampled based on their weights to concentrate likely poses.

## ROS Topics
## Subscribed

/odom (nav_msgs/Odometry) – Used to compute robot motion between time steps.

## Published

/mini_pf_particles (geometry_msgs/PoseArray) – Visualizes the particle cloud in RViz.

## Scope and Limitations

This exercise intentionally does not include laser scan processing or map-based localization. Instead, I focused on the core mechanics of particle filtering—motion uncertainty, weighting, and resampling—to provide a clear and accessible implementation.

## Key Takeaways

Odometry uncertainty causes pose estimates to spread over time.

Resampling helps concentrate particles around more likely poses.

Visualization is critical for understanding and debugging probabilistic robotics algorithms.
