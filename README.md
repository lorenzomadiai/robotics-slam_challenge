# LiDAR-Based Autonomous Robot Navigation

Autonomous robotics project focused on navigation and obstacle avoidance in a simulated indoor environment using LiDAR sensor data.

The goal of the project was to develop an algorithm capable of controlling a robotic vacuum cleaner inside a house-like environment, allowing it to:
- detect doors,
- navigate through multiple rooms,
- avoid obstacles,
- recover from collisions or stuck situations,
- autonomously reach a final target area.

---

# Project Overview

The robot operates in a simulated environment using the CoppeliaSim/V-REP robotics simulator.

The navigation strategy is entirely based on real-time LiDAR sensor readings.  
The algorithm analyzes distance patterns in the environment to identify:
- free paths,
- door openings,
- nearby obstacles,
- room transitions.

The system dynamically adjusts wheel velocities to steer the robot toward safe and navigable directions.

---

# Main Features

## LiDAR-Based Navigation

The robot continuously processes 270-degree LiDAR scans to:
- estimate obstacle proximity,
- detect open spaces,
- infer doorway locations,
- select the safest movement direction.

---

## Door Detection Heuristics

Custom heuristics were implemented to identify doors by recognizing:
- sudden increases in measured distances,
- obstacle-free gaps between nearby walls,
- characteristic door-frame patterns.

These heuristics allow the robot to autonomously move between rooms.

---

## Obstacle Avoidance

The algorithm:
- avoids collisions,
- reacts to nearby obstacles,
- adapts motion trajectories in real time.

Wheel velocities are dynamically adjusted according to the detected navigation angle.

---

## Recovery Behaviors

The robot can detect when it becomes stuck by monitoring motor velocities.

Recovery actions include:
- backward motion,
- direction changes,
- reorientation maneuvers.
