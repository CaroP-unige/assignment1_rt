# Assignment 1 - Research Track - Turtle Motions

This project was designed as a practical exercise to learn the fundamental concepts of ROS (Robot Operating System). Through the turtle simulation provided by the turtlesim package, users can interactively explore the use of ROS node communications based on Publishers, Subscribers, and services.

The project allows users to control the movements of two turtles within a simulated environment while implementing essential safety controls to ensure proper behavior.

Specifically, it focuses on key ROS concepts such as:

- Publishing and subscribing to messages between ROS nodes.
- Using services to add new elements to the environment (e.g., creating a second turtle with `turtlesim/Spawn`).
- Controlling movements via velocity messages (`geometry_msgs/Twist`).
- Managing collisions and enforcing spatial boundaries.

This approach provides an ideal opportunity for those seeking to delve into robotic programming and ROS.
Additionally, comments in the code provide further technical details.

## Table of Contents
1. [Description](#description)
2. [Features](#features)
3. [Technologies](#technologies)
4. [Installation](#installation)
5. [Usage](#usage)

## Description

This project enables users to move one of the two turtles in the workspace, allowing them to choose:
- Which turtle will perform the movement.
- The direction of movement (right, left, forward, backward).
- The linear speed of the movement.

**Implemented Controls:**
- Turtles cannot leave the workspace boundaries.
- Turtles maintain a minimum safety distance (>= 2 units).

In both cases, the movement is stopped by forcing the speed to zero and applying a corrective velocity to ensure the turtles stay within boundaries or maintain a safe distance.

## Features

- **Feature 1: User interaction (`node1.cpp`)**

   - Spaw a new turtle with `turtlesim/Spawn`.
   - Users can select which turtle to move, its direction, and speed.
   - Commands are managed via `std::cin` and processed with `switch` statements.
   - Velocity are updated in real-time using two `Publisher` objects.

   **Example interaction:**

	Which turtle do you want to move? \ 1 = turtle1 \ 2 = turtle2
	-----------------------------
	How do you want the turtle to move? \ 1 = right \ 2 = left \ 3 = forward \ 4 = backward
	-----------------------------
	At what speed do you want the turtle to move?
	-----------------------------
  
- **Feature 2: Safety controls (`node2.cpp`)**

  - *Workspace boundaries:* If a turtle gets too close to the edges, its speed is stopped.
  - *Minimum distance:* If the distance between the turtles is less than 2 units, both stop moving.
  - Positions and velocity are updated in real-time using `Subscriber` and `Publisher` objects.

## Technologies 

- **Languages:** C++
- **Framework:** ROS (Robot Operating System)
- **Additional tools:** Docker, GitHub
- **Used packages:**
- `turtlesim`: to simulate the turtles and the workspace.
- `geometry_msgs`: to handle velocity messages.
- `roscpp`: to implement ROS nodes in C++.
- `std_msgs`: handles generic messages, such as the distance between the turtles.

## Installation

Follow these steps to set up the project:

1. Clone the repository:

	- *git clone https://github.com/CaroP-unige/assignment1_rt.git*
	- *cd assignment1_rt*

2. Build the project: *catkin_make*

3. Launch processes in separate terminals:

	- **Terminal 1**: Start ROS: *roscore*
	- **Terminal 2**: Launch the turtlesim simulator: *rosrun turtlesim turtlesim_node*
	- **Terminal 3**: Launch node1: *rosrun turtlesim node1*
	- **Terminal 4**: Launch node2: *rosrun turtlesim node2*

## Usage

This project is ideal for beginners who want to learn ROS by exploring basic concepts such as:

Publisher and Subscriber communication.
User-terminal interaction.
Collision management in a simulated environment.
