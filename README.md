# Assignment 1 - Research Track - Turtle Motions

This project allows users to learn the basics of the ROS language by controlling the movement of two turtles within a virtual workspace. It includes controls to prevent collisions with the workspace edges and between the turtles. Additionally, comments in the code provide further technical details.

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
1. Turtles cannot leave the workspace boundaries.
2. Turtles maintain a minimum safety distance (>= 2 units).

In both cases, the movement is stopped by forcing the speed to zero.

## Features

- **Feature 1: User interaction (`node1.cpp`)**

   - Users can select which turtle to move, its direction, and speed.
   - Commands are managed via `std::cin` and processed with `switch` statements.

   **Example interaction:**

	% Which turtle do you want to move? / 1 = turtle1 / 2 = turtle2
	-----------------------------
	% How do you want the turtle to move? / 1 = right / 2 = left / 3 = forward / 4 = backward
	-----------------------------
	% At what speed do you want the turtle to move?
	-----------------------------
- **Feature 2: Safety controls (`node2.cpp`)**

  - *Workspace boundaries:* If a turtle gets too close to the edges, its speed is stopped.
  - *Minimum distance:* If the distance between the turtles is less than 2 units, both stop moving.
  - Positions are updated in real-time using two `Subscriber` objects.

## Technologies 

- **Languages:** C++
- **Framework:** ROS (Robot Operating System)
- **Additional tools:** Docker, GitHub
- **Used packages:**
- `turtlesim`: to simulate the turtles and the workspace.
- `geometry_msgs`: to handle velocity messages.
- `roscpp`: to implement ROS nodes in C++.

## Installation

Follow these steps to set up the project:

1. Clone the repository:

git clone <repository-URL>
cd <folder-name>

2. Build the project: catkin_make

3. Launch processes in separate terminals:

	- Terminal 1: Start ROS: roscore
	- Terminal 2: Launch the turtlesim simulator: rosrun turtlesim turtlesim_node
	- Terminal 3: Launch node1: rosrun <package-name> node1
	- Terminal 4: Launch node2: rosrun <package-name> node2

## Usage

This project is ideal for beginners who want to learn ROS by exploring basic concepts such as:

Publisher and Subscriber communication.
User-terminal interaction.
Collision management in a simulated environment.
Follow the on-screen instructions in the terminal to control the turtles. For any doubts, refer to the comments in the code.
