# Kinematic Problem Solver for 6-Axis Collaborative Robot

This repository contains a solution for the kinematic inverse problem of a 6-axis collaborative robot's DH-model. The goal of this project is to calculate the joint angles required to achieve a specific end-effector position in the Cartesian coordinate system.

## Table of Contents
- [Kinematic Problem Solver for 6-Axis Collaborative Robot](#kinematic-problem-solver-for-6-axis-collaborative-robot)
  - [Table of Contents](#table-of-contents)
  - [Description](#description)
  - [Algorithm](#algorithm)
  - [Compilation and Execution](#compilation-and-execution)
  - [Requirements](#requirements)
  - [Usage](#usage)
  - [Results](#results)
  - [LICENSE](#license)

## Description
The project involves implementing an algorithm to solve the kinematics problem for a DH-model 6-axis collaborative robot. The goal is to find the joint angles that correspond to a given end-effector position. The DH parameters for the robot's joints are provided in the table below:

| Joint    | theta (deg) | a (m) | d (m)      | alpha (rad) |
|----------|-------------|-------|------------|-------------|
| Joint 0  | 10          | 0     | 0.21       | π/2         |
| Joint 1  | -50         | -0.8  | 0.193      | 0           |
| Joint 2  | -60         | -0.598| -0.16      | 0           |
| Joint 3  | 90          | 0     | 0.25       | π/2         |
| Joint 4  | 50          | 0     | 0.25       | -π/2        |
| Joint 5  | 0           | 0     | 0.25       | 0           |

## Algorithm

The kinematics problem is solved using a mathematical algorithm that iteratively computes the joint angles corresponding to the desired end-effector position. The algorithm involves calculating trigonometric functions and matrix operations based on the provided DH parameters.

## Compilation and Execution

The project can be compiled and executed on a Linux operating system. The recommended programming languages for implementation are C for console applications and C++ with the QT library for GUI applications.

To compile and run the console application:

```bash
make clean && make
make run
```

## Requirements

- GCC (for C implementation)

## Usage

1. Run the compiled executable.
2. Input the desired end-effector coordinates (x, y, z) in the Cartesian coordinate system.
3. The algorithm will calculate and display the corresponding joint angles (theta) for the robot.

## Results

The results of the implementation, along with the source code, should be uploaded to a GitHub repository.

## LICENSE
This project is licensed under the [MIT LICENSE](LICENSE).
