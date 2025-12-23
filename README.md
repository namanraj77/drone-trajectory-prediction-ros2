Drone Trajectory Prediction using ROS 2 and Machine Learning

This project demonstrates a machine-learning–based drone trajectory predictor integrated into ROS 2, with real-time inference and live visualization in RViz.

A trained neural network predicts the future trajectory of a drone based on recent pose history, running online as a ROS 2 node.

Project Overview

The system consists of three main components:

Pose Publisher
Publishes a simulated drone pose following a smooth trajectory.

ML Trajectory Predictor
Uses a trained neural network to predict future positions based on recent motion.

RViz Visualization
Displays the current drone pose and predicted future trajectory in real time.

The entire system can be launched using a single ROS 2 launch command.

Features

Real-time ML inference inside a ROS 2 node

Predicts future trajectory from recent pose history

Clean ROS 2 package structure and installation

One-command demo launch

Live visualization in RViz

Easily extendable to Gazebo or real rosbag input

Repository Structure
drone_predictor_clean/
├── drone_predictor_clean/
│   ├── pose_publisher.py        # Simulated drone motion
│   ├── predictor_node.py        # ML-based trajectory predictor
│   └── __init__.py
├── launch/
│   └── demo.launch.py           # One-command demo launch
├── models/
│   └── position_ml_model.pt     # Trained ML model
├── rviz/
│   └── demo.rviz                # RViz configuration
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
└── README.md

Requirements

Ubuntu (tested on 22.04)

ROS 2 Jazzy

Python 3.12

PyTorch (CPU version sufficient)

RViz 2

The workspace must already be built using colcon build.

Running the Demo (One Command)

After sourcing the environment:

source /opt/ros/jazzy/setup.bash
source ~/drone_ws/install/setup.bash


Run the entire system with:

ros2 launch drone_predictor_clean demo.launch.py


This command starts:

the pose publisher

the ML predictor node

RViz with the correct configuration

RViz Visualization

In RViz you will see:

A red arrow showing the current drone pose

A green curve showing the predicted future trajectory

Continuous updates as the drone moves

The fixed frame is set to map.

How the ML Model Works

Input: Last 10 drone positions (x, y)

Output: Next 10 predicted positions (x, y)

Model: Fully connected neural network

Inference: Runs online inside a ROS 2 node

Training: Performed offline using trajectory data

The model is loaded at runtime from the installed package resources.

Useful Runtime Checks

Check input pose rate:

ros2 topic hz /drone/pose_stamped


Check prediction rate:

ros2 topic hz /drone/predicted_path

Demo Summary

This project shows how a trained machine-learning model can be integrated into ROS 2 to perform real-time trajectory prediction, with live visualization and clean system design.

Future Extensions

Replace synthetic pose publisher with Gazebo simulation

Use real rosbag recordings as input

Horizon switching between position-based and velocity-based models

Online error evaluation

Uncertainty visualization

ONNX or TensorRT inference optimization

Status

The system is fully functional and demo-ready.
