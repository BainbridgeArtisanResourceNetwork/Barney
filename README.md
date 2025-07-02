# Barney
Repository for all code related to the BARN Barney greeting robot.

The project will use Ubuntu 22.04 and ROS2 Humble as a starting point.

Table of Contents:

  - [Overview](#overview)
  - [Demo](#demo)
  - [Features](#features)
  - [Project Structure](#projectstructure)
  - [Installation](#installation)
  - [Usage](#usage)
  - [Acknowledgments](#acknowledgments)

## Overview

PyRoboSim is a lightweight 2D mobile robot simulator for behaviour prototyping. In this project, we use PyRoboSim to create a custom world of the BARN building using its floor plan. It includes defining the office rooms and studios, hallways, some locations and objects from pick-and-place tasks and robots (IDOGs). We use the PyRoboSim demo robot. To control the robots, we can use the GUI, python scripts or ROS actions/services.

## Demo

TBD

## Features

- **Build complex worlds** using the world modeling framework, both manually and programmatically.

- **Define custom actions and action executors** (e.g., path planning/following or decision-making algorithms).

- **Design task and motion planners** that go from task specification to an executable plan.

- **Export worlds to Gazebo** to test in a 3D world with more complex sensing and actuation models.


## Project Structure

### 📁 Project Structure

```plaintext
Barney/                               # Project Root
├── src/                              # Main folder for ROS2 files
│   ├── 📂 pyrobosim/                 # Main folder for PyRoboSim
├── LICENSE.md                        # license
├── README.md                         # README

```


## Installation

Prerequisites
Follow this link to install and set up pyrobosim: https://pyrobosim.readthedocs.io/en/latest/setup.html
  
Clone and Build the Package

```bash
# Clone the repository
git clone https://github.com/BainbridgeArtisanResourceNetwork/Barney.git

```

## Usage

TBD

## Acknowledgments

Thanks to Sebastian Castro, the owner and maintainer of the PyRoboSim simulator for creaing this useful open-source tool. All rights and license for PyRoboSim used in this project belongs to Sebastian Castro.
