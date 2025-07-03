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

We are going to start the Barney project by simulating a greeting robot in PyRoboSim.

PyRoboSim is a lightweight 2D mobile robot simulator for behaviour prototyping. We use PyRoboSim to create a custom world of each of the BARN building floors using a BARN floor plan. It includes defining the office rooms and studios, hallways, some locations and objects from pick-and-place tasks. We use the PyRoboSim demo robot. To control the robots, we can use the GUI, python scripts or ROS actions/services.

## Demo

- **TBD**

## Features

- **TBD**

## Project Structure

### 📁 Project Structure

```plaintext
Barney/                               # Barney project Root
├── pyrobosim/                        # PyRoboSim git subtree
├── LICENSE.md                        # license
├── README.md                         # README

```

## Prerequisites

This installation assumes that you are running on Ubuntu 22.04 LTS (Jammy Jellyfish) with ROS2 Humble. You can find these installs at:

- https://releases.ubuntu.com/jammy/
- https://docs.ros.org/en/humble/Installation.html

## Installation

First, you must clone this repository

```bash
# Clone the repository
git clone https://github.com/BainbridgeArtisanResourceNetwork/Barney.git

```

Next, you will have to setup PyRoboSim. Change directory to Barney/pyrobosim directory and follow this link to set up pyrobosim: https://pyrobosim.readthedocs.io/en/latest/setup.html

We strongly recommend that you use the Docker Setup steps for PyRoboSim. This will require you to learn and install Docker. See https://docs.docker.com/get-started/
  
## Usage

- **TBD**

## Acknowledgments

Thanks to Sebastian Castro, the owner and maintainer of the PyRoboSim simulator for creaing this useful open-source tool. All rights and license for PyRoboSim used in this project belongs to Sebastian Castro.
