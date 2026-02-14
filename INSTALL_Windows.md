# Windows Install

The project will use Ubuntu 22.04 (Jammy JellyFish) and ROS2 Humble as a starting point. In order to run Ubuntu on Windows, we will use WSL2 (Windows Subsystem for Linux). Our 2D simulator, PyRoboSim, will run under Ubuntu. We will use a Python virtual environment for Windows, but will not use Docker as WSL provides most of the Docker benefit. You can stil use Docker if you want - see instructions at the bottom of this file.

## WSL2 Setup

Start up a Windows PowerShell session. You install Ubuntu 22.04 on WSL with the command below. This may ask you to set up a root user and password.

```bash
wsl --install -d Ubuntu-22.04
```

After the install, You can start a session with the Ubuntu 22.04 distribution with the command below. 

```bash
wsl -d Ubuntu-22.04
```

## ROS2 Humble Setup

Since you are now running under Ubuntu with WSL, you want to install ROS2 Humble for Ubuntu.
See https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html for details.


## PyRoboSim Setup

Once you complete the install for ROS2 Humble, run the commands below.

```bash
source /opt/ros/humble/setup.bash
cd
git clone https://github.com/BainbridgeArtisanResourceNetwork/Barney.git
cd ~/Barney/src
sudo apt install python3-venv
```

The PyRoboSim setup will ask whether you want to install ROS (yes) and PDDL (no) when you run this command. For the ROS home directory, enter /home/user/Barney where 'user' is the account your created when you setup the WSL Ubuntu distribution.

```bash
source ./setup/setup_pyrobosim.bash
```

Now you have to setup the virtual environment. Use these commands to fix a bug in the virtual environment with empy.

```bash
source ./setup/source_pyrobosim.bash
pip uninstall empy
pip install empy==3.3.4
source ./setup/source_pyrobosim.bash
```

Next, rebuild the ROS2 Workspace with the commands below.

```bash
cd ~/Barney
sudo rosdep init
rosdep update
colcon build
source install/local_setup.bash
```

Now, to run on Windows, you need only issue the following commands in the WSL Ubuntu distribution.

```bash
cd ~/Barney
source src/setup/source_pyrobosim.bash
```

Go back to the Usage section of [README](./README.md) file for instructions on how to start the PyRoboSim simulator.

## Docker information

You can add Docker support to WSL.

In Docker Desktop, go to Settings, then Resources. Then, click on WSL integration tab, and turn on integration for Ubuntu-22.04 (or whatever name you gave your Barney distribution). Leave the 'Enable integration with my default WSL distro' on as well. Start your WSL distribution and you should have all the Docker commands.

To run under Docker in the WSL image, following the instructions starting at the Docker Setup section of the [INSTALL_MAC](./INSTALL_MAC.md) file.
