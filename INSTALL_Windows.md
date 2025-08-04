# Windows Install

The project will use Ubuntu 22.04 (Jammy JellyFish) and ROS2 Humble as a starting point. In order to run Ubuntu on Windows, we will use WSL2 (Windows Subsystem for Linux). Our 2D simulator, PyRoboSim, will run under Ubuntu. We will use a Python virtual environment for Windows, but will not use Docker as WSL provides most of the Docker benefit.

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
source ./setup/setup_pyrobosim.bash
source ./setup/source_pyrobosim.bash
```

You will likely get an error on the last command. This is typically due to an incorrect version of empy being installed. While still in the virtual environment, use the commands below to fix this error.

```bash
pip uninstall empy
pip install empy==3.3.4
```

Now we have to rebuild the ROS2 Workspace with the commands below.

```bash
colcon build
source install/local_setup.bash
```

Go back to the Usage section of [README](./README.md) file for instructions on how to start PyRoboSim.
