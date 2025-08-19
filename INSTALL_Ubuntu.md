# Ubuntu Install

The project will use Ubuntu 22.04 (Jammy JellyFish) and ROS2 Humble as a starting point. We will use a Docker container on Ubuntu for a virtual Barney environment. This will allow you to run other projects that require different versions of ROS, python, etc.

## Docker Setup

Install Docker Desktop for Ubuntu.

See https://docs.docker.com/get-started/get-docker/

## ROS2 Humble Setup

Install ROS2 Humble for Ubuntu.
See https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html for details.

When the ROS2 Humble install completes, run the following command.

```bash
source /opt/ros/humble/setup.bash
```

## Build Docker Image

Clone the Barney GitHub repository and build the Docker image with the following commands.

```bash
git clone https://github.com/BainbridgeArtisanResourceNetwork/Barney.git
cd Barney/src
docker compose build
```

## Run Docker image
When the build completes, we will start the image in one terminal with this command.

```bash
docker compose run --name pyrobosim --remove-orphans base
```

The last command will not return until you type Ctrl-C. On another terminal, enter this command to start an interactive shell for the Docker container you started in the other terminal.

```bash
docker exec -it pyrobosim bash
```

The last two commands are what you will need to run on Ubuntu in the future. You start the Docker container in the first terminal (leave it running) and then start a shell for this container in a second terminal. By doing this, you keep the host Ubuntu image clean, allowing you to run different versions of Python, ROS2, or any other software.

Go back to the Usage section of [README](./README.md) file for instructions on how to start the PyRoboSim simulator. Note that you may run into an issue with the ALSO audio device. Specifically, you may get an error similar to the following.

```bash
pygame.error: ALSA: Couldn't open audio device: No such file or directory
```

To fix this problem, you will need to issue the following command.

```bash
cp asoundrc ~/.asoundrc
```

If you still enounter errors, post the error in slack and ask for additional steps that may be needed to fix the issue.

## Docker image management

Here are the Docker commands to push a new Docker image:

```bash
docker login
docker tag barney:humble bainbridgebarn/barney:humble
docker push bainbridgebarn/barney:humble
```

Here are Docker commands to cleanup the system - delete unused containers, images, and system data. Use the last one sparingly - as it will cause Docker builds to take much longer.

```bash
docker image prune
docker container prune
docker system -a prune
```
