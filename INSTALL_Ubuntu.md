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

## Docker Image Build

Clone the Barney GitHub repository and build the Docker image with the following commands.

```bash
git clone https://github.com/BainbridgeArtisanResourceNetwork/Barney.git
cd Barney
docker compose build
```

## Run Docker image
When the build completes, we will start the image in one terminal with this command.

```bash
docker compose run --name pyrobosim --remove-orphans base
```

The last command will not return until you type Ctrl-C. On another terminal, enter this command to get the Docker container image.

```bash
docker ps
```

Then use this command to start an interactive shell in the Docker container you started above.

```bash
docker exec -it <IMAGE_ID> bash
```

Go back to the Usage section of [README](./README.md) file for instructions on how to start PyRoboSim.
