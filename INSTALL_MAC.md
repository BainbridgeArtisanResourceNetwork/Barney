# MAC Install

The project will use Ubuntu 22.04 (Jammy JellyFish) and ROS2 Humble as a starting point. In order to run Ubuntu on a MAC, we will download a Docker image. Our 2D simulator, PyRoboSim, will run under Ubuntu in the Docker image.

## Docker Setup

Install Docker Desktop for MAC.

See https://docs.docker.com/get-started/get-docker/

Pull the Docker image with the following commands.

```bash
docker pull bainbridgebarn/barney:humble
docker tag bainbridgebarn/barney:humble barney:humble
```

To run the Barney simulation with the Docker image, we need to get the Docker image ID
Use this command to get the image ID.

```bash
docker image ls
```

We will also need the IP address of the MAC for X Windows.
You can find this in the Network section in System Settings.

Below is the command to run the Docker image. Replace <IP_ADDR> with your IP address and <IMAGE_ID> with the Docker image from above.

```bash
docker run -it -e DISPLAY=<IP_ADDR>:0 --name pyrobosim <IMAGE_ID> /bin/bash
```

Go back to the Usage section of [README](./README.md) file for instructions on how to start PyRoboSim.
