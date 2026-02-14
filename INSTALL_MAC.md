# MAC Install

The project will use Ubuntu 22.04 (Jammy JellyFish) and ROS2 Humble as a starting point. In order to run Ubuntu on a MAC, we will download a Docker image. Our 2D simulator, PyRoboSim, will run under Ubuntu in the Docker image.

## Docker Install

Install Docker Desktop for MAC.

See https://docs.docker.com/get-started/get-docker/

## Docker Setup

Pull the Docker Barney image for ROS2 Humble with the following command (this may take a while).

```bash
docker pull bainbridgebarn/barney:humble
```

Now tag the image to make it easier to use

```bash
docker tag bainbridgebarn/barney:humble barney:humble
```

## X11 Fix for MAC (not needed for WSL)

You will likely get an X11 server error for the PyRoboSim GUI. You can follow these instructions to enable X11 forwarding on your MAC:

https://gist.github.com/sorny/969fe55d85c9b0035b0109a31cbcb088

In the XTerm that pops up when XQuartz is started, type this command.

```bash
xhost +
```

Below is the command to run the Docker image.

```bash
docker run -it -e DISPLAY=host.docker.internal:0 --name pyrobosim barney:humble /bin/bash
```

Note: WSL users can omit the DISPLAY flag - X11 should work without any additional changes.

Use the command above to start an interactive shell for your Docker image. On a MAC, make sure that X Windows / XQuartz is started before running the command.

Go back to the Usage section of [README](./README.md) file for instructions on how to start the PyRoboSim simulator.
