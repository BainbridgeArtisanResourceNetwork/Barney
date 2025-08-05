# MAC Install

The project will use Ubuntu 22.04 (Jammy JellyFish) and ROS2 Humble as a starting point. In order to run Ubuntu on a MAC, we will download a Docker image. Our 2D simulator, PyRoboSim, will run under Ubuntu in the Docker image.

## Docker Setup

Install Docker Desktop for MAC.

See https://docs.docker.com/get-started/get-docker/

Pull the Docker image with the following command (this may take a while).

```bash
docker pull bainbridgebarn/barney:humble
```

Now tag the image to make it easier to use

```bash
docker tag bainbridgebarn/barney:humble barney:humble
```

You will need to get an X11 server error for the PyRoboSim GUI. You can  follow these instructions to enable X11 forwarding on your MAC:

https://gist.github.com/sorny/969fe55d85c9b0035b0109a31cbcb088

In the XTerm that pops up when XQuartz is started, type this command.

```bash
xhost +
```

To run the Barney simulation with the Docker image, we need to get the Docker image ID.
Use this command to get the image ID.

```bash
docker image ls
```

We will also need the IP address of the MAC for X Windows.
You can find this in the Network section in System Settings.

Below is the command to run the Docker image. Replace <IP_ADDR> with your IP address and <IMAGE_ID> with the Docker image ID from above.

```bash
docker run -it -e DISPLAY=<IP_ADDR>:0 --name pyrobosim <IMAGE_ID> /bin/bash
```

Go back to the Usage section of [README](./README.md) file for instructions on how to start PyRoboSim.
