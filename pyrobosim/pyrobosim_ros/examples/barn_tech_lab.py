#!/usr/bin/env python3

"""
Barney RAMP simulation of BARN Tech Lab using PyRobosim,
starting up a ROS interface.
"""
import os
import rclpy
import threading

from pyrobosim.core import Robot, World, WorldYamlLoader
from pyrobosim.gui import start_gui
from pyrobosim.navigation.execution import ConstantVelocityExecutor
from pyrobosim.navigation.rrt import RRTPlanner
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose
from pyrobosim_ros.ros_interface import WorldROSWrapper

data_folder = get_data_folder()

def create_world() -> World:
    """Create a simulation world"""
    world = World()

    # Set the location and object metadata
    world.add_metadata(
        locations=[
            os.path.join(data_folder, "barn_location_data_furniture.yaml"),
            os.path.join(data_folder, "barn_location_data_accessories.yaml"),
        ],
        objects=[
            os.path.join(data_folder, "barn_object_data_food.yaml"),
            os.path.join(data_folder, "barn_object_data_drink.yaml"),
        ],
    )

    # Add rooms
    r1coords = [(0, 0), (1.5, 0), (1.5, 2.0), (2.5, 2.0), (2.5, 4.0), (0, 4.0)]
    world.add_room(
        name="tech_lab",
        pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r1coords,
        color="blue",
        wall_width=0.05,
        nav_poses=[Pose(x=0.75, y=0.75, z=0.0, yaw=0.0)],
    )

    r2coords = [(2.6, 2.0), (4.0, 2.0), (4.0, 4.0), (2.6, 4.0)]
    world.add_room(
        name="laser_lab",
        footprint=r2coords,
        wall_width=0.05,
        color="red",
    )
    
    # Add locations
    dock_pose = world.get_pose_relative_to(
        Pose(x=0.45, y=0.2, z=0.0, yaw=0.0), "tech_lab"
    )
    dock = world.add_location(
        category="desk",
        name="dock",
        parent="tech_lab",
        pose=dock_pose,
    )

    print3d_pose = world.get_pose_relative_to(
        Pose(x=0.45, y=0.9, z=0.0, yaw=0.0), "tech_lab"
    )
    print3d = world.add_location(
        category="desk",
        name="3dprinters",
        parent="tech_lab",
        pose=print3d_pose,
    )

    desktops_pose = world.get_pose_relative_to(
        Pose(x=0.45, y=1.6, z=0.0, yaw=0.0), "tech_lab"
    )
    desktops = world.add_location(
        category="desk",
        name="desktops",
        parent="tech_lab",
        pose=desktops_pose,
    )

    class1 = world.add_location(
        category="table",
        name="class",
        parent="tech_lab",
        pose=Pose(x=1.0, y=3.2, z=0.0, yaw=0, angle_units="degrees"),
    )

    cricut = world.add_location(
        category="counter",
        name="cricut",
        parent="tech_lab",
        pose=Pose(x=2.3, y=3.1, z=0.0, yaw=90, angle_units="degrees"),
    )

    blue = world.add_location(
        category="counter",
        name="blue",
        parent="laser_lab",
        pose=Pose(x=3.4, y=2.2, z=0.0, yaw=0, angle_units="degrees"),
    )

    red = world.add_location(
        category="counter",
        name="red",
        parent="laser_lab",
        pose=Pose(x=3.8, y=2.6, z=0.0, yaw=90, angle_units="degrees"),
    )
    
    # Add doorways / hallways
    world.add_hallway(
        room_start="tech_lab", 
        room_end="laser_lab", 
        width=.5, 
        color="dimgray",
        wall_width=0.05,
        conn_method="points",
        conn_points=[(2.2, 2.6), (2.8, 2.6)],
    )
    
    # Add objects
    # world.add_object(category="water", parent=dock)
    # world.add_object(category="water", parent=class)
    
    # Add a robot
    # Create path planner
    planner_config = {
        "bidirectional": True,
        "rrt_connect": False,
        "rrt_star": True,
        "collision_check_step_dist": 0.025,
        "max_connection_dist": 0.5,
        "rewire_radius": 1.5,
        "compress_path": False,
    }
    path_planner = RRTPlanner(**planner_config)
    robot = Robot(
        name="robot",
        radius=0.1,
        path_executor=ConstantVelocityExecutor(),
        path_planner=path_planner,
    )
    world.add_robot(robot, loc="tech_lab")

    return world


def create_world_from_yaml(world_file: str) -> World:
    return WorldYamlLoader().from_file(os.path.join(data_folder, world_file))


def create_ros_node() -> WorldROSWrapper:
    """Initializes ROS node"""
    rclpy.init()
    node = WorldROSWrapper(state_pub_rate=0.1, dynamics_rate=0.01)
    node.declare_parameter("world_file", value="")

    # Set the world
    world_file = node.get_parameter("world_file").get_parameter_value().string_value
    if world_file == "":
        node.get_logger().info("Creating demo world programmatically.")
        world = create_world()
    else:
        node.get_logger().info(f"Using world file {world_file}.")
        world = create_world_from_yaml(world_file)

    node.set_world(world)

    return node


if __name__ == "__main__":
    node = create_ros_node()

    # Start ROS node in separate thread
    ros_thread = threading.Thread(target=lambda: node.start(wait_for_gui=True))
    ros_thread.start()

    # Start GUI in main thread
    start_gui(node.world)
