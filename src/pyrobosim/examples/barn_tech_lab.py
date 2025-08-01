#!/usr/bin/env python3

"""
Test script showing how to build a world and use it with PyRoboSim.
"""
import os
import argparse

from pyrobosim.core.robot import Robot
from pyrobosim.core.world import World
from pyrobosim.core.yaml_utils import WorldYamlLoader
from pyrobosim.gui import start_gui
from pyrobosim.manipulation import GraspGenerator, ParallelGraspProperties
from pyrobosim.navigation.execution import ConstantVelocityExecutor
from pyrobosim.navigation.a_star import AStarPlanner
from pyrobosim.navigation.prm import PRMPlanner
from pyrobosim.navigation.rrt import RRTPlanner
from pyrobosim.sensors.lidar import Lidar2D
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose


data_folder = get_data_folder()


def create_world(multirobot: bool = False) -> World:
    """Create a test world"""
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
    
    # Add robots
    grasp_props = ParallelGraspProperties(
        max_width=0.175,
        depth=0.1,
        height=0.04,
        width_clearance=0.01,
        depth_clearance=0.01,
    )
    lidar = Lidar2D(
        update_rate_s=0.1,
        angle_units="degrees",
        min_angle=-120.0,
        max_angle=120.0,
        angular_resolution=5.0,
        max_range_m=2.0,
    )

    robot0 = Robot(
        name="robot0",
        radius=0.1,
        path_executor=ConstantVelocityExecutor(
            linear_velocity=1.0,
            dt=0.1,
            max_angular_velocity=4.0,
            validate_during_execution=True,
        ),
        sensors={"lidar": lidar} if args.lidar else None,
        grasp_generator=GraspGenerator(grasp_props),
        partial_observability=args.partial_observability,
        color="#CC00CC",
    )
    planner_config_rrt = {
        "bidirectional": True,
        "rrt_connect": False,
        "rrt_star": True,
        "collision_check_step_dist": 0.025,
        "max_connection_dist": 0.5,
        "rewire_radius": 1.5,
        "compress_path": False,
    }
    rrt_planner = RRTPlanner(**planner_config_rrt)
    robot0.set_path_planner(rrt_planner)
    world.add_robot(robot0, loc="tech_lab")

    if multirobot:
        robot1 = Robot(
            name="robot1",
            radius=0.08,
            color=(0.8, 0.8, 0),
            path_executor=ConstantVelocityExecutor(),
            grasp_generator=GraspGenerator(grasp_props),
            partial_observability=args.partial_observability,
        )
        planner_config_prm = {
            "collision_check_step_dist": 0.025,
            "max_connection_dist": 1.5,
            "max_nodes": 100,
            "compress_path": False,
        }
        prm_planner = PRMPlanner(**planner_config_prm)
        robot1.set_path_planner(prm_planner)
        world.add_robot(robot1, loc="laser_lab")

        robot2 = Robot(
            name="robot2",
            radius=0.06,
            color=(0, 0.8, 0.8),
            path_executor=ConstantVelocityExecutor(),
            grasp_generator=GraspGenerator(grasp_props),
            partial_observability=args.partial_observability,
        )
        planner_config_astar = {
            "grid_resolution": 0.05,
            "grid_inflation_radius": 0.15,
            "diagonal_motion": True,
            "heuristic": "euclidean",
        }
        astar_planner = AStarPlanner(**planner_config_astar)
        robot2.set_path_planner(astar_planner)
        world.add_robot(robot2, loc="tech_lab")

    return world

def create_world_from_yaml(world_file: str) -> World:
    return WorldYamlLoader().from_file(os.path.join(data_folder, world_file))


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments"""
    parser = argparse.ArgumentParser(description="Main pyrobosim demo.")
    parser.add_argument(
        "--multirobot",
        action="store_true",
        help="If no YAML file is specified, this option will add "
        "multiple robots to the world defined in this file.",
    )
    parser.add_argument(
        "--world-file",
        default="",
        help="YAML file name (should be in the pyrobosim/data folder). "
        + "If not specified, a world will be created programmatically.",
    )
    parser.add_argument(
        "--partial-observability",
        action="store_true",
        help="If True, robots have partial observability and must detect objects.",
    )
    parser.add_argument(
        "--lidar",
        action="store_true",
        help="If True, adds a lidar sensor to the first robot.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    # Create a world or load it from file.
    if args.world_file == "":
        world = create_world(args.multirobot)
    else:
        world = create_world_from_yaml(args.world_file)

    # Start the program either as ROS node or standalone.
    start_gui(world)
