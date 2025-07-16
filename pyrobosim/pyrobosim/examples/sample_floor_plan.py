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
            os.path.join(data_folder, "example_location_data_furniture.yaml"),
            os.path.join(data_folder, "example_location_data_accessories.yaml"),
        ],
        objects=[
            os.path.join(data_folder, "example_object_data_food.yaml"),
            os.path.join(data_folder, "example_object_data_drink.yaml"),
        ],
    )

        # Add rooms

    r1coords = [(0, 0), (5.3, 0), (5.3, 3.73), (0, 3.73)]
    world.add_room(
        name="Office_1",
        #pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r1coords,
        color="green",
        wall_width=0.1,
    )

    r2coords = [(5.5, 0), (10.8, 0), (10.8, 3.73), (5.5, 3.73)]
    world.add_room(
        name="Office_2",
        #pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r2coords,
        color="blue",
        wall_width=0.1,
        nav_poses=[Pose(x=0.75, y=0.75, z=0.0, yaw=0.0)],
    )

    r3coords = [(11, 0), (16.3, 0), (16.3, 3.73), (11, 3.73)]
    world.add_room(
        name="Office_3",
        #pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r3coords,
        color="green",
        wall_width=0.1,
    )

    r4coords = [(16.5, 0), (21.8, 0), (21.8, 3.73), (16.5, 3.73)]
    world.add_room(
        name="Office_4",
        #pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r4coords,
        color="blue",
        wall_width=0.1,
    )

    r5coords = [(22, 0), (27.3, 0), (27.3, 3.73), (22, 3.73)]
    world.add_room(
        name="Office_5",
        #pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r5coords,
        color="green",
        wall_width=0.1,
    )

    r6coords = [(27.5, 0), (32.8, 0), (32.8, 3.73), (27.5, 3.73)]
    world.add_room(
        name="Office_6",
        #pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r6coords,
        color="blue",
        wall_width=0.1,
    )

    r7coords = [(33, 6.02), (38.67, 6.02), (38.67, 8.51), (33, 8.51)]
    world.add_room(
        name="Office_7",
        #pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r7coords,
        color="green",
        wall_width=0.1,
    )

    r8coords = [(33, 8.71), (38.67, 8.71), (38.67, 15.05), (33, 15.05)]
    world.add_room(
        name="Office_8",
        #pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r8coords,
        color="blue",
        wall_width=0.1,
    )

    r9coords = [(5.5, 6.02), (10.8, 6.02), (10.8, 9.27), (5.5, 9.27)]
    world.add_room(
        name="Sensor_Lab",
        #pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r9coords,
        color="red",
        wall_width=0.1,
    )


    r10coords = [(13.34, 6.02), (16.3, 6.02), (16.3, 9.27), (13.34, 9.27)]
    world.add_room(
        name="Lager",
        #pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r10coords,
        color="red",
        wall_width=0.1,
    )

    r11coords = [(13.34, 9.47), (16.3, 9.47), (16.3, 11.81), (13.34, 11.81)]
    world.add_room(
        name="Drucker",
        #pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r11coords,
        color="red",
        wall_width=0.1,
    )

    r12coords = [(16.5, 6.02), (21.8, 6.02), (21.8, 9.27), (16.5, 9.27)]
    world.add_room(
        name="WC",
        #pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r12coords,
        color="red",
        wall_width=0.1,
    )

    r12coords = [(16.5, 9.47), (21.8, 9.47), (21.8, 11.81), (16.5, 11.81)]
    world.add_room(
        name="Server",
        #pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r12coords,
        color="red",
        wall_width=0.1,
    )

    ######################## OpenSpaces ########################################

    r13coords = [(0, 3.93), (5.3, 3.93), (5.3, 15.05), (0, 15.05)]
    world.add_room(
        name="OpenSpace_1",
        #pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r13coords,
        color="grey",
        wall_width=0.1,
    )

    r14coords = [(5.5, 9.47), (13.14, 9.47), (13.14, 15.05), (5.5, 15.05), (5.5, 9.47)]
    world.add_room(
        name="OpenSpace_2",
        #pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r14coords,
        color="grey",
        wall_width=0.1,
    )

    r15coords = [(13.34, 12.01), (21.8, 12.01), (21.8, 15.05), (13.34, 15.05), (13.34, 12.01)]
    world.add_room(
        name="OpenSpace_3",
        #pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r15coords,
        color="grey",
        wall_width=0.1,
    )

    r16coords = [(11, 6.02), (13.14, 6.02), (13.14, 9.27), (11, 9.27)]
    world.add_room(
        name="OpenSpace_4",
        #pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r16coords,
        color="grey",
        wall_width=0.1,
    )

    r17coords = [(5.5, 3.93), (32.8, 3.93), (32.8, 5.82), (5.5, 5.82)]
    world.add_room(
        name="OpenSpace_5",
        #pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r17coords,
        color="grey",
        wall_width=0.1,
    )

    r18coords = [(33, 0), (38.67, 0), (38.67, 5.82), (33, 5.82)]
    world.add_room(
        name="OpenSpace_6",
        #pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r18coords,
        color="grey",
        wall_width=0.1,
    )

    r19coords = [(27.5, 6.02), (32.8, 6.02), (32.8, 15.05), (27.5, 15.05)]
    world.add_room(
        name="OpenSpace_7",
        #pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r19coords,
        color="grey",
        wall_width=0.1,
    )

    r20coords = [(22, 6.02), (27.3, 6.02), (27.3, 8), (22, 8)]
    world.add_room(
        name="Entrance",
        #pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r20coords,
        color="grey",
        wall_width=0.1,
    )

    ################## Connect Hallways #################

    world.add_hallway(
        room_start="OpenSpace_1", 
        room_end="OpenSpace_2", 
        width=5.58, 
        color="dimgray",
        wall_width=0.1,
        conn_method="points",
        conn_points=[(5, 12.26), (5.5, 12.26), (6, 12.26)],
    )

    world.add_hallway(
        room_start="OpenSpace_1", 
        room_end="OpenSpace_5", 
        width=1.89, 
        color="dimgray",
        wall_width=0.1,
        conn_method="points",
        conn_points=[(5, 4.875), (6, 4.875), (7, 4.875)],
    )
        
    world.add_hallway(
        room_start="OpenSpace_2", 
        room_end="OpenSpace_4", 
        width=2.14, 
        color="dimgray",
        wall_width=0.1,
        conn_method="points",
        conn_points=[(12.07, 9), (12.07, 9.5), (12.07, 10)],
    )

    world.add_hallway(
        room_start="OpenSpace_2", 
        room_end="OpenSpace_3", 
        width=3.04, 
        color="dimgray",
        wall_width=0.1,
        conn_method="points",
        conn_points=[(12, 13.53), (13, 13.53), (14, 13.53)],
    )

    world.add_hallway(
        room_start="OpenSpace_5", 
        room_end="OpenSpace_4", 
        width=2.14, 
        color="dimgray",
        wall_width=0.1,
        conn_method="points",
        conn_points=[(12.07, 5), (12.07, 6), (12.07, 7)],
    )

    world.add_hallway(
        room_start="OpenSpace_5", 
        room_end="OpenSpace_6", 
        width=1.89, 
        color="dimgray",
        wall_width=0.1,
        conn_method="points",
        conn_points=[(32, 4.875), (33, 4.875), (34, 4.875)],
    )

    world.add_hallway(
        room_start="OpenSpace_5", 
        room_end="OpenSpace_7", 
        width=5.3, 
        color="dimgray",
        wall_width=0.1,
        conn_method="points",
        conn_points=[(30.15, 5), (30.15, 6), (30.15, 7)],
    )
    

    ############################ Doors ############################
    
    world.add_hallway(
        room_start="Entrance", 
        room_end="OpenSpace_5", 
        width=1.1, 
        color="dimgray",
        wall_width=0.1,
        conn_method="points",
        conn_points=[(23, 5), (23, 6), (23, 7)],
    )

    world.add_hallway(
        room_start="OpenSpace_1", 
        room_end="Office_1", 
        width=1.1, 
        color="dimgray",
        wall_width=0.1,
        conn_method="points",
        conn_points=[(4.3, 3), (4.3, 4)],
    )

    world.add_hallway(
        room_start="OpenSpace_5", 
        room_end="Office_2", 
        width=1.1, 
        color="dimgray",
        wall_width=0.1,
        conn_method="points",
        conn_points=[(9.8, 3), (9.8, 4)],
    )

    world.add_hallway(
        room_start="OpenSpace_5", 
        room_end="Office_3", 
        width=1.1, 
        color="dimgray",
        wall_width=0.1,
        conn_method="points",
        conn_points=[(12, 3), (12, 4)],
    )

    world.add_hallway(
        room_start="OpenSpace_5", 
        room_end="Office_4", 
        width=1.1, 
        color="dimgray",
        wall_width=0.1,
        conn_method="points",
        conn_points=[(20.8, 3), (20.8, 4)],
    )
    
    world.add_hallway(
        room_start="OpenSpace_5", 
        room_end="Office_5", 
        width=1.1, 
        color="dimgray",
        wall_width=0.1,
        conn_method="points",
        conn_points=[(26.75, 3), (26.75, 4)],
    )

    world.add_hallway(
        room_start="OpenSpace_5", 
        room_end="Office_6", 
        width=1.1, 
        color="dimgray",
        wall_width=0.1,
        conn_method="points",
        conn_points=[(28.05, 3), (28.05, 4)],
    )

    world.add_hallway(
        room_start="OpenSpace_7", 
        room_end="Office_7", 
        width=1.1, 
        color="dimgray",
        wall_width=0.1,
        conn_method="points",
        conn_points=[(32, 7.02), (34, 7.02)],
    )

    world.add_hallway(
        room_start="OpenSpace_7", 
        room_end="Office_8", 
        width=1.1, 
        color="dimgray",
        wall_width=0.1,
        conn_method="points",
        conn_points=[(32, 10), (34, 10)],
    )

    world.add_hallway(
        room_start="OpenSpace_5", 
        room_end="Sensor_Lab", 
        width=1.1, 
        color="dimgray",
        wall_width=0.1,
        conn_method="points",
        conn_points=[(9.8, 5), (9.8, 6.5)],
    )

    world.add_hallway(
        room_start="OpenSpace_5", 
        room_end="WC", 
        width=1.1, 
        color="dimgray",
        wall_width=0.1,
        conn_method="points",
        conn_points=[(20.8, 5), (20.8, 6.5)],
    )

    world.add_hallway(
        room_start="OpenSpace_4", 
        room_end="Lager", 
        width=1.1, 
        color="dimgray",
        wall_width=0.1,
        conn_method="points",
        conn_points=[(13, 7.1), (14, 7.1)],
    )

    world.add_hallway(
        room_start="OpenSpace_3", 
        room_end="Drucker", 
        width=1.1, 
        color="dimgray",
        wall_width=0.1,
        conn_method="points",
        conn_points=[(14.34, 11), (14.34, 12.5)],
    )

    world.add_hallway(
        room_start="OpenSpace_3", 
        room_end="Server", 
        width=1.1, 
        color="dimgray",
        wall_width=0.1,
        conn_method="points",
        conn_points=[(20.8, 11), (20.8, 12.5)],
    )

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
    world.add_robot(robot0, loc="OpenSpace_1")

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
        world.add_robot(robot1, loc="OpenSpace_3")

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
        world.add_robot(robot2, loc="OpenSpace_5")

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
