#!/usr/bin/env python3

"""
Test script showing how to build a world and use it with pyrobosim
"""
import os
import argparse
import numpy as np

from robot import Robot
from world import World
from yaml_utils import WorldYamlLoader
from main import start_gui
from grasping import GraspGenerator, ParallelGraspProperties
from execution import ConstantVelocityExecutor
from path_planner import PathPlanner
from general import get_data_folder
from pose import Pose


data_folder = get_data_folder()


def create_world(multirobot=False):
    """Create a test world"""
    world = World()

    # Set the location and object metadata
    world.set_metadata(
        locations=os.path.join(data_folder, "example_location_data.yaml"),
        objects=os.path.join(data_folder, "example_object_data.yaml"),
    )

    # Add rooms
    r1_xshift = -2.7
    r1_yshift = -3.5
    r1coords = [(-1 + r1_xshift, -1 + r1_yshift), (1.5 + r1_xshift, -1 + r1_yshift), (1.5 + r1_xshift, 1.5 + r1_yshift), (0.5 + r1_xshift, 1.5 + r1_yshift)]

    world.add_room(
        name="kitchen",
        footprint=r1coords,
        color=[1, 0, 0],
        nav_poses=[Pose(x=0.75, y=0.75, z=0.0, yaw=0.0)],
    )
    r2coords = [(1.75, 2.5), (3.5, 2.5), (3.5, 4), (1.75, 4)]
    world.add_room(name="bedroom", footprint=r2coords, color=[0, 0.6, 0])
    r3coords = [(-1, 1), (-1, 3.5), (-3.0, 3.5), (-2.5, 1)]
    world.add_room(name="bathroom", footprint=r3coords, color=[0, 0, 0.6])

    # Add hallways between the rooms
    world.add_hallway(room_start="kitchen", room_end="bathroom", width=1.2)
    
    world.add_hallway(
        room_start="bathroom",
        room_end="bedroom",
        width=1.2,
        conn_method="angle",
        conn_angle=0,
        offset=0.8,
    )

    # Add locations

    table = world.add_location(
        category="table",
        parent="kitchen",
        pose=Pose(x=r1coords[0][0]+1.85, y=r1coords[0][1]+0.5, z=0.0, yaw=-np.pi / 2.0),
    )
    desk = world.add_location(
        category="desk", parent="bedroom", pose=Pose(x=3.15, y=3.65, z=0.0, yaw=0.0)
    )
    counter = world.add_location(
        category="counter",
        parent="bathroom",
        pose=Pose(x=-2.45, y=2.5, z=0.0, q=[0.634411, 0.0, 0.0, 0.7729959]),
    )

    # Add objects
    world.add_object(
        category="banana",
        parent=table,
    )
    world.add_object(
        category="apple", parent=desk
    )
    world.add_object(category="apple", parent=table)
    world.add_object(category="apple", parent=table)
    world.add_object(category="water", parent=counter)
    world.add_object(category="banana", parent=counter)
    world.add_object(category="water", parent=desk)

    # Add robots
    grasp_props = ParallelGraspProperties(
        max_width=0.175,
        depth=0.1,
        height=0.04,
        width_clearance=0.01,
        depth_clearance=0.01,
    )

    robot0 = Robot(
        name="robot0",
        radius=0.1,
        path_executor=ConstantVelocityExecutor(
            linear_velocity=1.0, dt=0.1, max_angular_velocity=4.0
        ),
        grasp_generator=GraspGenerator(grasp_props),
    )
    planner_config_rrt = {
        "world": world,
        #"bidirectional": False,
        "rrt_connect": False,
        "rrt_star": True,
        "collision_check_step_dist": 0.025,
        "max_connection_dist": 1,
        "rewire_radius": 10,
        "max_nodes_sampled": 20000,
        "max_time": 120
        #"compress_path": False,
    }
    rrt_planner = PathPlanner("rrt", **planner_config_rrt)
    robot0.set_path_planner(rrt_planner)
    world.add_robot(robot0, loc="kitchen", pose=Pose(r1_xshift, r1_yshift))

    return world


def create_world_from_yaml(world_file):
    return WorldYamlLoader().from_yaml(os.path.join(data_folder, world_file))


def parse_args():
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
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    # Create a world or load it from file.
    if args.world_file == "":
        world = create_world(args.multirobot)
    else:
        world = create_world_from_yaml(args.world_file)

    # Scripted order of actions to perform
        # Actions can be of type 'Navigate', 'Pick', and 'Place'
        # Corresponding object must be associated with the action type
    action_list = [('Navigate', 'bedroom')]

    # Start the program either as ROS node or standalone.
    start_gui(world, action_list)
