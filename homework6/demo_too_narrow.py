#!/usr/bin/env python3

"""
Test script showing how to build a world and use it with pyrobosim
"""
import pddl_plan as pddl

from main import start_gui
from general import get_data_folder
from pddl_plan import Action, Fluent, State, Planner
from test_world import create_impossible_world

def q5_answer():
    # Your code here should just be a single return statement that returns the answer to the question, as a string.
    "*** BEGIN YOUR CODE HERE ***"  
    return "DOWNWARD REFINABILITY"
    "*** END YOUR CODE HERE ***"

data_folder = get_data_folder()


def pddl_foundations(robots, rooms, locations, objects, fluents):
    # Initializing actions
    action_transit = pddl.create_action_transit()
    action_orient = pddl.create_action_orient()
    action_recenter = pddl.create_action_recenter()
    action_pick = pddl.create_action_pick()
    action_place = pddl.create_action_place()

    actions = [action_transit, action_orient, action_recenter, action_pick, action_place]

    objects_of_type = {'robot': robots,
                       'room': rooms,
                       'spawn': locations,
                       'object': objects,
                       }

    start_state = State(objects_of_type, fluents)

    return start_state, actions


def world_abstraction(world):
    """
    Turns World object into a high level abstraction of fluents
    :param world:
    :return:
    """
    # Creating lists for variables used in planning
    robots = []
    rooms = []
    locations = []
    objects = []

    # Initial fluents
    start_fluent_list = set()

    # Gathering robot fluents
    robot = world.robots[0]
    robots.append(robot)
    init_loc = robot.location
    robot_pose = robot.get_pose()
    if not init_loc:
        init_loc = world.get_location_from_pose(robot_pose)
    start_fluent_list.add(Fluent('empty', robot))
    start_fluent_list.add(Fluent('robot_in_room', robot, init_loc))

    # Gathering initial fluents from the rest of the world
    for room in world.rooms:
        rooms.append(room)
    for hallway in world.hallways:
        start_fluent_list.add(Fluent('adjacent', hallway.room_start, hallway.room_end))
        start_fluent_list.add(Fluent('adjacent', hallway.room_end, hallway.room_start))
    for loc in world.locations:
        for spawn in loc.children:
            locations.append(spawn)
            start_fluent_list.add(Fluent('spawn_in_room', spawn, loc.parent))

    # Loop through all the objects and their relationships.
    for obj in world.objects:
        objects.append(obj)
        start_fluent_list.add(Fluent("at_spawn", obj, obj.parent))

    return pddl_foundations(robots, rooms, locations, objects, list(start_fluent_list))


def stream_to_action(stream: Action):
    if stream.name == 'transit' or stream.name == 'orient' or stream.name == 'recenter':
        return ('Navigate', stream.args[2].name)
    if stream.name == 'pick':
        return ('Pick', stream.args[1].name)
    if stream.name == 'place':
        return ('Place', stream.args[1].name)


if __name__ == "__main__":
    # Create a world or load it from file.
    world = create_impossible_world()

    start_state, actions = world_abstraction(world)

    goal_fluent_list = [Fluent('at_spawn', world.get_object_by_name('apple0'), world.get_object_spawn_by_name('counter0_left')),
                        Fluent('at_spawn', world.get_object_by_name('banana0'), world.get_object_spawn_by_name('counter0_right')),
                        Fluent('robot_in_room', world.robots[0], world.get_room_by_name('bedroom')),
                        Fluent('empty', world.robots[0]),
                        ]
    print("IMPORTANT ROBOT MISSION: Put the apple and the banana in the bathroom (for some reason)")
    print("THEN, go to the bedroom to await further fruit mischief instructions.")
    print()
    print()
    print("The goal is to achieve all of the following fluents:")
    print(goal_fluent_list)

    print()
    print("Right now, the world is in the following state:")
    print(start_state)

    planner = Planner()

    print()
    print("Action space:")
    for a in actions:
        print(a)

    print()

    plan = planner.solve_action(start_state, goal_fluent_list, actions)
    action_list = []
    for n, a in enumerate(plan):
        print(f"Step {n+1}: {a}")
        action_list.append(stream_to_action(a))

    # ****************************** NOTE: auto_gui is not currently working *******************************************
    start_gui(world, action_list, no_graphics=False, pretty_actions=plan)
