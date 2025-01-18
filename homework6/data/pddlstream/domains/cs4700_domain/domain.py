"""
PDDL PLANNING DOMAIN (duplicate of 'pddlstream/domains/01_simple/domain.ppdl' but in python)

This planning domain contains `navigate`, `pick`, and `place` actions.

All actions are symbolic, meaning there are no different types of grasps
or feasibility checks, under the assumption that a downstream planner exists.

Accompanying streams are defined in the `streams.pddl` file.
"""
import warnings
class Fluent:
    def __init__(self, fluent_type, a1, a2 = None):
        self.fluents = {
            "Robot",
            "Obj",
            "Room",
            "Location",
            "HandEmpty",
            "CanMove",
            "Holding",
            "At"
        }

        if fluent_type not in self.fluents:
            warnings.warn(
                f"{fluent_type} is not a supported fluent type.", UserWarning
            )
            return None
        if a2 != None and fluent_type != "At":
            warnings.warn(
                f"{fluent_type} only accepts one argument.", UserWarning
            )
            return None

        self.fluent_type = fluent_type
        self.fluent_arg1 = a1
        self.fluent_arg2 = a2
class State:
    def __init__(self, init_fluents: list[Fluent] = []):
        self.fluents = init_fluents
class Domain:
    def __init__(self, actions = []):
        self.actions = actions

class Action:
    """
    Navigate requires 3 inputs, a robot, a start location, and an end location.
    Pick requires 3 inputs, a robot, an object, and a location.
    Place requires 3 inputes, a robot, an object, and a location.
    """
    def __init__(self, action, args, types):
        self.actions = {
            "Navigate",
            "Pick",
            "Place"
        }
        self.possible_types = {
            "Robot",
            "Obj",
            "Location"
        }

        if action not in self.actions:
            warnings.warn(
                f"{action} is not a supported action.", UserWarning
            )
            return None
        if len(args) != len(types):
            warnings.warn(
                f"There are {len(args)} args and {len(types)} types, there must be as many args as types.", UserWarning
            )
            return None
        for i in types:
            if i not in self.possible_types:
                warnings.warn(
                    f"{i} is not a supported type.", UserWarning
                )
                return None
        if action == "Navigate":
            assert(len(args) == 3)
            assert(type[0] == "Robot")
            assert(type[1] == "Location")
            assert(type[2] == "Location")
        if action == "Pick":
            assert(len(args) == 3)
            assert(type[0] == "Robot")
            assert(type[1] == "Obj")
            assert(type[2] == "Location")
        if action == "Place":
            assert(len(args) == 3)
            assert(type[0] == "Robot")
            assert(type[1] == "Obj")
            assert(type[2] == "Location")
        self.action = action
        self.args = args
        self.type = types

    def do_action(self, state):
        if self.action == "Navigate":
            # Checking preconditions
            assert(Fluent("Robot", self.args[0]) in state)
            assert(Fluent("CanMove", self.args[0]) in state)
            assert(Fluent("Location", self.args[1]) in state)
            assert(Fluent("Location", self.args[2]) in state)
            assert(Fluent("At", self.args[0], self.args[1]) in state)

            #Removing fluents
            state.fluents.remove(Fluent("Robot"))


    def pick(self, state):
(define (domain domain_simple)
  (:requirements :strips :equality)
  (:predicates
    ; Static predicates
    (Robot ?r)          ; Represents the robot
    (Obj ?o)            ; Object representation
    (Room ?r)           ; Room representation
    (Location ?l)       ; Location representation

    ; Fluent predicates
    (HandEmpty ?r)      ; Whether the robot's gripper is empty
    (CanMove ?r)        ; Whether the robot can move (prevents duplicate moves)
    (Holding ?r ?o)     ; Object the robot is holding
    (At ?o ?l)          ; Robot/Object's location, or location's Room
  )

  ; FUNCTIONS : See their descriptions in the stream PDDL file
  (:functions
    (Dist ?l1 ?l2)
    (PickPlaceCost ?l ?o)
  )

  ; ACTIONS
  ; NAVIGATE: Moves the robot from one location to the other
  (:action navigate
    :parameters (?r ?l1 ?l2)
    :precondition (and (Robot ?r)
                       (CanMove ?r)
                       (Location ?l1)
                       (Location ?l2)
                       (At ?r ?l1))
    :effect (and (not (CanMove ?r))
                 (At ?r ?l2) (not (At ?r ?l1))
                 (increase (total-cost) (Dist ?l1 ?l2)))
  )

  ; PICK: Picks up an object from a specified location
  (:action pick
    :parameters (?r ?o ?l)
    :precondition (and (Robot ?r)
                       (Obj ?o)
                       (Location ?l)
                       (not (Room ?l))
                       (HandEmpty ?r)
                       (At ?r ?l)
                       (At ?o ?l))
    :effect (and (Holding ?r ?o) (CanMove ?r)
                 (not (HandEmpty ?r))
                 (not (At ?o ?l))
                 (increase (total-cost) (PickPlaceCost ?l ?o)))
  )

  ; PLACE: Places an object in a specified location
  (:action place
    :parameters (?r ?o ?l)
    :precondition (and (Robot ?r)
                       (Obj ?o)
                       (Location ?l)
                       (not (Room ?l))
                       (At ?r ?l)
                       (not (HandEmpty ?r))
                       (Holding ?r ?o))
    :effect (and (HandEmpty ?r) (CanMove ?r)
                 (At ?o ?l)
                 (not (Holding ?r ?o))
                 (increase (total-cost) (PickPlaceCost ?l ?o)))
  )

)
