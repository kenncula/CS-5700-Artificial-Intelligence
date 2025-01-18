import itertools
import copy
import hashlib
from util import *


class Fluent:
    def __init__(self, name, *args):
        """
        a Fluent is a *proposition* about the state of the world, and that which fluents are true can change over time. 
        Think of a fluent as a binary flag.
        The arguments should be strings, and can be either variables (which start with a '?') or constants (which do not start with a '?')

        Example:
        >>> Fluent('holding', '?robot', '?object')
                   ^^ name     ^^^^ args
        """
        self.name = name
        self.args = args
        
    def replace(self, replace_dict):  
        """
        Given a dictionary mapping variables to values, 
        return a new fluent fluent that is the same as this one, 
        but with its variables replaced according to the dictionary.

        Example:
        >>> f = Fluent('holding', '?robot', '?object')
        >>> f.replace({'?robot': 'robot0', '?object': 'apple0'})
        Fluent('holding', 'robot0', 'apple0')        
        """
        arg_list = list(self.args)
        for i, arg in enumerate(arg_list):
            if arg[0] == '?':
                arg_list[i] = replace_dict[arg]
        return Fluent(self.name, *arg_list)
    
    def __eq__(self, other):
        return self.name == other.name and self.args == other.args
    
    def __str__(self):
        arguments = ', '.join(map(str,self.args))
        return f"{self.name}({arguments})"
    
    def __repr__(self):
        return str(self)

    def __hash__(self):
        return int(hashlib.md5((str(self.name) + str(self.args)).encode('utf-8')).hexdigest(),16) % 4700
    
    def __lt__(self, other):
        return hash(self) < hash(other)


class Action:
    def __init__(self, name, args, types, pre_condition, post_condition):
        """
        Example:
        
        move_action = Action('move',
                             ['?agent', '?from', '?to'], # args are variables => lifted action
                             ['robot', 'room', 'room'], # types of each argument
                             # precondition: fluents that must be true/false before the action can be executed
                             [('+', Fluent('at', '?agent', '?from'),
                               ('+', Fluent('adjacent', '?from', '?to')),
                               ('-', Fluent('at', '?agent', '?to'))],
                             # postcondition: fluents that will be true/false after the action is executed
                             [('+', Fluent('at', '?agent', '?to')),
                              ('-', Fluent('at', '?agent', '?from'))])        
        """
        self.name = name
        self.args = args
        self.types = types
        self.pre_condition = pre_condition
        self.post_condition = post_condition
        self.args_to_type = {k: v for k, v in zip(args, types)}
        
    @property
    def ground(self):
        """Check that there are no variables in the arguments"""
        return all(not isinstance(arg, str) or arg[0] != '?' for arg in self.args)

    def __str__(self):
        if self.ground:
            arguments = ', '.join(map(str,self.args))
            return f"{self.name}({arguments})"
        else:

            arguments = ', '.join(f"{o} : {t}" for o, t in zip(self.args, self.types))
            pre = '∧'.join(f"{'¬' if sign=='-' else ''}{fluent}" for sign, fluent in self.pre_condition)
            post = '∧'.join(f"{'¬' if sign=='-' else ''}{fluent}" for sign, fluent in self.post_condition)
            return f"{self.name}({arguments}, pre={pre}, post={post})"
    
    def __repr__(self):
        return str(self)

    def grounding(self, state):
        """
        
        """
        action_list = []
        type_map = []
        for type in self.types:
            type_map.append(state.objects_of_type[type])
        for assignment in itertools.product(*type_map):
            replace_dict = {k: v for k, v in zip(self.args, assignment)}
            new_pre_condition = []
            new_post_condition = []
            for sign, fluent in self.pre_condition:
                new_fluent = (sign, fluent.replace(replace_dict))
                new_pre_condition.append(new_fluent)
            for sign, fluent in self.post_condition:
                new_fluent = (sign, fluent.replace(replace_dict))
                new_post_condition.append(new_fluent)
            new_action = Action(self.name, assignment, self.types, new_pre_condition, new_post_condition)
            action_list.append(new_action)
        return action_list

    def check_pre_condition(self, state):
        for sign, fluent in self.pre_condition:
            if sign == '+' and fluent not in state.fluent_list:
                return False
            if sign == '-' and fluent in state.fluent_list:
                return False
        return True

    def apply_post_condition(self, state):
        new_fluent_list = copy.copy(state.fluent_list)
        for sign, fluent in self.post_condition:
            if sign == '+':
                new_fluent_list.append(fluent)
            if sign == '-':
                new_fluent_list.remove(fluent)
        return State(state.objects_of_type, new_fluent_list)

# Example code to help you understand the action class and how it relates to the fluents for this robot problem
def create_action_orient():
    action_orient = Action(
        name='orient',
        args=['?r', '?from', '?to'],
        types=['robot', 'room', 'spawn'],
        pre_condition=[('+', Fluent('robot_in_room', '?r', '?from')),
                       ('+', Fluent('spawn_in_room', '?to', '?from')),
                       ('-', Fluent('at_spawn', '?r', '?to')),
                       ],
        post_condition=[('+', Fluent('at_spawn', '?r', '?to')),
                        ('+', Fluent('oriented', '?r')),
                        ],
    )
    return action_orient

# You need to fill in the following actions.
def create_action_transit():
    """
    When not oriented, move
    robot from room to adjacent
    room
    """
    "*** BEGIN YOUR CODE HERE ***"    
    action_transit = Action(
        name='transit',
        args=['?r', '?from', '?to'],
        types=['robot', 'room', 'room'],
        pre_condition=[('+', Fluent('robot_in_room', '?r', '?from')),
                       ('+', Fluent('adjacent', '?from', '?to')),
                       ('-', Fluent('oriented', '?r')),
                       ],
        post_condition=[('+', Fluent('robot_in_room', '?r', '?to')),
                        ('-', Fluent('robot_in_room', '?r', '?from')),
                        ],
    )
    return action_transit
    "*** END YOUR CODE HERE ***"


def create_action_recenter():
    """
    If the robot is oriented at a
    spawn location in a room,
    unorient by moving to the
    middle of the room
    """
    "*** BEGIN YOUR CODE HERE ***"
    action_recenter = Action(
        name='recenter',
        args=['?r', '?spawn', '?room'],
        types=['robot', 'spawn', 'room'],
        pre_condition=[('+', Fluent('spawn_in_room', '?spawn', '?room')),
                       ('+', Fluent('oriented', '?r')),
                       ('+', Fluent('at_spawn', '?r', '?spawn')),
                       ('+', Fluent('robot_in_room', '?r', '?room'))
                       ],
        post_condition=[('-', Fluent('at_spawn', '?r', '?spawn')),
                        ('-', Fluent('oriented', '?r')),
                        ],
    )
    return action_recenter
    "*** END YOUR CODE HERE ***"


def create_action_pick():
    """
    Pick up an object when
    both the robot and the
    object are at the same
    spawn. Robot can only hold
    one thing at a time. The
    object will no longer be at
    the spawn, because it will
    be on the robot.
    """
    "*** BEGIN YOUR CODE HERE ***"
    action_pick = Action(
        name='pick',
        args=['?r', '?object', '?spawn'],
        types=['robot', 'object', 'spawn'],
        pre_condition=[('+', Fluent('at_spawn', '?r', '?spawn')),
                       ('+', Fluent('at_spawn', '?object', '?spawn')),
                       ('+', Fluent('empty', '?r'))
                       ],
        post_condition=[('+', Fluent('holding', '?object', '?r')),
                        ('-', Fluent('empty', '?r')),
                        ('-', Fluent('at_spawn', '?object', '?spawn'))
                        ],
    )
    return action_pick
    "*** END YOUR CODE HERE ***"


def create_action_place():
    """
    Place down a held object on
    the target spawn location
    """
    "*** BEGIN YOUR CODE HERE ***"
    action_place = Action(
        name='place',
        args=['?r', '?object', '?spawn'],
        types=['robot', 'object', 'spawn'],
        pre_condition=[('+', Fluent('holding', '?object', '?r')),
                       ('+', Fluent('at_spawn', '?r', '?spawn')),
                       ('-', Fluent('empty', '?r')),
                       ],
        post_condition=[('+', Fluent('empty', '?r')),
                        ('+', Fluent('at_spawn', '?object', '?spawn')),
                        ('-', Fluent('holding', '?object', '?r')),
                        ],
    )
    return action_place
    "*** END YOUR CODE HERE ***"


class State:    
    def __init__(self, objects_of_type, fluent_list):
        self.objects_of_type = objects_of_type
        self.fluent_list = fluent_list

    def __eq__(self, other):
        fluent_check = (frozenset(self.fluent_list) == frozenset(other.fluent_list))
        obejects_of_type_check = self.objects_of_type == other.objects_of_type
        return (fluent_check and obejects_of_type_check)    

    def __hash__(self):
        return int(hashlib.md5(str(tuple(sorted(set(self.fluent_list)))).encode('utf-8')).hexdigest(), 16) % 4700

    def __str__(self):
        return f"{self.fluent_list}"
    
    def __repr__(self):
        return str(self)


class Planner:

    def solve_action(self, start_state, goal, actions):

        '''
        start_state: State
        goal: List[(Fluent())]
        actions: List[Action]
        '''
        self.start_state = start_state
        self.goal = goal
        self.actions = actions

        self.action_list = []
        self.state_list = []

        if self.check_goal(start_state, self.goal):
            return []

        # all grounded action
        ground_actions = []
        for action in actions:
            for act in action.grounding(start_state):
                ground_actions.append(act)

        visited = set([start_state])
        fringe = [(start_state, None)]
        while fringe:
            state, plan = fringe.pop(0)
            for act in ground_actions:
                if act.check_pre_condition(state):
                    new_state = act.apply_post_condition(state)
                    if new_state not in visited:
                        if self.check_goal(new_state, self.goal):
                            full_plan = [act]
                            while plan:
                                act, plan = plan
                                full_plan.insert(0, act)
                            return full_plan
                        visited.add(new_state)
                        fringe.append((new_state, (act, plan)))
        return []   
    
    def solve_states(self, start_state, goal, actions):
        state_list = [start_state]
        action_plan = self.solve_action(start_state, goal, actions)
        for a in action_plan:
            prev_state = state_list[-1]
            next_state = a.apply_post_condition(prev_state)
            state_list.append(next_state)
        return state_list              

    def check_goal(self, state, goal):
        for fluent in goal:
            if fluent not in state.fluent_list:
                return False
        return True


if __name__ == '__main__':
    
    action_transit = create_action_transit()
    action_orient = create_action_orient()
    action_recenter = create_action_recenter()
    action_pick = create_action_pick()
    action_place = create_action_place()
    actions = [action_transit, action_orient, action_recenter, action_pick, action_place]
    
    objects_of_type = {'robot': ['robot0'], 
                       'room': ['room0', 'room1'],
                       'object': ['apple0', 'banana0'],
                       'spawn': ['desk0_desktop', 'counter0_left']
                       }

    start_fluent_list = [Fluent('robot_in_room', 'robot0', 'room0'),
                         Fluent('at_spawn', 'apple0', 'desk0_desktop'),
                         Fluent('at_spawn', 'banana0', 'counter0_left'),
                         Fluent('adjacent', 'room0', 'room1'),
                         Fluent('adjacent', 'room1', 'room0'),
                         Fluent('spawn_in_room', 'desk0_desktop', 'room0'),
                         Fluent('spawn_in_room', 'counter0_left', 'room1'),
                         Fluent('empty', 'robot0'),
                         ]

    start_state = State(objects_of_type, start_fluent_list)

    goal_fluent_list = [Fluent('robot_in_room', 'robot0', 'room0'),
                         Fluent('at_spawn', 'apple0', 'desk0_desktop'),
                         Fluent('at_spawn', 'banana0', 'desk0_desktop'),
                         Fluent('empty', 'robot0'),
                         ]
    
    print("IMPORTANT ROBOT MISSION: Put the fruit on the desktop!")
    print("Start state:", start_state)
    print("Goal fluents:", goal_fluent_list)
    print()
    print("Actions:")
    for a in actions:
        print(a)
    print()
    print("Planning...")
    print()
    print("Plan:")

    planner = Planner()

    plan = planner.solve_action(start_state, goal_fluent_list, actions)
    for i, a in enumerate(plan):
        print(f"Step {i+1}:", a)
    
    print()
    print("Verifying correctness of plan...")
    for i, a in enumerate(plan):
        if not a.check_pre_condition(start_state):
            print(f"Step {i+1} failed: precondition not met.")
            sys.exit(0)
        print(f"Step {i+1} passed: precondition met")
        start_state = a.apply_post_condition(start_state)
    if planner.check_goal(start_state, goal_fluent_list):
        print("Plan is correct!")
    else:
        print("Plan failed: goal not met")