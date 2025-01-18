""" Rapidly-exploring Random Tree (RRT) implementation. """

import copy
import time
import numpy as np

from planner_base import PathPlannerBase
from motion import Path, reduce_waypoints_polygon
from pose import Pose
from search_graph import SearchGraph, Node
from util import *


class RRTPlannerPolygon:
    """
    Polygon representation based implementation of RRT.
    """

    def __init__(
        self,
        world,
        rrt_connect=True,
        rrt_star=False,
        collision_check_step_dist=0.025,
        max_connection_dist=0.25,
        max_nodes_sampled=1500,
        max_time=10.0,
        rewire_radius=1.0,
    ):
        """
        Creates an instance of an RRT planner.

        :param world: World object to use in the planner.
        :type world: :class:`world.World`
        :param bidirectional: If True, uses bidirectional RRT to grow trees
            from both start and goal.
        :type bidirectional: bool
        :param rrt_connect: If True, uses RRT-connect to bias tree growth
            towards goals.
        :type rrt_connect: bool
        :param rrt_star: If True, uses RRT* to rewire trees to smooth and
            shorten paths.
        :type rrt_star: bool
        :param collision_check_step_dist: Step size for discretizing collision checking.
        :type collision_check_step_dist: float
        :param max_connection_dist: Maximum connection distance between nodes.
        :type max_connection_dist: float
        :param max_nodes_sampled: Maximum nodes sampled before planning stops.
        :type max_nodes_sampled: int
        :param max_time: Maximum wall clock time before planning stops.
        :type max_time: float
        :param rewire_radius: Radius around a node to rewire the RRT,
            if using the RRT* algorithm.
        :param rewire_radius: float
        """

        self.world = world

        # Algorithm options
        self.rrt_connect = rrt_connect
        self.rrt_star = rrt_star

        # Parameters
        self.collision_check_step_dist = collision_check_step_dist
        self.max_connection_dist = max_connection_dist
        self.max_nodes_sampled = max_nodes_sampled
        self.max_time = max_time
        self.rewire_radius = rewire_radius

        # Visualization
        self.color_start = [0, 0, 0]
        self.color_goal = [0, 0.4, 0.8]
        self.color_alpha = 0.5

        self.latest_path = Path()

        self.reset()

    def reset(self):
        """Resets the search trees and planning metrics."""
        self.graph = SearchGraph(color=[0, 0, 0])
        self.latest_path = Path()
        self.planning_time = 0.0
        self.nodes_sampled = 0
        self.n_rewires = 0

    def is_connectable(self, first_node, second_node):
        """
        You need to call this!

        Checks if two nodes in the graph we are building can be connected.
        """
        return self.world.is_connectable(
            first_node.pose,
            second_node.pose,
            self.collision_check_step_dist,
            self.max_connection_dist,
        )

    def plan(self, start, goal):
        """
        Plans a path from start to goal.

        :param start: Start pose.
        :type start: :class:`pose.Pose`
        :param goal: Goal pose.
        :type goal: :class:`pose.Pose`
        :return: Path from start to goal.
        :rtype: :class:`motion.Path`
        """
        self.reset()
        
        # Fix the random seed. Do NOT change
        np.random.seed(3)

        # Create the start and goal nodes
        n_start = Node(start, parent=None)
        n_goal = Node(goal, parent=None)
        self.graph.nodes = {n_start}
        t_start = time.time()
        goal_found = False

        # If the goal is within max connection distance of the start, connect them directly
        if self.is_connectable(n_start, n_goal):
            path_poses = [n_start.pose, n_goal.pose]
            self.latest_path = Path(poses=path_poses)
            self.latest_path.fill_yaws() # geometry thing, don't worry about this
            self.planning_time = time.time() - t_start
            return self.latest_path

        while not goal_found:

            # Notes to help you:
            # Remember to keep track of how many configurations you have sampled (self.nodes_sampled)
            #   (otherwise the termination code at the end of this won't work)
            # High level pseudocode:
            # 1. Sample a node
            # 2. Find its nearest neighbor in the graph (`self.graph`)
            # 3. Clip the max distance to grow the tree toward the sampled node (see handout, this is done by `self.make_clipped_node`)
            # 4. Check if the clipped node is connectable to the nearest neighbor. If so, add to graph (add both node and edge)
            # 5. If RRT* is enabled, rewire the graph (assuming that we added the clipped node to the graph)
            # 6. Set `goal_found` to the result of trying to connect the new node to the goal (use `self.try_connect`)
            "*** BEGIN YOUR CODE HERE ***"
            sampled_node = self.sample_configuration()
            self.nodes_sampled += 1
            nearest_neighbor = self.graph.nearest(sampled_node)
            clip = self.make_clipped_node(nearest_neighbor, sampled_node)
            if self.is_connectable(clip, nearest_neighbor):
                self.graph.add_node(clip)
                self.graph.add_edge(nearest_neighbor, clip)
                if self.rrt_star:
                    self.rewire_node(self.graph,clip)
                goal_found = self.try_connect(self.graph, clip, n_goal)
            "*** END YOUR CODE HERE ***"

            # Termination code
            # Check max nodes samples or max time elapsed
            self.planning_time = time.time() - t_start
            if self.planning_time > self.max_time or self.nodes_sampled > self.max_nodes_sampled:
                self.latest_path = Path()
                return self.latest_path
        
        n = n_goal
        path_poses = [n.pose]
        path_built = False
        while not path_built:
            if n.parent is None:
                path_built = True
            else:
                n = n.parent
                path_poses.append(n.pose)
        path_poses.reverse()
        
        self.latest_path = Path(poses=path_poses)
        self.latest_path.fill_yaws()
        print("Finished planning in", self.planning_time, f"seconds ({self.nodes_sampled} sampled points)")
        print("Number of rewires:", self.n_rewires)
        print("Total path length:", self.latest_path.length)
        return self.latest_path

    def sample_configuration(self):
        """
        You will need to call this function.

        Sample an unoccupied robot pose in the world.
        This is done using uniform sampling within the world X-Y bounds and rejecting
        any samples that are in collision with entities in the world.

        :return: Collision-free pose if found, else ``None``.
        :rtype: :class:`pose.Pose`
        """
        return self.world.sample_free_robot_pose_uniform()

    def make_clipped_node(self, n_start, q_target):
        """
        You need to call this function!

        Grows the RRT toward a sampled pose in the world starting from a specific node (eg, its closest neighbor)
        The maximum distance to grow the tree is dictated by the
        ``max_connection_dist`` parameter.
        If the target pose is nearer than this distance, a new node is created
        at exactly that pose.

        :param n_start: Tree node from which to grow the new node.
        :type n_start: :class:`search_graph.Node`
        :param q_target: Target pose towards which to grow the new node.
        :type q_target: :class:`pose.Pose`
        :return: A new node grown from the start node towards the target pose.
        :rtype: :class:`search_graph.Node`
        """
        q_start = n_start.pose
        dist = q_start.get_linear_distance(q_target)
        
        step_dist = self.max_connection_dist
        if dist <= step_dist:
            q_new = q_target
        else:
            theta = q_start.get_angular_distance(q_target)
            q_new = Pose(
                x=q_start.x + step_dist * np.cos(theta),
                y=q_start.y + step_dist * np.sin(theta),
            )
        dist = min(dist, step_dist)
        return Node(q_new, parent=n_start, cost=n_start.cost + dist)
    
    def rewire_node(self, graph, n_tgt):
        """
        Rewires a node in the RRT by checking if switching the parent node to
        another nearby node will reduce its total cost from the root node.

        This is the key modification in the RRT* algorithm which requires more
        computation, but produces paths that are shorter and smoother than
        plain RRT. The vicinity around the node is defined by the
        ``rewire_radius`` parameter.

        :param graph: The tree to rewire.
        :type graph: :class:`search_graph.SearchGraph`
        :param n_tgt: The target tree node to rewire within the tree.
        :type n_tgt: :class:`search_graph.Node`
        """
        # First, find the node to rewire, if any
        #
        # Remember that the rewiring node should be:
        # 1. Connectable to the target node
        # 2. Within `self.rewire_radius`
        # 3. Should reduce the cost of reaching the target node from the start node
        #
        # You will find the `is_connectable` function useful here, and also the `cost` field of nodes, which is the total distance from the start to that node.
        #
        # You will also want to get the distance between the poses (configurations) of nodes
        # You can do that using the `get_linear_distance` method in the `Pose` class. Every node has a `pose` field that you can access.
        n_rewire = None
        
        "*** BEGIN YOUR CODE HERE ***"
        old_parent = n_tgt.parent
        min_cost = n_tgt.cost
        for node in graph.nodes:        
            dist = node.pose.get_linear_distance(n_tgt.pose, ignore_z=True)
            if self.is_connectable(node, n_tgt) and dist <= self.rewire_radius:
                new_cost = node.cost + dist
                if new_cost < min_cost:
                    n_rewire = node
                    min_cost = new_cost
        "*** END YOUR CODE HERE ***"

        # If we found a rewire node, do the rewire
        # This involves doing the following:
        # 1. Set the parent of the target node to the rewire node, update `parent` field
        # 2. Recalculate the cost of the target node (if you didn't do that in the above code block you wrote), update `cost` field
        # 3. Remove the edge between the old parent and the target node (`graph.remove_edge`)
        # 4. Add an edge between the rewire node and the target node (`graph.add_edge`)
        # Consult the lecture slides for a visual overview of this process.
        if n_rewire is not None:
            "*** BEGIN YOUR CODE HERE ***"
            n_tgt.parent = n_rewire
            n_tgt.cost = min_cost   
            graph.remove_edge(old_parent,n_tgt)
            graph.add_edge(n_rewire, n_tgt)
            "*** END YOUR CODE HERE ***"
            self.n_rewires += 1

    def try_connect(self, graph, n_curr, n_tgt):
        """
        Try to connect a node ``n_curr`` to a target node ``n_tgt``.
        This will keep extending the current node towards the target if
        RRT-Connect is enabled, or else will just try once.

        :param graph: The tree object.
        :type graph: :class:`search_graph.SearchGraph`
        :param n_curr: The current tree node to try connect to the target node.
        :type n_curr: :class:`search_graph.Node`
        :param n_tgt: The target tree node defining the connection goal.
        :type n_tgt: :class:`search_graph.Node`
        :return: A tuple containing connection success and final node added.
        :rtype: (bool, :class:`search_graph.Node`)
        """
        while True:
            dist = n_curr.pose.get_linear_distance(n_tgt.pose)

            # First, try directly connecting to the goal
            if dist < self.max_connection_dist and self.is_connectable(n_curr, n_tgt):
                n_tgt.parent = n_curr
                graph.nodes.add(n_tgt)
                if self.rrt_star:
                    self.rewire_node(graph, n_tgt)
                print("directly connected to tgt")
                return True

            if self.rrt_connect:
                "*** BEGIN YOUR CODE HERE ***" 
                """
                1. Make a new node in between the current node and the goal by clipping the line that connects them.
                This way the new node is close to the current node, but it inches a small amount toward the goal.
                2. Check if the current node and the new node can be connected. If so, connect them together.
                3. The current node is updated to be the new node, and the process repeats until new connections cannot
                be formed
                """
                
                clip = self.make_clipped_node(n_curr, n_tgt.pose)
               
                if  self.is_connectable(n_curr, clip):
                    # print("connecting to clip")
                    #clip.parent = n_curr
                    graph.add_node(clip)
                    graph.add_edge(n_curr, clip)
                    if self.rrt_star:
                        self.rewire_node(graph, clip)
                    return self.try_connect(graph, clip, n_tgt)
                else:
                    return False
                "*** END YOUR CODE HERE ***"
    
            if not self.rrt_connect:
                # If not using RRT-Connect, just return False.
                return False
                

    def get_graphs(self):
        """
        Returns the graphs generated by the planner, if any.

        :return: List of graphs.
        :rtype: list[:class:`pyrobosim.utils.search_graph.SearchGraph`]
        """
        graphs = [self.graph]
        return graphs


class RRTPlanner(PathPlannerBase):
    """Factory class for Rapidly-Exploring Random Trees path planner."""

    def __init__(self, **planner_config):
        """
        Creates and instance of RRT Planner.
        """
        super().__init__()

        self.impl = None
        if planner_config.get("grid", None):
            raise NotImplementedError("RRT does not support grid based search. ")
        else:
            self.impl = RRTPlannerPolygon(**planner_config)

    def plan(self, start, goal):
        """
        Plans a path from start to goal.

        :param start: Start pose.
        :type start: :class:`pyrobosim.utils.pose.Pose`
        :param goal: Goal pose.
        :type goal: :class:`pyrobosim.utils.pose.Pose`
        :return: Path from start to goal.
        :rtype: :class:`pyrobosim.utils.motion.Path`
        """
        start_time = time.time()
        self.latest_path = self.impl.plan(start, goal)
        self.planning_time = time.time() - start_time
        self.graphs = self.impl.get_graphs()
        return self.latest_path