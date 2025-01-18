""" Main utilities for pyrobosim GUI. """

import numpy as np
import sys
import time
import warnings

from PyQt5 import QtWidgets
from PyQt5.QtCore import QTimer
from matplotlib.backends.qt_compat import QtCore
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT

from world_canvas import WorldCanvas
from knowledge import query_to_entity


def start_gui(world, action_list, pretty_actions=None, no_graphics=False):
    """
    Helper function to start a pyrobosim GUI for a world model.

    :param world: World object to attach to the GUI.
    :type world: :class:`pyrobosim.core.world.World`

    :param action_list: List of actions to perform in the GUI.
    :param pretty_actions: List of pretty actions to display in the GUI, in place of the actual actions
    """
    from autograder import no_graphics_dict
    if not no_graphics:
        no_graphics = no_graphics_dict['val']
    app = PyRoboSimGUI(world, action_list, sys.argv, auto_gui=False, show=not no_graphics, pretty_actions=pretty_actions)

    timer = QTimer(parent=app)
    timer.timeout.connect(lambda: None)
    timer.start(1000)

    sys.exit(app.exec_())


class PyRoboSimGUI(QtWidgets.QApplication):
    """Main pyrobosim GUI class."""

    def __init__(self, world, action_list, args, auto_gui, show=True, pretty_actions=None):
        """
        Creates an instance of the pyrobosim GUI.

        :param world: World object to attach to the GUI.
        :type world: :class:`pyrobosim.core.world.World`
        :param args: System arguments, needed by the QApplication constructor.
        :type args: list[str]
        :param show: If true (default), shows the GUI. Otherwise runs headless for testing.
        :type show: bool, optional
        """
        super(PyRoboSimGUI, self).__init__(args)
        self.world = world
        self.main_window = PyRoboSimMainWindow(world, action_list, auto_gui, pretty_actions=pretty_actions)
        if show:
            self.main_window.show()


class PyRoboSimMainWindow(QtWidgets.QMainWindow):
    """Main application window for the pyrobosim GUI."""

    def __init__(self, world, action_list, auto_gui, pretty_actions=None, *args, **kwargs):
        """
        Creates an instance of the pyrobosim application main window.

        :param world: World object to attach.
        :type world: :class:`pyrobosim.core.world.World`
        """
        super(PyRoboSimMainWindow, self).__init__(*args, **kwargs)
        # Imports scripted action_list
        self.action_list = action_list
        self.pretty_actions = pretty_actions
        # Sets if GUI advances without button press, but only if scripted gui
        self.auto_gui = auto_gui if action_list else False
        self.setWindowTitle("pyrobosim")
        self.set_window_dims()

        # Connect the GUI to the world
        self.world = world
        self.world.gui = self
        self.world.has_gui = True

        self.layout_created = False
        self.canvas = WorldCanvas(self, world)
        self.create_layout()
        self.update_manip_state()
        self.canvas.show()


    def set_window_dims(self, screen_fraction=0.8):
        """
        Set window dimensions.

        :param screen_fraction: Fraction of screen (0.0 to 1.0) used by window.
        :type screen_fraction: float
        """
        screen = QtWidgets.QDesktopWidget().availableGeometry()
        window_width = int(screen.width() * screen_fraction)
        window_height = int(screen.height() * screen_fraction)
        window_x = int(screen.left() + 0.5 * (screen.width() - window_width))
        window_y = int(screen.top() + 0.5 * (screen.height() - window_height))
        self.setGeometry(window_x, window_y, window_width, window_height)

    def create_layout(self):
        """Creates the main GUI layout."""
        self.main_widget = QtWidgets.QWidget()

        # Action buttons
        self.action_layout = QtWidgets.QHBoxLayout()
        if self.auto_gui:
            self.continue_button = QtWidgets.QPushButton("Start")
        else:
            self.continue_button = QtWidgets.QPushButton("Continue")
        self.continue_button.clicked.connect(self.on_continue_click)
        self.action_layout.addWidget(self.continue_button)

        # World layout (Matplotlib affordances)
        self.world_layout = QtWidgets.QVBoxLayout()
        self.nav_toolbar = NavigationToolbar2QT(self.canvas, self)
        self.addToolBar(QtCore.Qt.BottomToolBarArea, self.nav_toolbar)
        self.world_layout.addWidget(self.canvas)

        # Main layout
        self.main_layout = QtWidgets.QVBoxLayout(self.main_widget)
        self.main_layout.addLayout(self.action_layout)
        self.main_layout.addLayout(self.world_layout)

        self.main_widget.setLayout(self.main_layout)
        self.setCentralWidget(self.main_widget)
        self.layout_created = True

    def get_current_robot(self):
        robot_name = self.world.robots[0].name
        return self.world.get_robot_by_name(robot_name)

    ####################
    # State Management #
    ####################
    def update_manip_state(self):
        """Update the manipulation state to enable/disable buttons."""
        robot = self.get_current_robot()
        if robot:
            self.canvas.show_world_state(robot, navigating=False)
            self.canvas.draw_and_sleep()

    def set_buttons_during_action(self, state):
        """
        Enables or disables buttons that should not be pressed while
        the robot is executing an action.

        :param state: Desired button state (True to enable, False to disable)
        :type state: bool
        """
        if not self.auto_gui:
            self.continue_button.setEnabled(state)

    ####################
    # Button Callbacks #
    ####################
    def rand_pose_cb(self):
        """Callback to randomize robot pose."""
        robot = self.get_current_robot()
        if not robot:
            print("No robot available.")
            return None

        sampled_pose = self.world.sample_free_robot_pose_uniform(
            robot, ignore_robots=False
        )
        if sampled_pose is not None:
            robot.set_pose(sampled_pose)
            if robot.manipulated_object is not None:
                robot.manipulated_object.pose = sampled_pose
        self.canvas.update_robots_plot()
        self.canvas.show_world_state(navigating=True)
        self.canvas.draw()

    def rand_goal_cb(self):
        """Callback to randomize robot goal."""
        all_entities = self.world.get_location_names() + self.world.get_room_names()
        entity_name = np.random.choice(all_entities)
        self.goal_textbox.setText(entity_name)

    def rand_obj_cb(self):
        """Callback to randomize manipulation object goal."""
        obj_name = np.random.choice(self.world.get_object_names())
        self.goal_textbox.setText(obj_name)

    def on_navigate_click(self):
        """Callback to navigate to a goal location."""
        robot = self.get_current_robot()
        if robot and robot.executing_action:
            return

        action_target = self.action_list.pop(0)[1].split(" ")

        query_list = [elem for elem in action_target if elem]
        #print(query_list)
        loc = query_to_entity(
            self.world,
            query_list,
            mode="location",
            robot=robot,
            resolution_strategy="nearest",
        )
        if not loc:
            return

        #print(f"[{robot.name}] Navigating to {loc}")
        self.canvas.navigate_in_thread(robot, loc)
        if self.pretty_actions is not None: print(f"PDDL action: {self.pretty_actions.pop(0)}")

    def on_pick_click(self):
        """Callback to pick an object."""
        robot = self.get_current_robot()
        if robot:
            loc = robot.location
            action_target = self.action_list.pop(0)[1].split(" ")

            query_list = [elem for elem in action_target if elem]
            if loc:
                query_list.append(loc.name)
            obj = query_to_entity(
                self.world,
                query_list,
                mode="object",
                robot=robot,
                resolution_strategy="nearest",
            )
            if obj:
                print(f"[{robot.name}] Picking {obj.name}")
                self.canvas.pick_object(robot, obj)
                self.update_manip_state()

    def on_place_click(self):
        """Callback to place an object."""
        robot = self.get_current_robot()

        # Continuing in action_list
        self.action_list.pop(0)

        if robot and robot.manipulated_object is not None:
            print(f"[{robot.name}] Placing {robot.manipulated_object.name}")
            self.canvas.place_object(robot)
            self.update_manip_state()

    def on_continue_click(self):
        """Universal button"""
        # Closes the program once clicked
        if len(self.action_list) == 0:
            self.close()
            time.sleep(0.1)
            return

        action_type = self.action_list[0][0]
        if action_type == 'Navigate':
            self.on_navigate_click()
        elif action_type == 'Pick':
            self.on_pick_click()
        elif action_type == 'Place':
            self.on_place_click()
        else:
            warnings.warn(f"{action_type} is not a valid action type")
            return
        print("Performed action")
        print(f"{len(self.action_list)} action(s) left")

        if self.auto_gui:
            time.sleep(1)
            self.on_continue_click()

        # Changes button text to indicate that there are no more actions and that the next press will close it.
        if len(self.action_list) == 0:
            self.continue_button.setText("EXIT")
