#!/usr/bin/env python

# Copyright 2022 PAL Robotics S.L.
# Copyright 2025 Wonik Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# This file is a modified version of the original joint_trajectory_controller.py
# by Noel Jimenez Garcia, adapted for the ForwardCommandController.
import os
import threading
import time
import subprocess
import yaml

import rclpy
import rclpy.exceptions

from ament_index_python.packages import get_package_share_directory

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer, Signal
from python_qt_binding.QtWidgets import QWidget, QFormLayout

from control_msgs.msg import DynamicJointState

from std_msgs.msg import Float64MultiArray

from .utils import ControllerLister, ControllerManagerLister
from .double_editor import DoubleEditor
from .joint_limits_urdf import get_joint_limits, subscribe_to_robot_description
from .update_combo import update_combo

# TODO:
# - Better UI support for continuous joints (see DoubleEditor TODO)
# - Can we load controller joints faster?, it's currently pretty slow
# - If URDF is reloaded, allow to reset the whole plugin?
# - Allow to configure:
#   - URDF location
#   - Command publishing and state update frequency
#   - Controller manager and fcc monitor frequency
# - Fail gracefully when the URDF or some other requisite is not set
# - Could users confuse the enable/disable button with controller start/stop
#   (in a controller manager sense)?
# - Better decoupling between model and view


class ForwardCommandControllerPlugin(Plugin):
    """
    Graphical frontend for a `ForwardCommandController`.

    There are two modes for interacting with a controller:

    1. **Monitor mode** Joint displays are updated with the state reported
          by the controller. This is a read-only mode and does *not* send
          control commands. Every time a new controller is selected, it starts
          in monitor mode for safety reasons.

    2. **Control mode** Joint displays update the control command that is
        sent to the controller. Commands are sent periodically even if the
        displays are not being updated by the user.
    """

    _cmd_pub_freq = 10.0  # Hz
    _widget_update_freq = 30.0  # Hz
    _ctrlrs_update_freq = 1  # Hz

    jointStateChanged = Signal([dict])

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName("ForwardCommandControllerPlugin")
        self._node = rclpy.node.Node("rqt_forward_command_controller")
        self._executor = None
        self._executor_thread = None

        # Create QWidget and extend it with all the attributes and children
        # from the UI file
        self._widget = QWidget()
        ui_file = os.path.join(
            get_package_share_directory("rqt_forward_command_controller"),
            "resource",
            "forward_command_controller.ui",
        )
        loadUi(ui_file, self._widget)
        self._widget.setObjectName("ForwardCommandControllerUi")
        ns = self._node.get_namespace()[1:-1]
        self._widget.controller_group.setTitle("ns: " + ns)

        # Remove speed scaling as it's not applicable for ForwardCommandController
        self._widget.speed_scaling_group.hide()

        # Show _widget.windowTitle on left-top of each plugin
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (" (%d)" % context.serial_number())
            )
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Initialize members
        self._fcc_name = []  # Name of selected forward command controller
        self._cm_ns = []  # Namespace of the selected controller manager
        self._joint_pos = {}  # name->pos map for joints of selected controller
        self._joint_names = []  # Ordered list of selected controller joints
        self._robot_joint_limits = {}  # Lazily evaluated on first use

        # Timer for sending commands to active controller
        self._update_cmd_timer = QTimer(self)
        self._update_cmd_timer.setInterval(int(1000.0 / self._cmd_pub_freq))
        self._update_cmd_timer.timeout.connect(self._update_cmd_cb)

        # Timer for updating the joint widgets from the controller state
        self._update_act_pos_timer = QTimer(self)
        self._update_act_pos_timer.setInterval(int(1000.0 / self._widget_update_freq))
        self._update_act_pos_timer.timeout.connect(self._update_joint_widgets)

        # Timer for controller manager updates
        self._list_cm = ControllerManagerLister()
        self._update_cm_list_timer = QTimer(self)
        self._update_cm_list_timer.setInterval(int(1000.0 / self._ctrlrs_update_freq))
        self._update_cm_list_timer.timeout.connect(self._update_cm_list)
        self._update_cm_list_timer.start()

        # Timer for running controller updates
        self._update_fcc_list_timer = QTimer(self)
        self._update_fcc_list_timer.setInterval(int(1000.0 / self._ctrlrs_update_freq))
        self._update_fcc_list_timer.timeout.connect(self._update_fcc_list)
        self._update_fcc_list_timer.start()

        # subscriptions
        subscribe_to_robot_description(self._node)

        # Signal connections
        w = self._widget
        w.enable_button.toggled.connect(self._on_fcc_enabled)
        w.fcc_combo.currentIndexChanged[str].connect(self._on_fcc_change)
        w.cm_combo.currentIndexChanged[str].connect(self._on_cm_change)

        self._cmd_pub = None  # Controller command publisher
        self._state_sub = None  # Controller state subscriber

        self._list_controllers = None

    def shutdown_plugin(self):
        self._update_cmd_timer.stop()
        self._update_act_pos_timer.stop()
        self._update_cm_list_timer.stop()
        self._update_fcc_list_timer.stop()
        self._unregister_state_sub()
        self._unregister_cmd_pub()
        self._unregister_executor()

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value("cm_ns", self._cm_ns)
        instance_settings.set_value("fcc_name", self._fcc_name)

    def restore_settings(self, plugin_settings, instance_settings):
        # Restore last session's controller_manager, if present
        self._update_cm_list()
        cm_ns = instance_settings.value("cm_ns")
        cm_combo = self._widget.cm_combo
        cm_list = [cm_combo.itemText(i) for i in range(cm_combo.count())]
        try:
            idx = cm_list.index(cm_ns)
            cm_combo.setCurrentIndex(idx)
            # Restore last session's controller, if running
            self._update_fcc_list()
            fcc_name = instance_settings.value("fcc_name")
            fcc_combo = self._widget.fcc_combo
            fcc_list = [fcc_combo.itemText(i) for i in range(fcc_combo.count())]
            try:
                idx = fcc_list.index(fcc_name)
                fcc_combo.setCurrentIndex(idx)
            except ValueError:
                pass
        except ValueError:
            pass

    def _update_cm_list(self):
        update_combo(self._widget.cm_combo, self._list_cm())

    def _update_fcc_list(self):
        # Clear controller list if no controller information is available
        if not self._list_controllers:
            self._widget.fcc_combo.clear()
            return

        # List of running controllers with a valid joint limits specification
        running_fcc = self._running_fcc_info()
        if running_fcc and not self._robot_joint_limits:
            self._robot_joint_limits = {}
            for fcc_info in running_fcc:
                self._robot_joint_limits.update(
                    get_joint_limits(self._node, _fcc_joint_names(fcc_info))
                )
        valid_fcc = []
        if self._robot_joint_limits:
            for fcc_info in running_fcc:
                has_limits = all(
                    name in self._robot_joint_limits
                    for name in _fcc_joint_names(fcc_info)
                )
                if has_limits:
                    valid_fcc.append(fcc_info)
        valid_fcc_names = [data.name for data in valid_fcc]

        # Update widget
        update_combo(self._widget.fcc_combo, sorted(valid_fcc_names))

    def _on_joint_state_change(self, current_pos):
        for name in current_pos.keys():
            try :
                self._joint_pos[name]["position"] = current_pos[name]
            except KeyError:
                pass 

    def _on_cm_change(self, cm_ns):
        self._cm_ns = cm_ns
        if cm_ns:
            self._list_controllers = ControllerLister(cm_ns)
            self._widget.fcc_combo.clear()
            self._update_fcc_list()
        else:
            self._list_controllers = None

    def _on_fcc_change(self, fcc_name):
        self._unload_fcc()
        self._fcc_name = fcc_name
        if self._fcc_name:
            self._load_fcc()

    def _on_fcc_enabled(self, val):
        # Don't allow enabling if there are no controllers selected
        if not self._fcc_name:
            self._widget.enable_button.setChecked(False)
            return

        # Enable/disable joint displays
        for joint_widget in self._joint_widgets():
            joint_widget.setEnabled(val)

        if val:
            # Widgets send reference position commands to controller
            self._update_act_pos_timer.stop()
            self._update_cmd_timer.start()
        else:
            # Controller updates widgets with feedback position
            self._update_cmd_timer.stop()
            self._update_act_pos_timer.start()

    def _load_fcc(self):

        # Initialize joint data corresponding to selected controller
        running_fcc_list = self._running_fcc_info()
        fcc_info = next(x for x in running_fcc_list if x.name == self._fcc_name)

        # Get the set of active joints from the running controller
        active_joints = _fcc_joint_names(fcc_info)

        # ('c')void WTF, why doesn't rclpy.parameter_client.AsyncParameterClient work?!
        try:
            target_node_name = f"/{self._fcc_name}"  # e.g., '/allegro_hand_controller'
            param_name_to_get = "joints"

            # Execute 'ros2 param get' command as a subprocess
            result = subprocess.run(
                ["ros2", "param", "get", target_node_name, param_name_to_get],
                capture_output=True,
                text=True,
                check=True,  # Raise an exception on error
                timeout=5,  # Timeout if it takes more than 5 seconds
            )

            # The command output (stdout) is a YAML-formatted string
            # e.g., "String array value is: ['joint1', 'joint2', ...]"
            # Parse only the list part from it
            output_str = result.stdout

            # Extract and parse the part after 'value is:'
            try:
                # Safely convert to a Python list using a YAML parser
                # Extract only the list from a format like 'String array value is: [...]'
                list_str = "[" + output_str.split("[", 1)[1]
                param_value = yaml.safe_load(list_str)

                if isinstance(param_value, list):
                    self._joint_names = [
                        joint for joint in param_value if joint in active_joints
                    ]

                else:
                    raise ValueError("Parsed value is not a list.")

            except (IndexError, ValueError) as e:
                self._node.get_logger().error(
                    f"Failed to parse subprocess output: {e}. Falling back to alphabetical order."
                )
                self._joint_names = sorted(list(active_joints))

        except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
            # Handle exceptions if the command execution fails
            self._node.get_logger().error(
                f"Subprocess call failed: {e}. Falling back to alphabetical order."
            )
            self._joint_names = sorted(list(active_joints))

        for name in self._joint_names:
            self._joint_pos[name] = {}

        self._node.get_logger().info("self._joint_names. %s" % (self._joint_names))

        # Update joint display
        try:
            layout = self._widget.joint_group.layout()
            for name in self._joint_names:
                limits = self._robot_joint_limits[name]
                joint_widget = DoubleEditor(
                    limits["min_position"], limits["max_position"]
                )
                layout.addRow(name, joint_widget)
                from functools import partial

                par = partial(self._update_single_cmd_cb, name=name)
                joint_widget.valueChanged.connect(par)
        except Exception:
            from sys import exc_info

            print("Unexpected error:", exc_info()[0])

        # Enter monitor mode (sending commands disabled)
        self._on_fcc_enabled(False)

        # Setup ROS interfaces
        fcc_ns = _resolve_controller_ns(self._cm_ns, self._fcc_name)
        # state_topic = fcc_ns + "/controller_state"
        state_topic = "/dynamic_joint_states"  # ('c')void
        cmd_topic = fcc_ns + "/commands"
        self._state_sub = self._node.create_subscription(
            DynamicJointState, state_topic, self._state_cb, 1
        )
        self._cmd_pub = self._node.create_publisher(Float64MultiArray, cmd_topic, 1)

        self.jointStateChanged.connect(self._on_joint_state_change)

        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._executor_thread = threading.Thread(
            target=self._executor.spin, daemon=True
        )
        self._executor_thread.start()

    def _unload_fcc(self):
        try:
            self.jointStateChanged.disconnect(self._on_joint_state_change)
        except Exception:
            pass

        self._unregister_state_sub()
        self._unregister_cmd_pub()
        self._unregister_executor()

        layout = self._widget.joint_group.layout()
        if layout is not None:
            while layout.count():
                layout.takeAt(0).widget().deleteLater()
            QWidget().setLayout(layout)
        self._widget.joint_group.setLayout(QFormLayout())

        self._joint_names = []
        self._joint_pos = {}

        self._widget.enable_button.setChecked(False)

    def _running_fcc_info(self):
        from .utils import filter_by_type, filter_by_state

        controller_list = self._list_controllers()
        fcc_list = filter_by_type(
            controller_list,
            "forward_command_controller/ForwardCommandController",
            match_substring=False,
        )
        running_fcc_list = filter_by_state(fcc_list, "active")
        return running_fcc_list

    def _unregister_cmd_pub(self):
        if self._cmd_pub is not None:
            self._node.destroy_publisher(self._cmd_pub)
            self._cmd_pub = None

    def _unregister_state_sub(self):
        if self._state_sub is not None:
            self._node.destroy_subscription(self._state_sub)
            self._state_sub = None

    def _unregister_executor(self):
        if self._executor is not None:
            self._executor.shutdown()
            self._executor_thread.join()
            self._executor = None

    def _state_cb(self, msg):
        # ('c')void. parse dynamic_joint_states topic 
        current_pos = {}
        for joint_idx, interface in enumerate(msg.interface_values):
            if "position" in interface.interface_names:
                position_idx = interface.interface_names.index("position")
                current_pos[msg.joint_names[joint_idx]] = interface.values[position_idx]

        if len(current_pos) > 0:
            self.jointStateChanged.emit(current_pos)

    def _update_single_cmd_cb(self, val, name):
        self._joint_pos[name]["command"] = val

    def _update_cmd_cb(self):
        cmd_msg = Float64MultiArray()
        cmd_list = []
        for name in self._joint_names:
            cmd = self._joint_pos[name].get(
                "position", 0.0
            )  # Default to current position
            try:
                cmd = self._joint_pos[name]["command"]
            except KeyError:
                pass
            cmd_list.append(cmd)
        cmd_msg.data = cmd_list
        self._cmd_pub.publish(cmd_msg)

    def _update_joint_widgets(self):
        joint_widgets = self._joint_widgets()
        for id in range(len(joint_widgets)):
            joint_name = self._joint_names[id]
            try:
                joint_pos = self._joint_pos[joint_name]["position"]
                joint_widgets[id].setValue(joint_pos)
            except KeyError:
                pass

    def _joint_widgets(self):
        widgets = []
        layout = self._widget.joint_group.layout()
        for row_id in range(layout.rowCount()):
            widgets.append(layout.itemAt(row_id, QFormLayout.FieldRole).widget())
        return widgets


def _fcc_joint_names(fcc_info):
    joint_names = set()
    # Interfaces are of the form <joint_name>/<interface_type>
    for interface in fcc_info.claimed_interfaces:
        joint_names.add(interface.split("/")[0])
    return joint_names


def _resolve_controller_ns(cm_ns, controller_name):
    assert controller_name
    ns = cm_ns.rsplit("/", 1)[0]
    if ns != "/":
        ns += "/"
    ns += controller_name
    return ns
