#BSD 3-Clause License
#
#Copyright (c) 2021, Florent Audonnet
#All rights reserved.
#
#Redistribution and use in source and binary forms, with or without
#modification, are permitted provided that the following conditions are met:
#
#1. Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
#2. Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
#3. Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os
import signal
import sys
from collections import defaultdict

import psutil
import rclpy



class ProcMonitor():
    """
    Create the main window and connect the menu bar slots.
    """

    def __init__(self, allowed, idx, sim_name, path):
        super().__init__('proccess_monitor')
        self.cpu_dict = defaultdict(list)
        self.ram_dict = defaultdict(list)
        self.allowed = allowed
        self.procs = {}
        self.pids = {}
        self.counter = 0
        self.current_length = 0
        self.update_missing()
        self.timer = self.create_timer(0.1, self.animate)
        self.idx = idx
        self.sim_name = sim_name
        self.path = path

    def update_missing(self):
        length = len(self.procs.keys())
        if self.counter < 10:
            if self.current_length >= length:
                self.counter += 1
            else:
                self.counter = 0
                self.current_length = length

            for proc in psutil.process_iter():
                p = proc.name()
                if proc.pid not in self.procs.keys():
                    self.procs[proc.pid] = proc
                    self.pids[proc.pid] = proc.name()
                    proc.cpu_percent()  # discard value
                    proc.cpu_percent()  # discard value

    def animate(self):
        self.update_missing()
        for pid, p in self.procs.items():
            try:
                with p.oneshot():
                    cpu_usage = p.cpu_percent()
                    if cpu_usage > 2400:
                        print("Error : High Cpu Usage")
                        cpu_usage = 2400
                    ram_usage = p.memory_info().rss / (1024*1024)
                    self.cpu_dict[pid].append(cpu_usage)
                    self.ram_dict[pid].append(ram_usage)
            except:
                pass

    def dump_values(self):
        self.name = {}
        for pid, name in self.pids.items():
            new_p = name
            counter = 0
            while new_p in self.name.keys():
                counter += 1
                new_p = new_p + "_" + str(counter)
            if counter != 0:
                name = name+"_"+str(counter)
            self.name[pid] = name

        with open(self.path + f"{self.sim_name}/cpu/cpu_{self.idx}.csv", "w") as f:
            for pid, el in self.cpu_dict.items():
                f.write(f"{self.name[pid]},{','.join(str(v) for v in el)}\n")
        with open(self.path + f"{self.sim_name}/ram/ram_{self.idx}.csv", "w") as f:
            for pid, el in self.ram_dict.items():
                f.write(f"{self.name[pid]},{','.join(str(v) for v in el)}\n")
        sys.exit(0)


allowed_gazebo = [
    "throw_moveit",
    "fake_joint_driver_node",
    "gzclient",
    "gzserver",
    "mongod",
    "move_group",
    "moveit_collision",
    "python3",
    "robot_state_publisher",
    "ros2",
    "rviz2",
    "static_transform_publisher",
    "run_recording",
    "moveit_controller",
]

allowed_webots = [
    "throw_moveit",
    "fake_joint_driver_node",
    "mongod",
    "move_group",
    "python3",
    "robot_state_publisher",
    "ros2",
    "rviz2",
    "static_transform_publisher",
    "webots",
    "webots-bin",
    "moveit_collision",
    "run_recording",
    "moveit_controller",
]
allowed_ignition = [
    "move_group",
    "parameter_bridge",
    "python3",
    "ros2",
    "ruby",
    "rviz2",
    "run_recording"
]
allowed_vr = [
"fake_joint_driver_node",
"parameter_bridge",
"republisher",
"robot_state_publisher",
"ros2",
"ruby",
"rviz2",
"servo_pose_tracking_demo",
"static_transform_publisher",
"run_recording"
]


def run(path, simulator="webots", idx=0):

    if "webots" == simulator:
        allowed = allowed_webots[1:]
    elif "gazebo" == simulator:
        allowed = allowed_gazebo[1:]
    elif "webots_throw" == simulator:
        allowed = allowed_webots[:-1]
    elif "gazebo_throw" == simulator:
        allowed = allowed_gazebo[:-1]
    elif "vr" == simulator:
        allowed = allowed_vr
    else:
        allowed = allowed_ignition

    monitor = ProcMonitor(allowed, idx, simulator, path)
    signal.signal(signal.SIGINT, lambda sig, frame: monitor.dump_values())
    signal.signal(signal.SIGTERM, lambda sig, frame: monitor.dump_values())

