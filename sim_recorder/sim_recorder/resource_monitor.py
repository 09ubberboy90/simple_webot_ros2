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

import pprint
from collections import defaultdict

import matplotlib.colors as mcolors
import numpy as np
import psutil
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_qt5agg import \
    FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import sys, os

class CpuFreqGraph(FigureCanvas, FuncAnimation):
    def __init__(self, parent=None):
        self._fig = Figure()
        self.idx = 0
        self.ax1 = self._fig.add_subplot(211)
        self.procs = []
        self.ax2 = self._fig.add_subplot(212)
        FigureCanvas.__init__(self, self._fig)
        FuncAnimation.__init__(
            self, self._fig, self.animate, interval=100, blit=False)
        self.setParent(parent)
        # Needed to initialize the capture
        psutil.cpu_times_percent(interval=None)
        self.x_data = np.arange(0, 100)
        self.cpu_data = np.zeros((0, 100))
        self.ram_data = np.zeros((0, 100))
        self.cpu_dict = defaultdict(list)
        self.ram_dict = defaultdict(list)
        self.colors = {}
        self.colors.update(mcolors.TABLEAU_COLORS)
        self.colors.update(mcolors.BASE_COLORS)
        self.colors.update(mcolors.CSS4_COLORS)
        self.colors = list(self.colors.values())
        for ax in [self.ax1, self.ax2]:
            #ax.set_xlim(0, 100)
            ax.set_xlabel("Time")
            #ax.set_ylim(0, 100)
            ax.get_xaxis().set_ticks([])
            ax.spines['right'].set_color(None)
            ax.spines['top'].set_color(None)
        self.ax1.set_ylabel("Usage (%)")
        self.ax2.set_ylabel("Usage (MB)")

        self.ax1.set_title("CPU Usage")
        self.ax2.set_title("Memory (RAM) Usage")

    def animate(self, i):

        self.cpu_data = np.roll(self.cpu_data, -1, axis=1)
        self.ram_data = np.roll(self.ram_data, -1, axis=1)
        labels = []

        # Remove processes that ended
        self.procs_copy = list(self.procs)
        for idx, p in enumerate(self.procs_copy):
            try:
                with p.oneshot():
                    p.name()
            except:
                self.procs.remove(p)


        for idx, p in enumerate(self.procs):
            with p.oneshot():
                cpu_usage = p.cpu_percent()
                self.cpu_data[idx+1][-1] = cpu_usage
                ram_usage = p.memory_info().rss / (1024*1024)
                self.ram_data[idx+1][-1] = ram_usage
                labels.append(p.name())
                self.cpu_dict[(p.name(), p.pid)].append(cpu_usage)
                self.ram_dict[(p.name(), p.pid)].append(ram_usage)
                # print(f"{p.name()} : {cpu_usage} : {p.cmdline()}")
        cpu_stack = np.cumsum(self.cpu_data, axis=0)
        ram_stack = np.cumsum(self.ram_data, axis=0)
        self.ax1.collections.clear()
        self.ax2.collections.clear()
        handle_cpu = []
        handle_ram = []
        for idx, (p, color) in enumerate(zip(self.procs, self.colors)):
            cpu = self.ax1.fill_between(
                self.x_data, cpu_stack[idx, :], cpu_stack[idx+1, :], color=color, label=labels[idx])
            ram = self.ax2.fill_between(
                self.x_data, ram_stack[idx, :], ram_stack[idx+1, :], color=color, label=labels[idx])
            handle_cpu.append((cpu, cpu_stack[idx+1, -1] - cpu_stack[idx, -1]))
            handle_ram.append((ram, ram_stack[idx+1, -1] - ram_stack[idx, -1]))

        if self.procs:
            handles, labels = self.ax1.get_legend_handles_labels()
            handle_cpu.sort(key = lambda x: x[1], reverse=True)
            handle_cpu = [x[0] for x in handle_cpu]
            self.ax1.legend(handle_cpu, labels, loc='upper left')
            handles, labels = self.ax2.get_legend_handles_labels()
            handle_ram.sort(key = lambda x: x[1], reverse=True)
            handle_ram = [x[0] for x in handle_ram]
            self.ax2.legend(handle_ram, labels, loc='upper left')

    def update_proc(self, selected_proc):
        self.procs = selected_proc

        # Reset arrays
        self.cpu_data = np.zeros((len(self.procs)+1, 100))
        self.ram_data = np.zeros((len(self.procs)+1, 100))
        self.cpu_dict = defaultdict(list)
        self.ram_dict = defaultdict(list)

        # Set cpu time
        for p in self.procs:
            p.cpu_percent() # <- Return a 0 that should be ignored

    def dump_values(self):
        with open(f"/home/ubb/Documents/PersonalProject/VrController/sim_recorder/data/cpu_{self.idx}.csv", "w") as f:
            for (name, pid), el in self.cpu_dict.items():
                f.write(f"{name},{','.join(str(v) for v in el)}\n")
        with open(f"/home/ubb/Documents/PersonalProject/VrController/sim_recorder/data/ram_{self.idx}.csv", "w") as f:
            for (name, pid), el in self.ram_dict.items():
                f.write(f"{name},{','.join(str(v) for v in el)}\n")
