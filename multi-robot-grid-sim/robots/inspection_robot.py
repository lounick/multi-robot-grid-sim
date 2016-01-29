#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (c) 2015, lounick and decabyte
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of task_scheduling nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from __future__ import division

import numpy as np
import math
from utils.priority_queue import PriorityQueue
from utils.utilities import getEucledianDistance
from robots.target import Target
from task_scheduling.tsp_problem import tsp_solver
import task_scheduling.utils as tsu
from utils.grid_with_weights import GridWithWeights


class InspectionRobot:

    def __init__(self, init_x, init_y, max_vel=None, pos_var=None, insp_dur=None, insp_var=None, world_map=None):
        self.pos_x = init_x
        self.pos_y = init_y

        if max_vel is None:
            max_vel = 1
        self.max_vel = max_vel

        if pos_var is None:
            pos_var = 0
        self.pos_var = pos_var

        if insp_dur is None:
            insp_dur = 1
        self.insp_dur = insp_dur

        if insp_var is None:
            insp_var = 0
        self.insp_var = insp_var

        if world_map is None:
            self.world_map = None
        self.world_map = world_map

        self.state = 'wait'

        self.action_start = 0
        self.expected_timeout = 0
        self.init_pos = (0, 0)
        self.targets_length = 0

        self.states = {'wait': self.state_wait, 'navigate_target': self.state_navigate_target,
                       'inspect': self.state_inspect, 'navigate_extraction': self.state_navigate_extraction,
                       'wait_extraction': self.state_wait_extraction}
        self.targets = []
        self.curr_target = None
        self.world_time = 0
        self.schedule = None



    def move(self, coord_x, coord_y):
        """
        Move from your current position to the next position given by coordinates x and y. If it is further than your
        maximum velocity just move as far as you can
        :param coord_x:
        :param coord_y:
        :return:
        """
        if abs(self.pos_x-coord_x) < self.max_vel:
            self.pos_x = coord_x
            if abs(self.pos_y-coord_y) < self.max_vel:
                self.pos_y = coord_y
            else:
                self.pos_y += self.max_vel
        else:
            self.pos_x += self.max_vel
            if abs(self.pos_y-coord_y) < self.max_vel:
                self.pos_y = coord_y
            else:
                self.pos_y += self.max_vel

    def update_position(self, time):
        self.pos_x = np.interp(time, [self.action_start, self.expected_timeout], [self.init_pos[0], self.curr_target.pos[0]])
        self.pos_y = np.interp(time, [self.action_start, self.expected_timeout], [self.init_pos[1], self.curr_target.pos[1]])

    def do_reschedule(self):
        self.targets_length = len(self.targets)
        if self.world_map is None: # There is no map, assume empty world.
            positions = []
            positions.append([self.pos_x, self.pos_y])
            for t in self.targets:
                positions.append(t.pos)
            positions = np.array(positions)
            cost = tsu.calculate_distances(positions)
            self.schedule, objective, _ = tsu.solve_problem(tsp_solver, cost)
        else:
            pass
        

    def state_wait(self, **kwargs):
        """
        1. Check for changes in Targets List.
        2. If there are changes make a new visiting schedule.
        3. If there are targets choose the next one in the schedule and go to inspect.
        """
        if self.targets_length != len(self.targets):
            self.do_reschedule()

        if len(self.targets) > 0:
            self.curr_target = self.targets[0]
            self.targets.pop(0)
            self.action_start = self.world_time
            self.init_pos = (self.pos_x, self.pos_y)
            self.expected_timeout = getEucledianDistance(self.init_pos, self.curr_target.pos) / self.max_vel
            self.state = 'navigate_target'

    def state_navigate_target(self, **kwargs):
        """
        1. Generate a path to the target using your map.
        2. Go from waypoint to waypoint.
        3. If you reach your goal change state.
        """
        if self.curr_target is not None:
            self.update_position(self.world_time)
            if self.world_time > self.expected_timeout:
                self.curr_target = None
                self.state = 'wait'

    def state_inspect(self, **kwargs):
        """
        Just waist time here. And produce some result.
        """
        pass

    def state_navigate_extraction(self, **kwargs):
        """
        If we exceed our time or energy limit we go to the extraction point.
        """
        pass

    def state_wait_extraction(self, **kwargs):
        """
        Sink state. Keep position and wait to be retreived.
        """
        pass

    def loop(self, world_time):
        """
        Get current state and decide what is your next step.
        :param world_time:
        :return:
        """
        self.world_time = world_time
        self.states[self.state]()


def main():
    timestep = 0.5 #Timestep in seconds
    max_time = 25 #Maximum simulation time in seconds
    time = 0

    vehicles = []

    iv = InspectionRobot(0, 0, 0.5)
    iv.targets.append(Target("U0001",(10,0)))
    vehicles.append(iv)

    while(time < max_time):
        print(time, end=" ")
        for v in vehicles:
            v.loop(time)
            print(v.pos_x, v.pos_y)
        time += timestep

if __name__ == '__main__':
    main()