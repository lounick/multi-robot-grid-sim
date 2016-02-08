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
from __future__ import print_function

import numpy as np
import copy
from collections import OrderedDict
import math
from utils.priority_queue import PriorityQueue
from utils.utilities import getEucledianDistance
from robots.target import Target
from task_scheduling.tsp_problem import tsp_solver
import task_scheduling.utils as tsu
from utils.grid_with_weights import GridWithWeights


class InspectionRobot:
    def __init__(self, init_pos, max_vel=None, pos_var=None, insp_dur=None, insp_var=None, world_map=None,
                 max_time=None, max_energy=None, ex_point=None):
        self.pos = copy.deepcopy(init_pos)

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

        if max_time is None:
            max_time = float("inf")
        self.max_time = max_time

        if max_energy is None:
            max_energy = float("inf")
        self.max_energy = max_energy

        if ex_point is None:
            ex_point = init_pos
        self.ex_point = ex_point

        self.state = 'wait'

        self.action_start = 0
        self.expected_timeout = 0
        self.init_pos = init_pos
        self.targets_length = 0

        self.states = {'wait': self.state_wait, 'navigate_target': self.state_navigate_target,
                       'inspect': self.state_inspect, 'navigate_extraction': self.state_navigate_extraction,
                       'wait_extraction': self.state_wait_extraction}

        self.targets = OrderedDict()
        self.curr_target_uid = None
        self.world_time = 0
        self.schedule = None
        self.schedule_uids = None
        self.inspecting = False
        self.distance_map = OrderedDict()
        self.classified_targets = set()
        self.unclassified_targets = set()
        self.target_pos = None
        self.action_init_pos = None



    def move(self, coord_x, coord_y):
        """
        Move from your current position to the next position given by coordinates x and y. If it is further than your
        maximum velocity just move as far as you can
        :param coord_x:
        :param coord_y:
        :return:
        """
        if abs(self.pos[0] - coord_x) < self.max_vel:
            self.pos[0] = coord_x
            if abs(self.pos[1] - coord_y) < self.max_vel:
                self.pos[1] = coord_y
            else:
                self.pos[1] += self.max_vel
        else:
            self.pos[0] += self.max_vel
            if abs(self.pos[1] - coord_y) < self.max_vel:
                self.pos[1] = coord_y
            else:
                self.pos[1] += self.max_vel

    def update_position(self, time):
        self.pos[0] = np.interp(time, [self.action_start, self.expected_timeout],
                                [self.action_init_pos[0], self.target_pos[0]])
        self.pos[1] = np.interp(time, [self.action_start, self.expected_timeout],
                                [self.action_init_pos[1], self.target_pos[1]])

    def insert_target(self, target):
        # Calculate new costs
        self.targets[target.id] = target
        self.unclassified_targets.add(target.id)
        distances = OrderedDict()
        if self.world_map is None:
            for k,v in self.distance_map.items():
                dist = getEucledianDistance(target.pos, self.targets[k].pos)
                v[target.id] = dist
                distances[k] = dist
            distances[target.id] = 0
            self.distance_map[target.id] = distances
        else:
            # Use A*
            pass

        self.do_reschedule()

    def do_reschedule(self):
        self.targets_length = len(self.unclassified_targets)
        if self.targets_length > 1:
            if self.world_map is None:  # There is no map, assume empty world.
                positions = []
                target_uids = []
                self.schedule_uids = []
                positions.append([self.pos[0], self.pos[1]])
                for uid in self.unclassified_targets: #TODO: Make use of the distance map.
                    target_uids.append(uid)
                    positions.append(self.targets[uid].pos)
                positions = np.array(positions)
                cost = tsu.calculate_distances(positions)
                self.schedule, objective, _ = tsu.solve_problem(tsp_solver, cost)
                self.schedule.pop(0)
                self.schedule.pop(len(self.schedule)-1)
                for i in range(len(self.schedule)):
                    self.schedule[i] -= 1
                    self.schedule_uids.append(target_uids[self.schedule[i]])
                print(self.schedule)
                print(self.schedule_uids)
            else:
                pass
        elif self.targets_length == 1:
            self.schedule = [0]
            self.schedule_uids = []
            for uid in self.unclassified_targets:
                self.schedule_uids.append(uid)
            print(self.schedule)
            print(self.schedule_uids)

    def state_wait(self, **kwargs):
        """
        1. Check for changes in Targets List.
        2. If there are changes make a new visiting schedule.
        3. If there are targets choose the next one in the schedule and go to inspect.
        """
        if self.targets_length != len(self.unclassified_targets):
            self.do_reschedule()

        # TODO: should implement a function that returns the time to reach a wp to have a more generic approach.
        # TODO: Implement energy constraints as well.
        self.action_start = self.world_time
        self.action_init_pos = copy.deepcopy(self.pos)

        if len(self.targets) > 0 and len(self.schedule_uids) > 0:
            # If there is time to navigate and inspect start the navigation. Else go to extraction.
            self.curr_target_uid = self.schedule_uids[0]
            self.target_pos = self.targets[self.curr_target_uid].pos
            self.expected_timeout = self.action_start + getEucledianDistance(self.pos, self.target_pos) / self.max_vel
            if self.expected_timeout + getEucledianDistance(self.ex_point, self.target_pos) / self.max_vel + self.insp_dur <= self.max_time:
                self.state = 'navigate_target'
            else:
                print("Going to extraction")
                self.target_pos = self.ex_point
                self.expected_timeout = self.action_start + getEucledianDistance(self.pos, self.ex_point) / self.max_vel
                self.state = 'navigate_extraction'
        else:
            self.expected_timeout = self.action_start + getEucledianDistance(self.pos, self.ex_point) / self.max_vel
            if self.expected_timeout >= self.max_time:
                print("Going to extraction")
                self.target_pos = self.ex_point
                self.state = 'navigate_extraction'

    def state_navigate_target(self, **kwargs):
        """
        1. Generate a path to the target using your map.
        2. Go from waypoint to waypoint.
        3. If you reach your goal change state.
        """
        if self.curr_target_uid is not None:
            if self.curr_target_uid == self.schedule_uids[0]:
                self.update_position(self.world_time)
                if self.world_time >= self.expected_timeout:
                    self.state = 'inspect'
            else:
                self.update_position(self.world_time)
                self.state = 'wait'
        else:
            # Should not be here. Just go to wait.
            self.update_position(self.world_time)
            self.state = 'wait'

    def state_inspect(self, **kwargs):
        """
        Just waist time here. And produce some result.
        """
        if not self.inspecting:
            self.inspecting = True
            self.expected_timeout = self.world_time + self.insp_dur
        else:
            if self.world_time >= self.expected_timeout:
                self.inspecting = False
                self.targets[self.curr_target_uid].classification = "Mine"
                # self.target_uids.pop(0)
                self.schedule_uids.pop(0)
                self.unclassified_targets.remove(self.curr_target_uid)
                self.classified_targets.add(self.curr_target_uid)
                self.curr_target_uid = None
                self.state = 'wait'

    def state_navigate_extraction(self, **kwargs):
        """
        If we exceed our time or energy limit we go to the extraction point.
        """
        self.update_position(self.world_time)
        if self.world_time >= self.expected_timeout:
            self.state = 'wait_extraction'

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
    timestep = 0.01  # Timestep in seconds
    max_time = 100  # Maximum simulation time in seconds
    time = 0

    vehicles = []

    iv = InspectionRobot([0, 0], max_vel=0.5, insp_dur=10, max_time=60)
    iv.insert_target(Target("U0001", (10, 0)))
    iv.insert_target(Target("U0002", (0, 10)))
    vehicles.append(iv)

    inserted = False

    count = 0
    do_print = False
    while (time < max_time):
        if count%100 == 0:
            do_print = True
        if do_print:
            print(time, end=' ')
        if time > 20 and not inserted:
            inserted = True
            vehicles[0].insert_target(Target("U0003", (10, 10)))
        for v in vehicles:
            v.loop(time)
            if do_print:
                print(v.pos[0], v.pos[1],end='')

        if do_print:
            print()
        time += timestep
        count += 1
        do_print = False



if __name__ == '__main__':
    main()
