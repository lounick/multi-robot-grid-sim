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
from utils.utilities import getEucledianDistance

class SearchRobot:
    def __init__(self, init_pos, area, max_vel=None, pos_var=None, max_time=None, max_energy=None, ex_point=None):
        """
        Class initialisation.
        :param init_pos: Position vector containing the initial position of the vehicle.
        :param area: List of points creating the bounding box of the area to be searched.
        :param max_vel: Optional maximum velocity of the vehicle.
        :param pos_var: Optional position variance for error calculation.
        :param max_time: Optional maximum mission time.
        :param max_energy: Optional maximum mission energy.
        :param ex_point: Optional extraction point for the vehicle.
        """
        self.init_pos = init_pos
        self.pos = copy.deepcopy(init_pos)

        self.area = area

        if max_vel is None:
            max_vel = 1
        self.max_vel = max_vel

        if pos_var is None:
            pos_var = 0
        self.pos_var = pos_var

        if max_time is None:
            max_time = float("inf")
        self.max_time = max_time

        if max_energy is None:
            max_energy = float("inf")
        self.max_energy = max_energy

        if ex_point is None:
            ex_point = init_pos
        self.ex_point = ex_point

        self.states = {'wait': self.state_wait, 'navigate_wp': self.state_navigate_wp,
                       'navigate_extraction': self.state_navigate_extraction,
                       'wait_extraction': self.state_wait_extraction}
        self.state = 'wait'

        self.action_start = 0
        self.expected_timeout = 0

        self.world_time = 0

        self.waypoints = []
        self.next_wp = None
        self.action_init_pos = None

    def update_position(self, time):
        self.pos[0] = np.interp(time, [self.action_start, self.expected_timeout],
                                [self.action_init_pos[0], self.next_wp[0]])
        self.pos[1] = np.interp(time, [self.action_start, self.expected_timeout],
                                [self.action_init_pos[1], self.next_wp[1]])

    def generate_lawnmower(self):
        """
        Generate a lawnmower pattern based on the area and the sensor range and type.
        """
        pass

    def target_detection(self):
        """
        Check if a target is detected based on sensor type and range.
        Should avoid multiple target detections.
        :return:
        """
        pass

    def state_wait(self, **kwargs):
        """
        If there is a next waypoint go to there.
        Else you have finished the search and have to move to extraction.
        """
        # TODO: Implement energy constraints as well.
        self.action_start = self.world_time
        self.action_init_pos = copy.deepcopy(self.pos)
        if len(self.waypoints) > 0:
            # Calculate time to see if you can achieve it
            self.next_wp = self.waypoints[0]
            self.expected_timeout = self.action_start + getEucledianDistance(self.pos, self.next_wp) / self.max_vel
            if self.expected_timeout + getEucledianDistance(self.ex_point, self.next_wp) / self.max_vel <= self.max_time:
                self.state = 'navigate_wp'
            else:
                self.next_wp = self.ex_point
                self.expected_timeout = self.action_start + getEucledianDistance(self.pos, self.ex_point) / self.max_vel
                self.state = 'navigate_extraction'
        else:
            self.next_wp = self.ex_point
            self.expected_timeout = self.action_start + getEucledianDistance(self.pos, self.ex_point) / self.max_vel
            self.state = 'navigate_extraction'
        pass

    def state_navigate_wp(self, **kwargs):
        """
        If you have reached your goal go to wait state.
        Else just update your position.
        """
        # TODO: Check if we found a target.
        if self.next_wp is not None:
            self.update_position(self.world_time)
            if self.world_time >= self.expected_timeout:
                self.next_wp = None
                self.waypoints.pop(0)
                self.state = 'wait'
        else:
            # Should not be here. Just go to wait.
            self.update_position(self.world_time)
            self.state = 'wait'


    def state_navigate_extraction(self, **kwargs):
        """
        If you have reached your goal go to wait state.
        Else just update your position.
        """
        pass

    def state_wait_extraction(self, **kwargs):
        """
        Sink state. Keep position and wait to be retreived.
        """
        pass

    def loop(self, world_time):
        self.world_time = world_time


def main():
    timestep = 0.01  # Timestep in seconds
    max_time = 100  # Maximum simulation time in seconds
    time = 0

    vehicles = []

    count = 0
    do_print = False

    while time < max_time:
        if count%100 == 0:
            do_print = True
        if do_print:
            print(time, end=' ')

        for v in vehicles:
            v.loop(time)
            if do_print:
                print(v.pos[0], v.pos[1], end='')

        if do_print:
            print()
        time += timestep
        count += 1
        do_print = False


if __name__ == '__main__':
    main()
