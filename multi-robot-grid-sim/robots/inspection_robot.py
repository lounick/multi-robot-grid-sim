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


class InspectionRobot:
    def __init__(self, init_x, init_y, max_vel=None, pos_var=None, insp_dur=None, insp_var=None):
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

    def move(self, coord_x, coord_y):
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
