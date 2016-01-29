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

# Implementation is from http://www.redblobgames.com/pathfinding/a-star/implementation.html

from __future__ import division
import numpy as np
from utils.square_grid import *
from utils.grid_with_weights import GridWithWeights
from utils.priority_queue import PriorityQueue
from utils.utilities import getEucledianDistance


def astar(map, start, goal, eight_way=None):
    (rows, columns) = map.shape
    g = GridWithWeights(rows, columns)
    if eight_way is not None:
        g.eight_way = eight_way

    for row in range(rows):
        for col in range(columns):
            if map[row][col] == 1:
                g.walls.append((col, row))
    draw_grid(g)
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            break

        for next in g.neighbors(current):
            new_cost = cost_so_far[current] + g.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + getEucledianDistance(goal, next)
                frontier.put(next, priority)
                came_from[next] = current

    return came_from, cost_so_far



def main():
    DIAGRAM1_WALLS = [from_id_width(id, width=30) for id in [21,22,51,52,81,82,93,94,111,112,123,124,133,134,141,142,153,154,163,164,171,172,173,174,175,183,184,193,194,201,202,203,204,205,213,214,223,224,243,244,253,254,273,274,283,284,303,304,313,314,333,334,343,344,373,374,403,404,433,434]]
    g = SquareGrid(15,30)
    g.walls = DIAGRAM1_WALLS
    draw_grid(g)

    map = np.zeros((15,30), dtype=np.int8)

    for wall in g.walls:
        map[wall[1]][wall[0]] = 1

    for i in range(15):
        for j in range(30):
            print(map[i][j], end=" ")
        print("")

    came_from, cost_so_far = astar(map, (0,0), (29,14), eight_way=True)
    end = (29,14)
    start = (0,0)
    next = end
    path = []
    while(next != start):
        path.append(next)
        next = came_from[next]
    path.append(next)
    path.reverse()
    for tile in path:
        print("("+str(tile[1])+", "+str(tile[0])+")",end="")
    print()
    draw_grid(g, width=3, point_to=came_from, start=(0,0), goal=(29,14))
    print()
    draw_grid(g, width=3, number=cost_so_far, start=(0,0), goal=(29,14))

if __name__ == '__main__':
    main()