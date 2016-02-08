#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Sample code from http://www.redblobgames.com/pathfinding/
# Copyright 2014 Red Blob Games <redblobgames@gmail.com>
#
# Feel free to use this code in your own projects, including commercial projects
# License: Apache v2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>
from __future__ import print_function


class SquareGrid:
    def __init__(self, height, width):
        self.width = width
        self.height = height
        self.eight_way = False
        self.walls = []

    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height

    def passable(self, id):
        return id not in self.walls

    def neighbors(self, id):
        (x, y) = id
        if self.eight_way:
            results = [(x+1, y), (x+1, y-1), (x, y-1), (x-1, y-1), (x-1, y), (x-1, y+1), (x, y+1), (x+1, y+1)]
        else:
            results = [(x+1, y), (x, y-1), (x-1, y), (x, y+1)]
        if (x + y) % 2 == 0: results.reverse() # aesthetics
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results


def from_id_width(id, width):
    return (id % width, id // width)


def draw_tile(graph, id, style, width):
    r = "."
    if 'number' in style and id in style['number']: r = "%d" % style['number'][id]
    if 'point_to' in style and style['point_to'].get(id, None) is not None:
        (x1, y1) = id
        (x2, y2) = style['point_to'][id]
        if x2 == x1 + 1: r = u"\u2192"
        if x2 == x1 - 1: r = u"\u2190"
        if y2 == y1 + 1: r = u"\u2193"
        if y2 == y1 - 1: r = u"\u2191"
    if 'start' in style and id == style['start']: r = "A"
    if 'goal' in style and id == style['goal']: r = "Z"
    if 'path' in style and id in style['path']: r = "@"
    if id in graph.walls: r = "#" * width
    return r


def draw_grid(graph, width=2, **style):
    print("%%-%ds" % width % " ", end="")
    for y in range(graph.width):
        print("%%-%ds" % width % y, end="")
    print()
    for y in range(graph.height):
        print("%%-%ds" % width % y, end="")
        for x in range(graph.width):
            print("%%-%ds" % width % draw_tile(graph, (x, y), style, width), end="")
        print()


def main():
    # data from main article
    DIAGRAM1_WALLS = [from_id_width(id, width=30) for id in [21,22,51,52,81,82,93,94,111,112,123,124,133,134,141,142,153,154,163,164,171,172,173,174,175,183,184,193,194,201,202,203,204,205,213,214,223,224,243,244,253,254,273,274,283,284,303,304,313,314,333,334,343,344,373,374,403,404,433,434]]
    g = SquareGrid(15,30)
    g.walls = DIAGRAM1_WALLS
    draw_grid(g, width=3)

if __name__ == '__main__':
    main()