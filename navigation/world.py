import math
from array import array
from typing import List, Dict, Callable, NamedTuple

import numpy as np


# Rotation is also an attribute in 3d space time lol


class RelativePosition:
    def __init__(self, dx, dy, rot):
        self.dx: float = dx  # in milimeters
        self.dy: float = dy  # in milimeters
        self.rot: float = rot  # in degrees

    def reverse(self):
        return RelativePosition(-self.dx, -self.dy, -self.rot)

    def length_sq(self):
        return self.dx * self.dx + self.dy * self.dy

    def length(self):
        return math.sqrt(self.length_sq())


class Cell:
    class Connection:
        def __init__(self, origin, target, rel_pos):
            self.rel_pos: RelativePosition = rel_pos
            self.origin: Cell = origin
            self.target: Cell = target

    def __init__(self, position: array):  # position is a embedding
        self.position: array = position
        self.neighbors: List[Cell.Connection] = []

    def connect(self, target, rel_pos: RelativePosition):
        self.neighbors.append(Cell.Connection(self, target, rel_pos))
        target.neighbors.append(Cell.Connection(target, self, rel_pos.reverse()))


class World:
    def __init__(self, similarity):
        self.nodes: List[Cell] = []
        self.func: Callable[[array, array], float] = similarity

    def add_cell(self, cell: Cell):
        self.nodes.append(cell)

    def probability(self, pos: array):
        probMap = {}
        for cell in self.nodes:
            probMap[cell] = self.func(pos, cell.position)

        return probMap

    def get_cell(self, pos: array) -> [Cell, float]:
        map = self.probability(pos)
        newlist = sorted(map.keys(), key=lambda d: map[d])
        return newlist[0], max(map.values())
