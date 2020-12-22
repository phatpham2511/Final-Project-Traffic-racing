import heapq
import sys
from termcolor import colored
import math

class PriorityQueue:
  def __init__(self):
    self.queue = []
  
  def push(self, value, label):
    heapq.heappush(self.queue, (value, label))
  
  def pop(self):
    return heapq.heappop(self.queue)
  
  def is_empty(self):
    # print(self.q)
    return len(self.queue) == 0

class MapTraffic:
  def __init__(self, atlas, walls, material):
    self.n = len(A)
    self.m = len(A[0])
    self.atlas = A
    self.walls = walls
    self.material = material

  def in_bounds(self, p):
    x,y  = p
    return x >=0 and y>=0 and x<self.n and y<self.m

  def passable(self, p):
    for wall_pos in self.walls:
      if wall_pos == p:
        return False
    return True
  
  def neighbors(self, p):
    x, y = p
    neighbors = [(x+1,y),(x-1,y),(x,y+1),(x,y-1)]
    valid_neighbors = []
    for pos in neighbors:
      if self.in_bounds(pos) and self.passable(pos):
        valid_neighbors.append(pos)
    return valid_neighbors

  def draw(self, show_weight=False, path=[]):
    for i in range(self.n):
      for j in range(self.m):
        if (i,j) in path:
          print("x",end="")
        elif (i, j) in self.material:
            if show_weight:
            print("!", end="\t")
          else:
            print("!", end="")
        elif self.passable((i,j)):
          if show_weight:
            print(f"{self.A[i][j]}",end="\t")
          else:
            print("_", end="")
        else:
          if show_weight:
            print("#", end="\t")
          else:
            print("#", end="")
      print()


class SearchAlg:
  def __init__(self, grid, start, goal):
    self.grid = grid
    self.start = start
    self.goal = goal
    self.came_from = {}

  def trace_path(self):
    #TODO
    pass

  def heuristic(self,p1, p2, heu_type="Manhanttan"):
    #TODO
    pass

  def a_star(self):
    #TODO
    pass

  def BFS(self):
    #TODO
    pass
  
  def DFS(self,):
    #TODO
    pass

  def UCS(self):
    #TODO
    pass


