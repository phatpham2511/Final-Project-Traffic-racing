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
    curr =  self.goal
    path = []
    while curr != self.start:
      path.append(curr)
      curr = self.came_from[curr]
    
    path.append(self.start)
    path.reverse()
    return path

  def heuristic(self,p1, p2, heu_type="Manhanttan"):
      return abs(p1[0]-p2[0]) + abs(p1[1]-p2[1])

  def a_star(self):
    open_list = PriorityQueue()
    gScore = {self.start: 0}  # lưu giá trị G của mỗi đỉnh
    fScore_start = self.heuristic(self.start, self.goal) # f = g + h = 0 + heu(start, goal)
    open_list.push(fScore_start, self.start) # push(value, label_node)
    self.came_from = {} # dùng để lưu dấu đường đi

    while not open_list.is_empty():
        curr = open_list.pop()  # lấy đỉnh curr có fScore nhỏ nhất
        # print(curr) # trả về (curr_fScore, curr_node)
        curr_fScore, curr_node = curr
        if curr_node == self.goal:
            print(colored("Finded path!", "green"))
            path = self.trace_path()
            self.grid.draw(path=path)
            return True
        for next_node in self.grid.neighbors(curr_node):
            new_g = gScore[curr_node] + self.grid.A[next_node[0]][next_node[1]]  # next_g = curr_g + A[curr_node->next_node]
            if (next_node not in gScore) or (new_g < gScore[next_node]): 
                gScore[next_node] = new_g
                fScore_next_node = gScore[next_node] + self.heuristic(next_node, self.goal)  # Khác với UCS là có thêm hàm Heuristic ở đây!
                open_list.push(fScore_next_node, next_node)
                self.came_from[next_node] = curr_node

        print(f"After search at f{curr_node}: f{open_list.queue}")
    
    print(colored("Can not find path.", "red"))
    return False

  def BFS(self):
    queue=[self.start]
    check = [[0 for x in range(self.grid.m)] for y in range(self.grid.n)] 
    for i in range(self.grid.m):
      for j in range(self.grid.n) :
        check[j][i]=False
    check[self.start[0]][self.start[1]]=True
    self.came_from = {}
    while len(queue)>0 :
      curr_node = queue.pop(0)
      if  curr_node == self.goal :
            print(colored("Finded path!", "green"))
            path = self.trace_path()
            self.grid.draw(path=path)
            return True
      for next_node in self.grid.neighbors(curr_node) :        
        if check[next_node[0]][next_node[1]] is False :
          check[next_node[0]][next_node[1]]=True
          queue.append(next_node)
          self.came_from[next_node] = curr_node
    print(colored("Can not find path.", "red"))
    return False
  
  def DFS(self,):
    stack=[self.start]
    check = [[0 for x in range(self.grid.m)] for y in range(self.grid.n)] 
    for i in range(self.grid.m):
      for j in range(self.grid.n) :
        check[j][i]=False
    check[self.start[0]][self.start[1]]=True
    self.came_from = {}
    while len(stack)>0 :
      curr_node = stack.pop()
      if  curr_node == self.goal :
            print(colored("Finded path!", "green"))
            path = self.trace_path()
            self.grid.draw(path=path)
            return True
      for next_node in self.grid.neighbors(curr_node) :        
        if check[next_node[0]][next_node[1]] is False :
          check[next_node[0]][next_node[1]]=True
          stack.append(next_node)
          self.came_from[next_node] = curr_node
    print(colored("Can not find path.", "red"))
    return False

  def UCS(self):
    pqueue = []# priority_queue. Khai báo hàng đợi ưu tiên <chi phí, đỉnh>
        heapq.heappush(pqueue, (0, self.start)) # Thêm vào cấu trúc vung đống Heap. chi phí của đỉnh đầu tiên strart=0
        cost_so_far = {self.start: 0} # chi phí đến đỉnh hiện tại -> start -> chi phí =0
        while len(pqueue) > 0: # Khi hàng đợi không rỗng
          curr_n = heapq.heappop(pqueue) # Lấy ra đỉnh tiếp theo trong cấu trúc Heap <chi phí, đỉnh>
          curr_cost, curr_node = curr_n  #Tuple<chi phí, đỉnh> = curr. curr_cost-> chi phí, curr_node -> đỉnh đang xét
          if curr_node == self.goal: # Khi đỉnh đang xét là goal -> Tìm thấy đường đi
            print(colored("Finded path!", "green"))
            path = self.trace_path()
            self.grid.draw(path=path)
            return True
          for next_node in self.grid.neighbors(curr_node): # Duyệt lần lượt các định còn lại
              # Tạo ra chi phí mới = Tổng chi phí hiện tại + Chi phí của node tiếp theo
              new_cost =  curr_cost + self.grid.A[next_node[0]][next_node[1]]
            # Nếu đỉnh đang xét chưa được tính chi phí HOẶC có chi phí nhỏ hơn chi phí đang xét 
              if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                heapq.heappush(pqueue, (new_cost, next_node)) # đưa chi phí mới vào Heap
                cost_so_far[next_node] = new_cost # cập nhật chi phí mới cho đỉnh đang xét
                self.came_from[next_node] = curr_node
        # Không tìm thấy đỉnh = goal 
        print(colored("Can not find path.", "red"))
        return False


