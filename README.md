# A* Path Finding Algorithm for 2D Grid World
## AIM

To develop a code to find the route from the source to the destination point using A* algorithm for 2D grid world.

## THEORY
The A∗ algorithm is the most commonly used heuristic graph search algorithm for state space. In addition to solving problems based on state space, it is often used for the path planning of robots.
Algorithms of global path planning are mainly divided into two types: heuristic search methods and intelligent algorithms. The initial representation of the heuristic search is the A∗ algorithm developed by the Dijkstra algorithm. The A∗ algorithm is the most commonly used heuristic graph search algorithm for state space. 

## DESIGN STEPS

### STEP 1:
Build a 2D grid world with initial state , goal state and obstacles.

### STEP 2:
Within the execution introduce a class named GridProblem to find a path on a 2D grid with obstacles.

### STEP 3:
Calculate the straight line difference between the two points.

### Step 4:
Draw the pathbased on the plan given by get ridding the obstacles that is minimum in distance.

## Draw the 2D 
![out](https://user-images.githubusercontent.com/75235402/168860120-9f017d9f-d4d6-4e00-adee-c84bf603d0d2.jpg)



## PROGRAM
```
/*
Developed by : A.Divya Meenakshi
Registration Number: 212220230014
*/
```
```
%matplotlib inline
import matplotlib.pyplot as plt
import random
import math
import sys
from collections import defaultdict, deque, Counter
from itertools import combinations
import heapq

class Problem(object):
    """The abstract class for a formal problem. A new domain subclasses this,
    overriding `actions` and `results`, and perhaps other methods.
    The default heuristic is 0 and the default action cost is 1 for all states.
    When yiou create an instance of a subclass, specify `initial`, and `goal` states 
    (or give an `is_goal` method) and perhaps other keyword args for the subclass."""

    def __init__(self, initial=None, goal=None, **kwds): 
        self.__dict__.update(initial=initial, goal=goal, **kwds) 
        
    def actions(self, state):        
        raise NotImplementedError
    def result(self, state, action): 
        raise NotImplementedError
    def is_goal(self, state):        
        return state == self.goal
    def action_cost(self, s, a, s1): 
        return 1
    
    def __str__(self):
        return '{0}({1}, {2})'.format(
            type(self).__name__, self.initial, self.goal)
            
class Node:
    "A Node in a search tree."
    def __init__(self, state, parent=None, action=None, path_cost=0):
        self.__dict__.update(state=state, parent=parent, action=action, path_cost=path_cost)

    def __str__(self): 
        return '<{0}>'.format(self.state)
    def __len__(self): 
        return 0 if self.parent is None else (1 + len(self.parent))
    def __lt__(self, other): 
        return self.path_cost < other.path_cost
        
failure = Node('failure', path_cost=math.inf) 
cutoff  = Node('cutoff',  path_cost=math.inf) 

def expand(problem, node):
    "Expand a node, generating the children nodes."
    s = node.state
    for action in problem.actions(s):
        s1 = problem.result(s, action)
        cost = node.path_cost + problem.action_cost(s, action, s1)
        yield Node(s1, node, action, cost)
        

def path_actions(node):
    "The sequence of actions to get to this node."
    if node.parent is None:
        return []  
    return path_actions(node.parent) + [node.action]


def path_states(node):
    "The sequence of states to get to this node."
    if node in (cutoff, failure, None): 
        return []
    return path_states(node.parent) + [node.state]

class PriorityQueue:
    """A queue in which the item with minimum f(item) is always popped first."""

    def __init__(self, items=(), key=lambda x: x): 
        self.key = key
        self.items = [] # a heap of (score, item) pairs
        for item in items:
            self.add(item)
         
    def add(self, item):
        """Add item to the queuez."""
        pair = (self.key(item), item)
        heapq.heappush(self.items, pair)

    def pop(self):
        """Pop and return the item with min f(item) value."""
        return heapq.heappop(self.items)[1]
    
    def top(self): return self.items[0][1]

    def __len__(self): return len(self.items)
    
def best_first_search(problem, f):
    "Search nodes with minimum f(node) value first."
    node = Node(problem.initial)
    frontier = PriorityQueue([node], key=f)
    reached = {problem.initial: node}
    while frontier:
        node = frontier.pop()
        if problem.is_goal(node.state):
            return node
        for child in expand(problem, node):
            s = child.state
            if s not in reached or child.path_cost < reached[s].path_cost:
                reached[s] = child
                frontier.add(child)
    return failure

def g(n): 
    return n.path_cost
    
class GridProblem(Problem):
    """Finding a path on a 2D grid with obstacles. Obstacles are (x, y) cells."""

    def __init__(self, initial=(15, 30), goal=(130, 30), obstacles=(), **kwds):
        Problem.__init__(self, initial=initial, goal=goal, 
                         obstacles=set(obstacles) - {initial, goal}, **kwds)

    directions = [(-1, -1), (0, -1), (1, -1),
                  (-1, 0),           (1,  0),
                  (-1, +1), (0, +1), (1, +1)]
    
    def action_cost(self, s, action, s1): 
        return straight_line_distance(s, s1)
    
    def h(self, node): 
        return straight_line_distance(node.state, self.goal)
                  
    def result(self, state, action): 
        "Both states and actions are represented by (x, y) pairs."
        return action if action not in self.obstacles else state
    
    def actions(self, state):
        """You can move one cell in any of `directions` to a non-obstacle cell."""
        
        x, y = state
        return {(x + dx, y + dy) for (dx,dy) in self.directions} - self.obstacles
        
def straight_line_distance(A, B):
    "Straight-line distance between two points."
    return sum(abs(a - b)**2 for (a,b) in zip(A, B)) ** 0.5 
    
def g(n): 
    return n.path_cost
    
def astar_search(problem, h=None):
    """Search nodes with minimum f(n) = g(n) + h(n)."""
    h = h or problem.h
    return best_first_search(problem, f=lambda n: g(n) + h(n))
    
grid1 = GridProblem(initial=(4,3), goal =(1,6) ,obstacles={(1,2),(1,10),(2,4),(2,7),(3,1),(3,6),(3,10),(4,2),(5,4),(5,8)})
solution1 = astar_search(grid1)
path_states(solution1)
```

## OUTPUT:
![code](https://user-images.githubusercontent.com/75235402/168860058-b7c8e765-e81c-4b89-9ea3-fd8411d31686.jpg)



The Completeness and complexity of the algorithm:
 A* algorithm is a heuristic function based algorithm for proper path planning. It calculates heuristic function's value at each node on the work area and involves the checking of too many adjacent nodes for finding the optimal solution with zero probability of collision. Hence, it takes much processing time and decreases the work speed. 

## RESULT:
Thus, the algorithm to find the route from the source to the destination point using A* algorithm for 2D grid world.
