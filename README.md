# A* Path Finding Algorithm for 2D Grid World
## AIM
To develop a code to find the route from the source to the destination point using A* algorithm for 2D grid world.

## THEORY
A* works by making a lowest-cost path tree from the start node to the target node using heuristic function.A* search is a combination of lowest-cost-first and best-first searches that considers both path cost and heuristic information in its selection of which path to expand.A* uses a function f (n)= g(n)+h(n) that gives an estimate of the total cost of a path using that node.

## DESIGN STEPS

### STEP 1:
Import the required packages.

### STEP 2:
Create a class with the appropriate functions.

### STEP 3:
Build a 2D grid world with initial state , goal state and obstacles.

### STEP 4:
Define a method for heuristic function.

### STEP 5:
Pass the values for the grid problem.

### STEP 6:
Print the solution.

## Draw the 2D 
![Capture3](https://user-images.githubusercontent.com/75234588/168871080-ebc9b855-a54b-4e76-8397-e9b8c109dc73.PNG)


## PROGRAM
```python
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
failure = Node('failure', path_cost=math.inf) # Indicates an algorithm couldn't find a solution.
cutoff  = Node('cutoff',  path_cost=math.inf) # Indicates iterative deepening search was cut off.
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
        x,y = state
        return {(x+dx,y+dy) for(dx,dy) in self.directions}-self.obstacles
   
def straight_line_distance(A, B):
    "Straight-line distance between two points."
    return sum(abs(a-b)**2 for (a,b) in zip(A,B))**0.5

def g(n): 
    return n.path_cost
def astar_search(problem, h=None):
    """Search nodes with minimum f(n) = g(n) + h(n)."""
    h = h or problem.h
    return best_first_search(problem, f=lambda n: g(n) + h(n))
grid1 = GridProblem(initial=(1,2), goal =(7,3) ,obstacles={(1,5),(2,3),(3,4),(4,2),(5,1),(6,3),(7,5),(8,3),(9,2),(10,1)})
solution1 = astar_search(grid1)
a=path_states(solution1)
print(a)
```

## OUTPUT:
![Capture4](https://user-images.githubusercontent.com/75234588/168871051-5fb4fd14-64f9-4487-8c42-fb25eecb98a7.PNG)


## SOLUTION JUSTIFICATION:
A* combines the advantages of Best-first Search and Uniform Cost Search: ensure to find the optimized path while increasing the algorithm efficiency using heuristics.Complexity in A* Search is that the Algorithm doesn’t produce the shortest path always, as it relies heavily on heuristics / approximations to calculate h.

## RESULT:
Hence, A* Search Algorithm was implemented for path finding from the source to the destination point in 2D grid world.
