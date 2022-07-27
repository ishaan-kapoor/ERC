# Path Planning using RRT

### Description
This program finds a path in a 2D array between 2 given points in the presence of obstacles and shows it too.

### Setup and running
* `pip install matplotlib, shapely`
* `python main.py`

### Path Planning
Defined as, finding a geometrical path from a given point to another such that it avoids obstacles, path planning appears really simple.
But, it is one of the most trivial problem to solve in autonomous navigation.

Path planning is of 2 types:
* Global Path Planning, where the surrounding environment is globally known (i.e. a map is available).
* Local Path Planning, where the local surroundings are reconstructed based on data from sensors, etc.

I implimented global path planning using RRT algorithm.

### Rapidly-exploring Random Trees (RRT)
It is a path planning algorithm which is popular despite it not necessarily providing an optimal solution (path).
In this algorithm, random points are generated and connected (avoiding the obstacles) to the closest node to them, provided the itself vertex does not lie within an obstacle.
When a generated node is within the goal region, end point is reached for the algorithm.

### Shapely
* Python package for manipulation and analysis of planar geometric objects.
* https://pypi.org/project/Shapely/
