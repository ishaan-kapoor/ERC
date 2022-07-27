import matplotlib.pyplot as plt
import random
from math import sqrt
from shapely.geometry import Point, Polygon, LineString


class Shape:
    def __init__(self, vertices):
        self.vertices = vertices
        self.polygon = Polygon(vertices)

def valid_point(point, connection_node, obstacles):
    for obstacle in obstacles:
        # Check if point is inside any obstacle
        if obstacle.polygon.contains(Point(*point)): return False
        # Check if the line through (node and point) passes through any obstacle
        for i in range(len(obstacle.vertices)-1):
            obstacle_line = LineString(obstacle.vertices[i : i+2])
            connection_line = LineString((point, connection_node))
            if connection_line.intersects(obstacle_line): return False
    return True

def append_to_tree(point, min_index, tree, goal):
    tree.append((*point, min_index))
    return goal.polygon.contains(Point(*point))

def generate_point(count, bounds, goal, goal_sampling_freq):
    min_x, min_y, max_x, max_y = goal.polygon.bounds if (count%goal_sampling_freq == 0) else (0, 0, *bounds)
    point = [random.uniform(min_x, max_x), random.uniform(min_y, max_y)]
    return point

def RRT(start, goal_point, obstacle_list):
    # Variables
    bounds = (100, 100)
    fixed_dist_squared = 4  # square of radius in which the next node should lie
    goal_sampling_freq = 5  # Every 5th point is sampled in the goal region
    count = 0  # just keeping a count the number of loops, so as to sample every 5th point in the goal region
    found = False

    # initializing obstacles
    obstacles = []
    for obst in obstacle_list:
        obst.append(obst[0])
        obstacles.append(Shape(obst))

    goal = Shape(  # goal region is a square of side 2 and a corner as goal point
        [
            (goal_point[0], goal_point[1]),
            (goal_point[0], goal_point[1] - 2),
            (goal_point[0] - 2, goal_point[1] - 2),
            (goal_point[0] - 2, goal_point[1]),
        ]
    )
    tree = [(*start, 0)]  # initializing tree with start point

    # Main loop of RRT
    while not found:
        # Sampling point
        point = generate_point(count, bounds, goal, goal_sampling_freq)
        count += 1

        # Calculating the point in the tree which has minimum distance from sampled point
        min_dist_squared = float('inf')
        min_index = -1

        for index, node in enumerate(tree):
            curr_dist_squared = (point[0]-node[0]) ** 2 + (point[1]-node[1]) ** 2
            if curr_dist_squared < min_dist_squared:
                min_index = index
                min_dist_squared = curr_dist_squared

        # Changing point if the point has a distance greater than fixed_distance from the nearest node
        min_index_node = tree[min_index]
        if min_dist_squared > fixed_dist_squared:
            point[0] = min_index_node[0] + (point[0]-min_index_node[0]) * sqrt(fixed_dist_squared/min_dist_squared)
            point[1] = min_index_node[1] + (point[1]-min_index_node[1]) * sqrt(fixed_dist_squared/min_dist_squared)

        if not valid_point(point, min_index_node, obstacles): continue

        # Add valid point in tree and check if goal is found or not
        found = append_to_tree(point, min_index, tree, goal)
        # if the goal has been found, add the final goal point to the tree
        if found:
            append_to_tree(goal_point, len(tree) - 1, tree, goal)

    return tree


def visualize(tree, obstacle_list):
    # Drawing obstacles
    for obstacles in obstacle_list:
        obstacles.append(obstacles[0])
        plt.plot(*zip(*obstacles))

    # Plotting and connecting nodes with respective parent nodes in tree
    for node in tree:
        plt.plot((node[0], tree[node[2]][0]), (node[1], tree[node[2]][1]), "r.-", markersize=2, linewidth=0.2)


    index = len(tree) - 1  # starting from goal
    while index != 0:
        parent_index = tree[index][2]
        current, parent = tree[index], tree[parent_index]
        plt.plot((current[0], parent[0]), (current[1], parent[1]), "b.-", markersize=6, linewidth=0.6)
        index = parent_index

    plt.show()


if __name__ == '__main__':
    obstacle_list = [
        [(40, 0), (40, 40), (50, 50), (60, 40), (50, 40)],
        [(10, 10), (20, 20), (10, 30), (0, 20)],
        [(50, 60), (70, 80), (60, 100), (40, 80), (45, 100)],
        [(70, 20), (90, 20), (80, 40)],
    ]
    start = (1, 1)
    goal = (100, 1)
    # Calculate Path using RRT
    path = RRT(start, goal, obstacle_list)
    # Visualize the calculated path in Matplotlib
    visualize(path, obstacle_list)
