import numpy as np                    # Used for functions with number arrays
import matplotlib.pyplot as plt       # Used for plotting
import matplotlib.patches as patches  # Used for plotting
import math                           # Used for math equations
import time                           # Used for code timer

# User Inputs and Initial Conditions
inflation = 0
x_start = (100, 100)
x_goal = (700, 400)
x_parent_start = x_start
map_size = (800, 800)                   # Define the size of the map
max_iter = 4000                         # iterations of sampling


cost_total_initial = 0                  # Initial Cost
child_parent = {}                       # dictionary to track parent
cost_total = {}                         # dictionary to track total cost of node
child_parent[x_start] = x_parent_start  # parent and child dictionary
cost_total[x_start] = 0                 # node and cost dictionary
node_tracker = [(x_parent_start, x_start, cost_total_initial)]  # node tracker for all info want to track


# Calculate distance (c2c) between two nodes.
def c2c(previous_node, current_node):
    current_cost = math.sqrt(((current_node[0] - previous_node[0]) ** 2) + ((current_node[1] - previous_node[1]) ** 2))
    cost_to_come = current_cost
    return cost_to_come

# Randomly sample a point within the map
def sample_free(map_size):
    x = np.random.uniform(0, map_size[0])
    y = np.random.uniform(0, map_size[1])
    return x, y

# Move towards sampled point
def move_towards(from_point, to_point, step_size):
    distance = np.linalg.norm(np.array(from_point) - np.array(to_point))
    if distance <= step_size:
        return to_point
    else:
        direction = np.array(to_point) - np.array(from_point)
        new_point = np.array(from_point) + direction * step_size
        return tuple(new_point)

# Find the nearest node in the tree to the given node
def nearest(node, tree):
    nearest_node = None
    min_dist = float('inf')
    for n in tree:
        dist = np.sqrt((node[0] - n[0])**2 + (node[1] - n[1])**2)
        if dist < min_dist:
            min_dist = dist
            nearest_node = n
    return nearest_node

# Find the nearest lowest cost to come node with following two functions
def find_near_nodes(sample, tree, max_distance):
    near_nodes = []
    for node in tree:
        dist = np.linalg.norm(np.array(sample) - np.array(node))
        if dist <= max_distance:
            near_nodes.append(node)
    return near_nodes

def find_parent_in_threshold(sample, near_nodes):
    min_cost = float('inf')
    min_cost_node = None
    for node in near_nodes:
        cost = cost_total[node] + np.linalg.norm(np.array(sample) - np.array(node))
        if cost < min_cost:
            min_cost = cost
            min_cost_node = node
    return min_cost_node


# Static obstacle function
def obst(x, y):
    if 380 - inflation <= x <= 420 + inflation and 150 - inflation <= y <= 350:  # Rectangle 1
        return True
    elif 380 - inflation <= x <= 420 + inflation and 450 - inflation <= y <= 650:  # Rectangle 2
        return True
    elif 180 - inflation <= x <= 220 + inflation and 300 - inflation <= y <= 500:  # Rectangle 3
        return True
    elif 700 - inflation <= x <= 800 + inflation and 0 <= y <= 100 + inflation:
        return True
    elif 0 - inflation <= x <= 210 + inflation and 150 <= y <= 170:
        return True
    elif 190 - inflation <= x <= 220 + inflation and 50 <= y <= 170:
        return True
    elif x <= 5:
        return True
    elif x >= 795:
        return True
    elif y <= 5:
        return True
    elif y >= 795:
        return True
    else:
        return False

# dynamic obstacles and movement function
dynamic_obstacles = [(200, 400, 1, 0), (500, 200, -1, 0), (300, 600, 0, -1), (100, 100, 0, 1), (600, 500, 1, 1)]  # (x, y, velocity_x, velocity_y)

def update_dynamic_obstacles():
    for obstacle in dynamic_obstacles:
        x, y, vx, vy = obstacle
        x += vx  # Update x position based on velocity
        y += vy  # Update y position based on velocity
        obstacle = (x, y, vx, vy)

# Perform collision check between two nodes
def collision_check(node1, node2):
    x1, y1 = node1
    x2, y2 = node2
    step = 0.1  # Adjust step size based on your preference
    num_steps = int(np.ceil(max(abs(x2 - x1), abs(y2 - y1)) / step))
    for i in range(num_steps + 1):
        x = x1 + i * (x2 - x1) / num_steps
        y = y1 + i * (y2 - y1) / num_steps
        if obst(x, y):
            return True
    # Check against dynamic obstacles
    for obstacle in dynamic_obstacles:
        obs_x, obs_y, _, _ = obstacle
        obs_center_x = obs_x + 25
        obs_center_y = obs_y + 25
        dist_to_segment = abs((y2 - y1) * obs_center_x - (x2 - x1) * obs_center_y + x2 * y1 - y2 * x1) / np.sqrt((y2 - y1)**2 + (x2 - x1)**2)
        if dist_to_segment < 25:
            projection = ((obs_center_x - x1) * (x2 - x1) + (obs_center_y - y1) * (y2 - y1)) / ((y2 - y1)**2 + (x2 - x1)**2)
            if 0 <= projection <= 1:
                return True

    return False


# Insert a new node into the tree
def insert_node(new_node, tree):
    tree.append(new_node)
    return tree

# Rewire the tree based on the new node
def rewire(new_node, tree):
    near_nodes_rewire = find_near_nodes(new_node, tree, 5)
    for node in near_nodes_rewire:
        new_cost_check = cost_total[node] + np.linalg.norm(np.array(new_node) - np.array(node))
        if new_cost_check < cost_total[node]:
            child_parent[new_node] = node
            cost_total[new_node] = new_cost_check
    return tree

# Main Algorithm
T = [x_start]
not_find_stop = True
iter_count = 0
max_distance = 20   # threshold for nearest node search, can edit this (want to be relatively small)
start_time = time.time()
while not_find_stop and iter_count < max_iter:
    update_dynamic_obstacles()  # Update dynamic obstacles
    x_rand = sample_free(map_size)   # random x,y point
    # x_nearest = nearest(x_rand, T)   # nearest point to random x,y point
    near_nodes = find_near_nodes(x_rand, T, max_distance)
    x_nearest = find_parent_in_threshold(x_rand, near_nodes)
    if x_nearest is None:
        x_nearest = nearest(x_rand, T)
    x_rand_new = move_towards(x_nearest, x_rand, .1) # move towards the sampled point in a step

    if collision_check(x_nearest, x_rand_new): # if in obstacle space then sample new point
        continue

    x_new = x_rand_new
    T = insert_node(x_new, T)
    current_cost_segment = c2c(x_nearest, x_new)
    current_cost_node = current_cost_segment + cost_total[x_nearest]
    child_parent[x_new] = x_nearest
    cost_total[x_new] = current_cost_node
    node_tracker.append((x_nearest, x_new, current_cost_node))
    T = rewire(x_new, T)
    if math.fabs(x_new[0] - x_goal[0]) <= 15 and math.fabs(x_new[1] - x_goal[1]) <= 15:  # goal_find(x_goal, x_new):
        print('Goal Located!!')
        print("continuing to run through defined number of iterations...")
        back_track_node_start = x_new
        # break
        # not_find_stop = False

    iter_count += 1

def triangle_decomp(path):
    simplified_path = [path[0]]  # Initialize with the start point
    start = 0
    end = len(path) - 1
    while start < end:  # run loop until end of original path index is reached
        mid = end # Start at the end of the path index
        while mid > start + 1:  # run until mid-index is back to the start
            if not collision_check(path[start], path[mid]):  # find a point that doesn't intersect with obstacle
                break
            mid -= 1 # backtrack by 1 from end
        simplified_path.append(path[mid])  # add point to simplified path
        start = mid  # Start again with last added point index
    simplified_path.append(path[-1])  # Include the end point
    return simplified_path


def catmull_rom_interpolation(simplified_path, num_points=100):
    interpolated_path = []
    for i in range(len(simplified_path) - 3):
        p0, p1, p2, p3 = simplified_path[i:i+4]
        for t in np.linspace(0, 1, num_points):
            x = 0.5 * (
                (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t**3 +
                (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t**2 +
                (-p0[0] + p2[0]) * t +
                (2 * p1[0])
            )
            y = 0.5 * (
                (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t**3 +
                (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t**2 +
                (-p0[1] + p2[1]) * t +
                (2 * p1[1])
            )
            interpolated_path.append((x, y))
    interpolated_path.append(simplified_path[-2])  # Add the second last point to avoid discontinuity
    return interpolated_path

# Back Tracking for Optimal Path
back_track = []
try:
    child = back_track_node_start

    # Search dictionary for backward path
    while tuple(child) is not tuple(x_start):
        if tuple(child) == tuple(x_start):
            break
        back_track.append(child)
        parent = child_parent[tuple(child)]
        child = tuple(parent)

    forward_path = back_track[::-1]  # reverse back_track to get forward path
    end_time = time.time()
    XS = [x[0] for x in forward_path]
    YS = [y[1] for y in forward_path]
    print("Original Path", forward_path)
    print("Original Path Size", len(forward_path))
    total_cost_to_come = 0
    for i in range(len(forward_path) - 1):
        p1 = forward_path[i]
        p2 = forward_path[i+1]
        total_cost_to_come += np.linalg.norm(np.array(p2)-np.array(p1))
    print("Original Path Cost", total_cost_to_come)
    print("Original Path Time", end_time - start_time)

    triangle_decomposition_path = triangle_decomp(forward_path)
    end_time_1 = time.time()
    XTD = [x[0] for x in triangle_decomposition_path]
    YTD = [y[1] for y in triangle_decomposition_path]
    print("Triangle Decomp Path", triangle_decomposition_path)
    print("Triangle Decomp Path Size", len(triangle_decomposition_path))
    total_cost_to_come_1 = 0
    for i in range(len(triangle_decomposition_path) - 1):
        p1 = triangle_decomposition_path[i]
        p2 = triangle_decomposition_path[i+1]
        total_cost_to_come_1 += np.linalg.norm(np.array(p2)-np.array(p1))
    print("Triangle Decomp Path Cost", total_cost_to_come_1)
    print("TD Path Time", end_time_1 - start_time)

    interpolated_path = catmull_rom_interpolation(triangle_decomposition_path, num_points=100)
    end_time_2 = time.time()
    XTDS = [x[0] for x in interpolated_path]
    YTDS = [y[1] for y in interpolated_path]
    print("Smoothed Path", interpolated_path)
    print("Smoothed Path Size", len(interpolated_path))
    total_cost_to_come_2 = 0
    for i in range(len(interpolated_path) - 1):
        p1 = interpolated_path[i]
        p2 = interpolated_path[i+1]
        total_cost_to_come_2 += np.linalg.norm(np.array(p2)-np.array(p1))
    print("Smoothed Path Cost", total_cost_to_come_2)
    print("Smoothed Path Time", end_time_2 - start_time)

    def plot_tree(tree_dict):
        for parent, child in tree_dict.items():
            px, py = parent
            cx, cy = child
            ax.plot([px, cx], [py, cy], color='blue')  # Plotting the edge between parent and child
            ax.plot(px, py, 'co', markersize=1)  # Plotting the parent node
            ax.plot(cx, cy, 'co', markersize=1)  # Plotting the child node
        plt.axis('equal')


    def plot_dynamic_obstacles(obstacles):
        for obstacle in obstacles:
            x, y, _, _ = obstacle
            rect = patches.Rectangle((x, y), 50, 50, linewidth=1, edgecolor='r', facecolor='r', alpha=0.5)
            ax.add_patch(rect)



    fig, ax = plt.subplots()
    ax.set_xlim([0, 800])
    ax.set_ylim([0, 800])
    rectangle = plt.Rectangle((380, 150), 40, 200, fc='black')
    rectangle_1 = plt.Rectangle((380, 450), 40, 200, fc='black')
    rectangle_2 = plt.Rectangle((180, 300), 40, 200, fc='black')
    rectangle_3 = plt.Rectangle((700, 0), 100, 100, fc='black')
    rectangle_4 = plt.Rectangle((190, 50), 30, 120, fc='black')
    rectangle_5 = plt.Rectangle((0, 150), 210, 20, fc='black')
    plt.plot(x_start[0], x_start[1], markersize=5, color="m", label="Start", marker='s', zorder=3)  # Plotting the parent node
    plt.plot(x_goal[0], x_goal[1], markersize=5, color="g", label="Goal", marker='s', zorder=3)  # Plotting the child node
    #
    plt.gca().add_patch(rectangle)
    plt.gca().add_patch(rectangle_1)
    plt.gca().add_patch(rectangle_2)
    plt.gca().add_patch(rectangle_3)
    plt.gca().add_patch(rectangle_4)
    plt.gca().add_patch(rectangle_5)

    plot_dynamic_obstacles(dynamic_obstacles)
    plot_tree(child_parent)
    graph_1, = plt.plot([], [], color="g", label="RRT*")
    graph_1.set_data(XS, YS)
    graph_2, = plt.plot([], [], color="r", label="TD-RRT*")
    graph_2.set_data(XTD, YTD)
    graph_3, = plt.plot([], [], color="y", label="Smooth TD-RRT*")
    graph_3.set_data(XTDS, YTDS)
    plt.title('RRT* vs TD-RRT* vs Smooth TD-RRT*')
    plt.legend(loc='upper right')
    plt.grid()
    plt.show()

except NameError:
    print("Sorry, No Goal was found. Run again!")