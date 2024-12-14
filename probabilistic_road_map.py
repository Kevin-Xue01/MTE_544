"""

Probabilistic Road Map (PRM) Planner

This version of the PRM implementation creates a connected graph by creating N_SAMPLES random samples in the free space,
then each sample is connected to max N_KNN neighbors, with a maximum distance of MAX_EDGE_LEN.
The PRM is implemented as follows:
- Parameters that need to be set:
  - N_SAMPLE = number of samples to create in the free space
  - N_KNN = number of neighbors to connect from each sample
  - MAX_EDGE_LEN = maximum distance for creating an edge between two samples
- Sample points are creates with the generate_sample_points function. The samples are randomly created in the free space,
  based on the list of obstacles, and the dimension of the robot. The samples should be at least radius of the robot distance away from any obstacles. 
- Create the roadmap using the generated sample_points, with the generate_road_map function. The roadmap is generated by looping
  over all samples. For each sample, N_KNN edges are created with neighboring samples. In creating the edge, a collision is checked with
  the is_collision function, to create edges that are not overlaying on obstacles. To introduce further control, a MAX_EDGE_LEN is imposed
  so that connected samples (nodes) are not too far from each other.

This code can run standalone without being integrated into the stack with control and navigation: python probabilistic_road_map.py
To do so, set use_map to False. This will help debugging your PRM implementation. It will use the obstacles list, start, and goal 
set in the main function.

Once it is integrated into the stack, set use_map to True and run everything from decisions.py as you have done in the labs.
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
from mapUtilities import *

# Parameters of PRM
N_SAMPLE = 1000  # number of sample_points
N_KNN = 10  # number of edge from one sampled point (one node)
MAX_EDGE_LEN = 5 # Maximum edge length, in [m]
ROBOT_RADIUS = 0.2 # Robot radius, in [m]
show_plot = False

# When set to false, you can run this script stand-alone, it will use the information specified in main
# When set to true, you are expected to use this with the stack and the specified map
use_map = True

def prm_graph(start, goal, obstacles_list, robot_radius, *, rng=None, m_utilities: mapManipulator=None):
    """
    Run probabilistic road map graph generation

    :param start: start x,y position
    :param goal: goal x,y position
    :param obstacle_list: obstacles x and y positions
    :param robot_radius: robot radius
    :param rng: (Optional) Random generator
    :pram m_utilities: when using a map, pass the map utilies that contains the costmap information
    :return: roadmap (and list of sample points if using the map)
    """
    print("Generating PRM graph")

    obstacle_kd_tree = KDTree(obstacles_list)

    # By default, we use the set maximum edge length
    max_edge_len = MAX_EDGE_LEN

    # Generate the sample points
    if use_map:
        # [Part 2] TODO The radius of the robot and the maximum edge lengths are given in [m], but the map is given in cell positions.
        # Therefore, when using the map, the radius and edge length need to be adjusted for the resolution of the cell positions
        # Hint: in the map utilities there is the resolution stored

        # Adjust the robot radius and max edge length based on the map resolution
        resolution = m_utilities.getResolution()
        robot_radius /= resolution
        max_edge_len /= resolution

    # Get sample data
    sample_points = generate_sample_points(start, goal,
                                       robot_radius,
                                       obstacles_list,
                                       obstacle_kd_tree, rng)

    # Create the roadmap
    if use_map:
        roadmap = generate_road_map(sample_points, robot_radius, obstacle_kd_tree, max_edge_len, m_utilities)
    else:
        roadmap = generate_road_map(sample_points, robot_radius, obstacle_kd_tree, max_edge_len)

    if show_plot:
        if use_map:
            # When using the map, first convert cells into positions, then plot (for a more intuitive visualization)
            # Plot the sample points
            samples_pos = np.array([m_utilities.cell_2_position([i, j]) for i, j in zip(sample_points[0], sample_points[1])])
            print(samples_pos)
            sample_x = samples_pos[:, 0]
            sample_y = samples_pos[:, 1]
            plt.plot(sample_x, sample_y, ".b")   
            # Plot list of obstacles
            obs_pos = np.array([m_utilities.cell_2_position([i, j]) for i, j in zip(obstacles_list[:, 0], obstacles_list[:, 1])])
            obs_x = obs_pos[:, 0]
            obs_y = obs_pos[:, 1]
            plt.plot(obs_x, obs_y, ".k")  
            # Plot the roadmap
            plot_road_map(roadmap, [sample_x, sample_y])
            # Plot the starting position as a red marker
            s_pos = m_utilities.cell_2_position(start)
            plt.plot(s_pos[0], s_pos[1], "^r")
            # Plot the goal position as a green marker
            g_pos = m_utilities.cell_2_position(goal)
            plt.plot(g_pos[0], g_pos[1], "^g")
        else:
            # plot the sample points
            plt.plot(sample_points[0], sample_points[1], ".b")   
            # Plot list of obstacles
            plt.plot(obstacles_list[:,0], obstacles_list[:,1], ".k")  
            # Plot the roadmap
            plot_road_map(roadmap, sample_points)
            # Plot the starting position as a red marker
            plt.plot(start[0], start[1], "^r")
            # Plot the goal position as a green marker
            plt.plot(goal[0], goal[1], "^g")
        plt.grid(True)
        plt.axis("equal")
        plt.show()
    
    # Return generated roadmap, if using a costmap, return also the list of indices of the sample points
    if use_map:
        sample_points_tuple = list(zip(*sample_points))
        return sample_points_tuple, roadmap
    else:
        return roadmap
    


# Sample points are creates with the generate_sample_points function. The samples are created in the free space,
# based on the list of obstacles, and the dimension of the robot. Two samples should not be closer than the dimension of the robot.
def generate_sample_points(start, goal, rr, obstacles_list, obstacle_kd_tree, rng):
    """
    Generate sample points

    :param start: start x,y position
    :param goal: goal x,y position
    :param obstacle_list: obstacle x and y positions
    :param rr: robot radius
    :param obstacl_kd_tree: KDTree object of obstacles
    :param rng: (Optional) Random number generator
    :return: roadmap
    """
    ox = obstacles_list[:, 0]
    oy = obstacles_list[:, 1]
    sx = start[0]
    sy = start[1]
    gx = goal[0]
    gy = goal[1]

    if rng is None:
        rng = np.random.default_rng()

    # [Part 2] TODO Create list of sample points within the min and max obstacle coordinates
    # Use rng to create random samples. 
    # NOTE: by using rng, the created random samples may not be integers. 
    # When using the map, the samples should be indices of the cells of the costmap, therefore remember to round them to integer values.
    # Hint: you may leverage on the query function of KDTree to find the nearest neighbors

    # Generate random points within the bounding box of the obstacles
    sample_x, sample_y = [], []

    while len(sample_x) <= N_SAMPLE:
        x_rand = round(rng.uniform(min(ox), max(ox))) # generate random x value
        y_rand = round(rng.uniform(min(oy), max(oy))) # generate random y value

        if obstacle_kd_tree.query([x_rand, y_rand])[0] > rr: # ensure that samples are spaced at least rr away from any obstacles
            sample_x.append(x_rand)
            sample_y.append(y_rand)

    # [Part 2] TODO Add also the start and goal to the samples so that they are connected to the roadmap
    sample_x.extend([sx, gx])
    sample_y.extend([sy, gy])

    return [sample_x, sample_y]

# Check whethere there is a possible collision between two nodes, used for generating the roadmap after sampling the points.
def is_collision(sx, sy, gx, gy, rr, obstacle_kd_tree: KDTree, max_edge_len):
    """
    Check collisions
    
    :param sx: start node x
    :param sy: start node y
    :param gx: end node x
    :param gy: end node y
    :param rr: robot radius
    :param obstacle_kd_tree: KDTree object of obstacles
    :param max_edge_len: maximum edge length between two nodes

    :return False if no collision, True otherwise

    Checks collision between node sx,sy and gx,gy, accounting for the dimension of the robot rr.
    If max_edge_len is exceeded, also return true as if a collision is verified.
    """

    # [Part 2] TODO Check where there would be a collision with an obstacle between two nodes at sx,sy and gx,gy, and wether the edge between the two nodes is greater than max_edge_len
    # Hint: you may leverage on the query function of KDTree

    # Compute the Euclidean distance
    distance = math.hypot(gx - sx, gy - sy)
    if distance > max_edge_len: # return collision if distance exceeds max_edge_len
        return True

    # Check for collisions along the path
    num_points = int(distance / rr)
    x_points = np.linspace(sx, gx, num_points)
    y_points = np.linspace(sy, gy, num_points)

    for x, y in zip(x_points, y_points):
        dist, _ = obstacle_kd_tree.query([x, y])
        if dist <= rr: # collision detected
            return True

    return False  # No collision

# Create the roadmap using the generated sample_points, with the generate_road_map function. The roadmap is generated by looping
# over all samples. For each sample, N_KNN edges are created with neighboring samples. In creating the edge, a collision is checked with
# the is_collision function, to create edges that are not overlaying on obstacles. To introduce further control, a MAX_EDGE_LEN is imposed
# so that connected samples (nodes) are not too far from each other.
def generate_road_map(sample_points, rr, obstacle_kd_tree, max_edge_len, m_utilities=None):
    """
    Road map generation

    :param sample_points: x and y positions of sampled points in cell indices
    :param rr: Robot Radius
    :param obstacle_kd_tree: KDTree object of obstacles
    :param m_utilities: optional, needed when using costmap
    :return road_map

    Create a roadmap that is a list of sublists.
    Each sublist is a list of tlists corresponding to the indices of the nodes to which the sample point has an edge connection.
    e.g. for N_KNN=3:
    [i, j, k], [.. , .., ..], [.. , .., ..], ...]
    Indicates that sample_point[0] is connected to 3 nodes, at positions i, j, k.
    """
    sample_x = sample_points[0]
    sample_y = sample_points[1]

    # Note: The roadmap should have the same length as sample_points
    road_map = []
    kd_tree = KDTree(np.column_stack((sample_x, sample_y)))

    #[Part 2] TODO Generate roadmap for all sample points, i.e. create the edges between nodes (sample points)
    # Note: use the is_collision function to check for possible collisions (do not make an edge if there is collision)
    # Hint: you may ceate a KDTree object to help with the generation of the roadmap, but other methods also work

    for i, (sx, sy) in enumerate(zip(sample_x, sample_y)):
        neighbors = kd_tree.query([sx, sy], k=N_KNN + 1)[1] # get index of N_KNN closest neighbors
        edges = []
        for neighbor in neighbors[1:]:  # Skip self in the neighbors list
            gx, gy = sample_x[neighbor], sample_y[neighbor]
            if not is_collision(sx, sy, gx, gy, rr, obstacle_kd_tree, max_edge_len):
                edges.append(neighbor)
        road_map.append(edges)

    return road_map


def plot_road_map(road_map, sample_points): 
    """
    Plot the generated roadmap by connecting the nodes with the edges as per generated roadmap
    """

    sample_x = sample_points[0]
    sample_y = sample_points[1]

    for i, _ in enumerate(road_map):
        for ii in range(len(road_map[i])):
            ind = road_map[i][ii]
            plt.plot([sample_x[i], sample_x[ind]],
                    [sample_y[i], sample_y[ind]], "-c")


def main(rng=None):
    print(__file__ + " start!!")

    # start and goal position
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    robot_size = 5.0  # [m]

    ox = []
    oy = []

    for i in range(60):
        ox.append(i)
        oy.append(0.0)
    for i in range(60):
        ox.append(60.0)
        oy.append(i)
    for i in range(61):
        ox.append(i)
        oy.append(60.0)
    for i in range(61):
        ox.append(0.0)
        oy.append(i)
    for i in range(40):
        ox.append(20.0)
        oy.append(i)
    for i in range(40):
        ox.append(40.0)
        oy.append(60.0 - i)


    obstacles = np.column_stack((ox, oy))

    prm_graph([sx, sy], [gx, gy], obstacles, robot_size, rng=rng)



if __name__ == '__main__':
    main()
