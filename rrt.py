import random
import math


def dist(p1, p2):
    """Returns the Euclidean distance between two points."""
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)


def get_nearest_node(nodes, point):
    """Returns the node in the tree that is closest to the given point."""
    distances = [dist(n[:2], point) for n in nodes]
    min_dist_idx = distances.index(min(distances))
    return nodes[min_dist_idx]


def get_new_point(bounds):
    """Returns a new random point within the given bounds."""
    x = random.randint(bounds[0][0], bounds[1][0])
    y = random.randint(bounds[0][1], bounds[1][1])
    return (x, y)


def get_new_point_near(bounds, goal, nearness_factor=0.1):
    """
    Generates a new point in the space within the bounds that is closer to the goal point.
    The nearness_factor determines how close the new point is to the goal point.
    """
    x = random.uniform(bounds[0][0], bounds[1][0])
    y = random.uniform(bounds[0][1], bounds[1][1])
    if random.uniform(0, 1) < nearness_factor:
        x = x * (1 - nearness_factor) + goal[0] * nearness_factor
        y = y * (1 - nearness_factor) + goal[1] * nearness_factor
    # if random.uniform(0, 1) < nearness_factor:
    #     x = goal[0]
    #     y = goal[1]
    return (x, y)


def get_new_node(nodes, point, step_size):
    """Returns a new node that is a certain distance away from the nearest node
    and in the direction of the given point."""
    nearest_node = get_nearest_node(nodes, point)
    dist_to_nearest = dist(nearest_node[:2], point)
    if dist_to_nearest <= step_size:
        return point
    ratio = step_size / dist_to_nearest
    x = int(nearest_node[0] + (point[0]-nearest_node[0]) * ratio)
    y = int(nearest_node[1] + (point[1]-nearest_node[1]) * ratio)
    return (x, y)


def is_intersecting(line, obstacle):
    # p1 и p2 - координаты точек, задающих прямую
    # r1 и r2 - координаты двух противоположных вершин прямоугольника
    p1 = line[0]
    p2 = line[1]
    # Находим уравнение прямой, проходящей через две заданные точки
    a = p2[1] - p1[1]
    b = p1[0] - p2[0]
    c = p2[0]*p1[1] - p1[0]*p2[1]

    # Находим координаты остальных вершин прямоугольника
    r1 = obstacle[0]
    r2 = obstacle[1]
    r3 = (r1[0], r2[1])
    r4 = (r2[0], r1[1])

    # Проверяем, пересекает ли прямая каждую из сторон прямоугольника
    if (a*r1[0] + b*r1[1] + c) * (a*r2[0] + b*r2[1] + c) <= 0:
        return True
    if (a*r2[0] + b*r2[1] + c) * (a*r3[0] + b*r3[1] + c) <= 0:
        return True
    if (a*r3[0] + b*r3[1] + c) * (a*r4[0] + b*r4[1] + c) <= 0:
        return True
    if (a*r4[0] + b*r4[1] + c) * (a*r1[0] + b*r1[1] + c) <= 0:
        return True

    # Если ни одна сторона не пересекается с прямой, то прямая не пересекает прямоугольник
    return False


def line_on_obstacle(line, obstacle):
    x1, y1 = line[0]
    x2, y2 = line[1]
    obs_x1, obs_y1 = obstacle[0]
    obs_x2, obs_y2 = obstacle[1]

    dx = x2 - x1
    dy = y2 - y1

    # Переменные для определения, является ли пересечение внутренним или внешним
    inside_x = False
    inside_y = False

    # Проверяем, есть ли пересечение с левой или правой гранью препятствия
    if x1 <= obs_x1 and x2 >= obs_x1:
        inside_x = True
        k = (y2 - y1) / (x2 - x1)
        b = y1 - k * x1
        y = k * obs_x1 + b
        if obs_y1 <= y <= obs_y2:
            return (obs_x1, y)

    if x1 <= obs_x2 and x2 >= obs_x2:
        inside_x = True
        k = (y2 - y1) / (x2 - x1)
        b = y1 - k * x1
        y = k * obs_x2 + b
        if obs_y1 <= y <= obs_y2:
            return (obs_x2, y)

    # Проверяем, есть ли пересечение с верхней или нижней гранью препятствия
    if y1 <= obs_y1 and y2 >= obs_y1:
        inside_y = True
        k = (x2 - x1) / (y2 - y1)
        b = x1 - k * y1
        x = k * obs_y1 + b
        if obs_x1 <= x <= obs_x2 and not inside_x:
            return (x, obs_y1)

    if y1 <= obs_y2 and y2 >= obs_y2:
        inside_y = True
        k = (x2 - x1) / (y2 - y1)
        b = x1 - k * y1
        x = k * obs_y2 + b
        if obs_x1 <= x <= obs_x2 and not inside_x:
            return (x, obs_y2)

    # Если нет пересечения, то возвращаем None
    return None


def get_nearest_node_index(nodes, point):
    """Returns the index of the node in the tree that is closest to the given point."""
    distances = [dist(n[:2], point) for n in nodes]
    min_dist_idx = distances.index(min(distances))
    return min_dist_idx


def rrt(start, goal, bounds, obstacles, step_size=25, max_iter=5000):
    """Runs the RRT algorithm and returns the path from start to goal if one is found."""
    nodes = [start]
    for i in range(max_iter):
        #point = get_new_point(bounds)
        point = get_new_point_near(bounds, goal, nearness_factor=0.9)
        # if point in obstacles:
        #     continue

        # if len(obstacles) > 0:
        #     #if [(obs[0][0] <= point[0] <= obs[1][0]) and (obs[1][1] <= point[1] <= obs[0][1]) for obs in obstacles]:
        #     if any([(obs[0][0] <= point[0] <= obs[1][0]) and (obs[1][1] <= point[1] <= obs[0][1]) for obs in
        #             obstacles]):
        #         continue

        new_node = get_new_node(nodes, point, step_size)

        #new node in obstacle square
        if len(obstacles) > 0:
            if any([(obs[0][0] <= new_node[0] <= obs[1][0]) and (obs[1][1] <= new_node[1] <= obs[0][1]) for obs in
                    obstacles]):
                continue

        if len(obstacles) > 0 and len(nodes) > 0:
            line = ((nodes[-1][0], nodes[-1][1]), (new_node[0], new_node[1]))
            for obs in obstacles:
                collision = is_intersecting(line, obs)
                if collision:
                    break
            if not collision:
                nodes.append(new_node)
                # if dist(new_node, goal) <= step_size + 20:
                #     nodes.append(goal)
                #     return nodes
        else:
            nodes.append(new_node)
            # if dist(new_node, goal) <= step_size + 20:
            #     nodes.append(goal)
            #     return nodes
        if dist(new_node, goal) <= step_size + 20:
            nodes.append(goal)
            return nodes

    return None


def rrt_star(start, goal, bounds, obstacles, max_iter=5000, step_size=40):
    """
    Constructs a path from start to goal while avoiding obstacles using the RRT* algorithm.
    Returns the path as a list of points.
    """
    nodes = [start]
    # Create a dictionary to keep track of the parent node for each node in the tree.
    parent_dict = {start: None}
    # Create a dictionary to keep track of the cost to reach each node.
    cost_dict = {start: 0}
    connections = []
    for i in range(max_iter):
        # Generate a new random point.
        #point = get_new_point(bounds)
        point = get_new_point_near(bounds, goal, nearness_factor=0.7)

        # Find the nearest node in the tree to the new point.
        nearest_node = get_nearest_node(nodes, point)

        # Generate a new node that is a certain distance away from the nearest node and in the direction of the new point.
        new_node = get_new_node(nodes, point, step_size)

        # Check if the line connecting the nearest node to the new node intersects any of the obstacles.
        intersecting = False
        for obstacle in obstacles:
            line = ((nearest_node[0], nearest_node[1]), (new_node[0], new_node[1]))
            if is_intersecting(line, obstacle):
                intersecting = True
                break

        # If the line intersects an obstacle, skip this iteration.
        if intersecting:
            # peresech = line_on_obstacle(line, obstacle)
            # if peresech is not None:
            #     x_per, y_per = peresech
            #     connections.append((nearest_node, (x_per, y_per)))
            continue

        # Calculate the cost to reach the new node.
        cost_to_new_node = cost_dict[nearest_node] + dist(nearest_node, new_node)

        # Find all nodes in the tree that are within a certain radius of the new node.
        near_nodes = [node for node in nodes if dist(node, new_node) <= step_size]
        if len(near_nodes) == 0:
            continue
        # Calculate the cost to reach each of the near nodes through the new node.
        cost_dict_through_new_node = {node: cost_to_new_node + dist(node, new_node) for node in near_nodes}

        # Find the parent node for the new node that results in the lowest cost to reach the new node.
        # if cost_dict_through_new_node:
        #     parent_node = min(cost_dict_through_new_node, key=cost_dict_through_new_node.get)
        parent_node = min(cost_dict_through_new_node, key=cost_dict_through_new_node.get)

        # Add the new node to the tree.
        nodes.append(new_node)

        # Set the parent of the new node to be the parent that resulted in the lowest cost.
        parent_dict[new_node] = parent_node

        # Update the cost to reach the new node.
        cost_dict[new_node] = cost_dict_through_new_node[parent_node]

        # Update the cost to reach all near nodes if the cost can be improved by going through the new node.
        for node in near_nodes:
            if cost_dict_through_new_node[node] < cost_dict[node]:
                parent_dict[node] = new_node
                cost_dict[node] = cost_dict_through_new_node[node]
        # Add the connection between the new node and its parent to the connections list
        connections.append((new_node, parent_node))

        # If the new node is within a certain distance of the goal, we're done.
        if dist(new_node, goal) <= step_size:
            path = [new_node]

            while parent_dict[path[0]] is not None:
                path.insert(0, parent_dict[path[0]])
            #path.insert(0, start)
            path.append(goal)
            return path, connections

    # If we've reached the maximum number of iterations, return None to indicate failure.
    return None, connections


def rrt_star_2(start, goal, bounds, obstacles, max_iter=5000, step_size=40):
    """
    Constructs a path from start to goal while avoiding obstacles using the RRT* algorithm.
    Returns the path as a list of points.
    """
    nodes = [start]
    # Create a dictionary to keep track of the parent node for each node in the tree.
    parent_dict = {start: None}
    # Create a dictionary to keep track of the cost to reach each node.
    cost_dict = {start: 0}
    connections = []
    for i in range(max_iter):
        # Generate a new random point.
        #point = get_new_point(bounds)
        point = get_new_point_near(bounds, goal, nearness_factor=0.7)

        # Find the nearest node in the tree to the new point.
        nearest_node = get_nearest_node(nodes, point)

        # Generate a new node that is a certain distance away from the nearest node and in the direction of the new point.
        new_node = get_new_node(nodes, point, step_size)

        # Check if the line connecting the nearest node to the new node intersects any of the obstacles.
        intersecting = False
        for obstacle in obstacles:
            line = ((nearest_node[0], nearest_node[1]), (new_node[0], new_node[1]))
            if is_intersecting(line, obstacle):
                intersecting = True
                break

        # If the line intersects an obstacle, skip this iteration.
        if intersecting:
            # peresech = line_on_obstacle(line, obstacle)
            # if peresech is not None:
            #     x_per, y_per = peresech
            #     connections.append((nearest_node, (x_per, y_per)))
            continue

        # Calculate the cost to reach the new node.
        cost_to_new_node = cost_dict[nearest_node] + dist(nearest_node, new_node)

        # Find all nodes in the tree that are within a certain radius of the new node.
        near_nodes = [node for node in nodes if dist(node, new_node) <= step_size]
        if len(near_nodes) == 0:
            continue
        # Calculate the cost to reach each of the near nodes through the new node.
        cost_dict_through_new_node = {node: cost_to_new_node + dist(node, new_node) for node in near_nodes}

        # Find the parent node for the new node that results in the lowest cost to reach the new node.
        # if cost_dict_through_new_node:
        #     parent_node = min(cost_dict_through_new_node, key=cost_dict_through_new_node.get)
        parent_node = min(cost_dict_through_new_node, key=cost_dict_through_new_node.get)

        # Add the new node to the tree.
        nodes.append(new_node)

        # Set the parent of the new node to be the parent that resulted in the lowest cost.
        parent_dict[new_node] = parent_node

        # Update the cost to reach the new node.
        cost_dict[new_node] = cost_dict_through_new_node[parent_node]

        # Update the cost to reach all near nodes if the cost can be improved by going through the new node.
        for node in near_nodes:
            if cost_dict_through_new_node[node] < cost_dict[node]:
                parent_dict[node] = new_node
                cost_dict[node] = cost_dict_through_new_node[node]
        # Add the connection between the new node and its parent to the connections list
        connections.append((new_node, parent_node))

        # If the new node is within a certain distance of the goal, we're done.
        if dist(new_node, goal) <= step_size:
            path = [new_node]

            while parent_dict[path[0]] is not None:
                path.insert(0, parent_dict[path[0]])
            #path.insert(0, start)
            path.append(goal)
            return path, connections

    # If we've reached the maximum number of iterations, return None to indicate failure.
    return None, connections