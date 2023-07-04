import numpy as np
import matplotlib.pyplot as plt
from environment import StaticEnvironment


class RRTPlanner:
    def __init__(self, start, goal, environment, max_iter=1000, max_distance=0.5):
        self.start = start
        self.goal = goal
        self.environment = environment
        self.max_iter = max_iter
        self.max_distance = max_distance

    def plan(self):
        tree = {self.start: None}

        for _ in range(self.max_iter):
            random_point = self._get_random_point()
            nearest_node = self._find_nearest_node(tree, random_point)
            new_node = self._extend(nearest_node, random_point)
            if new_node and self.environment.is_free(*new_node):
                tree[new_node] = nearest_node
                if self._is_goal_reached(new_node):
                    return self._construct_path(tree, new_node)

        return None

    def _get_random_point(self):
        if np.random.rand() < 0.1:
            return self.goal
        return self.environment.random_free_space()

    def _find_nearest_node(self, tree, point):
        min_distance = float('inf')
        nearest_node = None

        for node in tree:
            distance = np.linalg.norm(np.array(node) - np.array(point))
            if distance < min_distance:
                min_distance = distance
                nearest_node = node

        return nearest_node

    def _extend(self, from_node, to_point):
        direction = np.array(to_point) - np.array(from_node)
        distance = np.linalg.norm(direction)
        if distance > self.max_distance:
            direction = direction / distance * self.max_distance
        new_node = tuple(np.array(from_node) + direction)
        return new_node

    def _is_goal_reached(self, node):
        return np.linalg.norm(np.array(node) - np.array(self.goal)) <= self.max_distance

    def _construct_path(self, tree, final_node):
        path = [final_node]
        current_node = final_node

        while current_node != self.start:
            parent_node = tree[current_node]
            path.append(parent_node)
            current_node = parent_node

        path.reverse()
        return path