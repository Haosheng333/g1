import heapq
import math

class AStarPlanner:
    def __init__(self):
        self.motions = [
            (1, 0, 1.0), (0, 1, 1.0), (-1, 0, 1.0), (0, -1, 1.0),
            (1, 1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414), (1, -1, 1.414)
        ]

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def plan(self, costmap, start_w, goal_w):
        start_node = costmap.world_to_map(start_w[0], start_w[1])
        goal_node = costmap.world_to_map(goal_w[0], goal_w[1])

        if not costmap.is_free(goal_node[0], goal_node[1]):
            return []

        open_set = []
        heapq.heappush(open_set, (0 + self.heuristic(start_node, goal_node), 0, start_node, None))
        
        came_from = {}
        cost_so_far = {start_node: 0}

        while open_set:
            _, current_cost, current, parent = heapq.heappop(open_set)
            came_from[current] = parent

            if current == goal_node:
                break

            for dx, dy, move_cost in self.motions:
                neighbor = (current[0] + dx, current[1] + dy)
                if costmap.is_free(neighbor[0], neighbor[1]):
                    new_cost = current_cost + move_cost
                    if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                        cost_so_far[neighbor] = new_cost
                        priority = new_cost + self.heuristic(neighbor, goal_node)
                        heapq.heappush(open_set, (priority, new_cost, neighbor, current))

        if goal_node not in came_from:
            return []

        path = []
        curr = goal_node
        while curr is not None:
            wx, wy = costmap.map_to_world(curr[0], curr[1])
            path.append((wx, wy))
            curr = came_from[curr]

        path.reverse()
        return path
