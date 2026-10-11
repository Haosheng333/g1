import numpy as np

# Tier values published on /map by g1_slam/grid_composer (doc 4.3):
#   -1 unknown, 0 WALKABLE, 33 RISK, 66 DANGER, 100 OBSTACLE.
# Until the full tier-aware costmap (section C of the fix list) is written,
# WALKABLE and RISK are traversable; DANGER, OBSTACLE and unknown are not.
# A plain binary map (0 / 100 / -1) behaves exactly as before.
BLOCKED_FROM = 50


class OccupancyCostmap:
    def __init__(self, inflation_cells=2):
        self.grid = None
        self.width = 0
        self.height = 0
        self.resolution = 0.05
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.inflation_cells = inflation_cells

    def update_map(self, map_msg):
        self.width = map_msg.info.width
        self.height = map_msg.info.height
        self.resolution = map_msg.info.resolution
        self.origin_x = map_msg.info.origin.position.x
        self.origin_y = map_msg.info.origin.position.y

        raw_data = np.array(map_msg.data, dtype=np.int8).reshape((self.height, self.width))
        self.grid = np.copy(raw_data)
        if self.inflation_cells > 0:
            self._inflate_obstacles()

    def world_to_map(self, wx, wy):
        mx = int((wx - self.origin_x) / self.resolution)
        my = int((wy - self.origin_y) / self.resolution)
        return mx, my

    def map_to_world(self, mx, my):
        wx = mx * self.resolution + self.origin_x + (self.resolution / 2.0)
        wy = my * self.resolution + self.origin_y + (self.resolution / 2.0)
        return wx, wy

    def is_free(self, mx, my):
        if 0 <= mx < self.width and 0 <= my < self.height:
            v = self.grid[my, mx]
            return 0 <= v < BLOCKED_FROM
        return False

    def _inflate_obstacles(self):
        obstacles = np.argwhere(self.grid >= BLOCKED_FROM)
        for r, c in obstacles:
            for dr in range(-self.inflation_cells, self.inflation_cells + 1):
                for dc in range(-self.inflation_cells, self.inflation_cells + 1):
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < self.height and 0 <= nc < self.width:
                        if 0 <= self.grid[nr, nc] < BLOCKED_FROM:
                            self.grid[nr, nc] = 100
