"""
Bunker Map, Collision Geometries, LiDAR Raycaster & A* Global Planner
=====================================================================
Models the 40m x 40m subterranean bunker facility matching the Gazebo world.
Provides:
  - Obstacle definitions (walls, pillars, crates, partition)
  - 2D LiDAR raycaster simulation (360-degree laser scan)
  - A* grid path planning with obstacle inflation for Nav2 emulation
"""

import math
import heapq
from typing import List, Tuple, Dict, Optional

class BunkerMap:
    def __init__(self):
        # Facility boundaries (meters)
        self.min_x = -20.0
        self.max_x = 20.0
        self.min_y = -20.0
        self.max_y = 20.0

        # Segment walls: [x1, y1, x2, y2]
        self.wall_segments: List[Tuple[float, float, float, float]] = [
            # Perimeter walls
            (-20.0, -20.0, 20.0, -20.0),  # South
            (20.0, -20.0, 20.0, 20.0),    # East
            (20.0, 20.0, -20.0, 20.0),    # North
            (-20.0, 20.0, -20.0, -20.0),  # West

            # Center partition wall (10m x 0.5m)
            (-5.0, -0.25, 5.0, -0.25),
            (5.0, -0.25, 5.0, 0.25),
            (5.0, 0.25, -5.0, 0.25),
            (-5.0, 0.25, -5.0, -0.25),

            # Crate stack 1 (centered at 12, 2, size 1.6 x 1.6)
            (11.2, 1.2, 12.8, 1.2),
            (12.8, 1.2, 12.8, 2.8),
            (12.8, 2.8, 11.2, 2.8),
            (11.2, 2.8, 11.2, 1.2),

            # Crate stack 2 (centered at -12, -4, size 1.8 x 1.4)
            (-12.9, -4.7, -11.1, -4.7),
            (-11.1, -4.7, -11.1, -3.3),
            (-11.1, -3.3, -12.9, -3.3),
            (-12.9, -3.3, -12.9, -4.7),
        ]

        # Cylindrical Pillars: (center_x, center_y, radius)
        self.pillars: List[Tuple[float, float, float]] = [
            (8.0, 8.0, 0.6),
            (-8.0, 8.0, 0.6),
            (8.0, -8.0, 0.6),
            (-8.0, -8.0, 0.6),
        ]

        # Tactical Checkpoints (matching ROS 2 patrol mission)
        self.default_checkpoints = [
            {"id": "Alpha", "name": "Alpha Checkpoint (Perimeter Ingress)", "x": 5.0, "y": 0.0, "yaw": 0.0},
            {"id": "Bravo", "name": "Bravo Checkpoint (North Corridor)", "x": 8.0, "y": 6.0, "yaw": 1.57},
            {"id": "Charlie", "name": "Charlie Checkpoint (Storage Vault)", "x": 0.0, "y": 8.0, "yaw": 3.14},
            {"id": "Delta", "name": "Delta Checkpoint (West Flank)", "x": -6.0, "y": 2.0, "yaw": -1.57},
            {"id": "Echo", "name": "Echo Checkpoint (South Armory)", "x": -10.0, "y": -8.0, "yaw": -0.78},
            {"id": "Base", "name": "Base Station (Extraction Point)", "x": 0.0, "y": 0.0, "yaw": 0.0},
        ]

        # Optical Feature Dropout Zone (Unlit bunker corridor: triggers visual blackout)
        self.dropout_zone = {
            "x_min": -15.0,
            "x_max": -4.0,
            "y_min": 1.0,
            "y_max": 7.0,
            "name": "Blackout Zone (Low Feature Density / IR Jamming)"
        }

        # Grid resolution for A* path planner
        self.resolution = 0.5  # 0.5m cells
        self.grid_w = int((self.max_x - self.min_x) / self.resolution)
        self.grid_h = int((self.max_y - self.min_y) / self.resolution)
        self.costmap = self._build_costmap(inflation_radius=1.2)

    def is_in_dropout_zone(self, x: float, y: float) -> bool:
        """Determines if UGV is inside visual blackout / low feature zone."""
        return (self.dropout_zone["x_min"] <= x <= self.dropout_zone["x_max"] and
                self.dropout_zone["y_min"] <= y <= self.dropout_zone["y_max"])

    def is_collision(self, x: float, y: float, margin: float = 0.4) -> bool:
        """Check if (x, y) collides with walls or pillars."""
        # Boundary check
        if x < self.min_x + margin or x > self.max_x - margin:
            return True
        if y < self.min_y + margin or y > self.max_y - margin:
            return True

        # Center partition
        if (-5.0 - margin <= x <= 5.0 + margin) and (-0.25 - margin <= y <= 0.25 + margin):
            return True

        # Crates
        if (11.2 - margin <= x <= 12.8 + margin) and (1.2 - margin <= y <= 2.8 + margin):
            return True
        if (-12.9 - margin <= x <= -11.1 + margin) and (-4.7 - margin <= y <= -3.3 + margin):
            return True

        # Pillars
        for px, py, pr in self.pillars:
            if math.hypot(x - px, y - py) <= (pr + margin):
                return True

        return False

    def _build_costmap(self, inflation_radius: float = 1.2) -> List[List[int]]:
        """Precomputes 2D occupancy grid with inflation layer."""
        grid = [[0 for _ in range(self.grid_w)] for _ in range(self.grid_h)]
        for gy in range(self.grid_h):
            for gx in range(self.grid_w):
                wx = self.min_x + (gx + 0.5) * self.resolution
                wy = self.min_y + (gy + 0.5) * self.resolution
                if self.is_collision(wx, wy, margin=inflation_radius):
                    grid[gy][gx] = 100  # Occupied / Lethal
        return grid

    def world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        gx = int((wx - self.min_x) / self.resolution)
        gy = int((wy - self.min_y) / self.resolution)
        gx = max(0, min(self.grid_w - 1, gx))
        gy = max(0, min(self.grid_h - 1, gy))
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        wx = self.min_x + (gx + 0.5) * self.resolution
        wy = self.min_y + (gy + 0.5) * self.resolution
        return wx, wy

    def plan_path_astar(self, start_pos: Tuple[float, float], goal_pos: Tuple[float, float]) -> List[Tuple[float, float]]:
        """
        Global Path Planning using A* algorithm over the 2D Costmap.
        Emulates Nav2 NavFn / Smac Planner.
        """
        sx, sy = self.world_to_grid(start_pos[0], start_pos[1])
        gx, gy = self.world_to_grid(goal_pos[0], goal_pos[1])

        if (sx, sy) == (gx, gy):
            return [goal_pos]

        # 8-connectivity directions
        directions = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (-1, -1, 1.414)
        ]

        open_set = []
        heapq.heappush(open_set, (0.0, sx, sy))
        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        g_score: Dict[Tuple[int, int], float] = {(sx, sy): 0.0}

        def heuristic(ax, ay, bx, by):
            return math.hypot(ax - bx, ay - by)

        found = False
        while open_set:
            _, cx, cy = heapq.heappop(open_set)

            if (cx, cy) == (gx, gy):
                found = True
                break

            for dx, dy, cost in directions:
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < self.grid_w and 0 <= ny < self.grid_h:
                    if self.costmap[ny][nx] >= 100:
                        continue  # In collision
                    tentative_g = g_score[(cx, cy)] + cost
                    if (nx, ny) not in g_score or tentative_g < g_score[(nx, ny)]:
                        g_score[(nx, ny)] = tentative_g
                        came_from[(nx, ny)] = (cx, cy)
                        f_score = tentative_g + heuristic(nx, ny, gx, gy)
                        heapq.heappush(open_set, (f_score, nx, ny))

        if not found:
            # Fallback direct path
            return [start_pos, goal_pos]

        # Reconstruct path
        path = []
        curr = (gx, gy)
        while curr in came_from:
            wx, wy = self.grid_to_world(curr[0], curr[1])
            path.append((wx, wy))
            curr = came_from[curr]
        path.reverse()

        # Simplify path (subsample waypoints)
        simplified = [start_pos]
        step = max(1, len(path) // 15)
        for i in range(0, len(path), step):
            simplified.append(path[i])
        simplified.append(goal_pos)
        return simplified

    def cast_ray(self, rx: float, ry: float, angle: float, max_range: float = 14.0) -> Tuple[float, float, float]:
        """
        Raycasts single 2D laser beam against bunker obstacles.
        Returns (hit_dist, hit_x, hit_y).
        """
        dx = math.cos(angle)
        dy = math.sin(angle)
        min_dist = max_range
        hit_x = rx + dx * max_range
        hit_y = ry + dy * max_range

        # Ray-segment intersections for wall segments
        for x1, y1, x2, y2 in self.wall_segments:
            d = self._ray_segment_intersect(rx, ry, dx, dy, x1, y1, x2, y2)
            if d is not None and 0.0 < d < min_dist:
                min_dist = d
                hit_x = rx + dx * d
                hit_y = ry + dy * d

        # Ray-circle intersections for pillars
        for px, py, pr in self.pillars:
            d = self._ray_circle_intersect(rx, ry, dx, dy, px, py, pr)
            if d is not None and 0.0 < d < min_dist:
                min_dist = d
                hit_x = rx + dx * d
                hit_y = ry + dy * d

        return min_dist, hit_x, hit_y

    def get_lidar_scan(self, rx: float, ry: float, heading: float, num_beams: int = 36, max_range: float = 14.0) -> List[Dict[str, float]]:
        """Generates 360-degree LiDAR scan (laser range points)."""
        scan = []
        angle_step = (2.0 * math.pi) / num_beams
        for i in range(num_beams):
            angle = heading + (i * angle_step) - math.pi
            dist, hx, hy = self.cast_ray(rx, ry, angle, max_range)
            scan.append({
                "angle": angle,
                "range": round(dist, 2),
                "x": round(hx, 2),
                "y": round(hy, 2)
            })
        return scan

    @staticmethod
    def _ray_segment_intersect(rx, ry, dx, dy, x1, y1, x2, y2) -> Optional[float]:
        # Line segment: P1 + t * (P2 - P1)
        # Ray: R + s * D
        v1x, v1y = rx - x1, ry - y1
        v2x, v2y = x2 - x1, y2 - y1
        v3x, v3y = -dy, dx

        dot = v2x * v3x + v2y * v3y
        if abs(dot) < 1e-6:
            return None

        t1 = (v2x * v1y - v2y * v1x) / dot
        t2 = (v1x * v3x + v1y * v3y) / dot

        if t1 >= 0.0 and 0.0 <= t2 <= 1.0:
            return t1
        return None

    @staticmethod
    def _ray_circle_intersect(rx, ry, dx, dy, cx, cy, radius) -> Optional[float]:
        ox = rx - cx
        oy = ry - cy
        a = dx * dx + dy * dy
        b = 2.0 * (ox * dx + oy * dy)
        c = ox * ox + oy * oy - radius * radius
        disc = b * b - 4.0 * a * c
        if disc < 0:
            return None
        sqrt_disc = math.sqrt(disc)
        t1 = (-b - sqrt_disc) / (2.0 * a)
        if t1 > 0:
            return t1
        t2 = (-b + sqrt_disc) / (2.0 * a)
        if t2 > 0:
            return t2
        return None
