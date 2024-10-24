# Copyright 2024 Alex Harbuzenko. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
from typing import List, Tuple
from prepared_triangle import PreparedTriangle

# Pre-computed immediate neighbor offsets (26 adjacent cells)
NEIGHBOR_OFFSETS = np.array([
    # Same layer (z=0)
    [-1, -1, 0], [0, -1, 0], [1, -1, 0],
    [-1,  0, 0], [0,  0, 0], [1,  0, 0],
    [-1,  1, 0], [0,  1, 0], [1,  1, 0],
    # Layer below (z=-1)
    [-1, -1, -1], [0, -1, -1], [1, -1, -1],
    [-1,  0, -1], [0,  0, -1], [1,  0, -1],
    [-1,  1, -1], [0,  1, -1], [1,  1, -1],
    # Layer above (z=1)
    [-1, -1, 1], [0, -1, 1], [1, -1, 1],
    [-1,  0, 1], [0,  0, 1], [1,  0, 1],
    [-1,  1, 1], [0,  1, 1], [1,  1, 1],
], dtype=np.int32)

def generate_ordered_offsets(grid_x: int, grid_y: int, grid_z: int) -> np.ndarray:
    """
    Generate grid cell offsets ordered by distance from center.
    
    Args:
        grid_x: Grid size in X dimension
        grid_y: Grid size in Y dimension
        grid_z: Grid size in Z dimension
        
    Returns:
        Array of [x,y,z] offsets sorted by distance from center, shape (N,3)
    """
    # For small grids, return just the immediate neighbors
    if max(grid_x, grid_y, grid_z) <= 1:
        return NEIGHBOR_OFFSETS
        
    # Generate additional offsets for larger grids
    offsets = []
    for z in range(-grid_z, grid_z):
        for y in range(-grid_y, grid_y):
            for x in range(-grid_x, grid_x):
                if max(abs(x), abs(y), abs(z)) > 1:  # Skip immediate neighbors
                    offsets.append([x, y, z])
    
    if not offsets:  # If no additional offsets needed
        return NEIGHBOR_OFFSETS
    
    offsets = np.array(offsets, dtype=np.int32)
    # Sort by distance from center
    lengths = np.sqrt(np.sum(offsets * offsets, axis=1))
    sorted_indices = np.argsort(lengths)
    
    # Combine immediate neighbors with sorted further offsets
    return np.vstack([NEIGHBOR_OFFSETS, offsets[sorted_indices]])

def ray_bound_intersection(lb: np.ndarray, ub: np.ndarray, point: np.ndarray, 
                         idir: np.ndarray) -> bool:
    """
    Determines Ray-Box intersection using the algorithm by 
    Amy Williams, Steve Barrus, R. Keith Morley, and Peter Shirley.
    
    Args:
        lb: Lower bound of box
        ub: Upper bound of box
        point: Ray origin
        idir: Inverted ray direction (1/dir)
        
    Returns:
        True if ray intersects box, False otherwise
    """
    # Handle x dimension
    if idir[0] >= 0:
        tmin = (lb[0] - point[0]) * idir[0]
        tmax = (ub[0] - point[0]) * idir[0]
    else:
        tmin = (ub[0] - point[0]) * idir[0]
        tmax = (lb[0] - point[0]) * idir[0]
    
    # Handle y dimension
    if idir[1] >= 0:
        tymin = (lb[1] - point[1]) * idir[1]
        tymax = (ub[1] - point[1]) * idir[1]
    else:
        tymin = (ub[1] - point[1]) * idir[1]
        tymax = (lb[1] - point[1]) * idir[1]
    
    if tmin > tymax or tmax < tymin:
        return False
    
    tmin = max(tmin, tymin)
    tmax = min(tmax, tymax)
    
    # Handle z dimension
    if idir[2] >= 0:
        tzmin = (lb[2] - point[2]) * idir[2]
        tzmax = (ub[2] - point[2]) * idir[2]
    else:
        tzmin = (ub[2] - point[2]) * idir[2]
        tzmax = (lb[2] - point[2]) * idir[2]
    
    return tmin <= tzmax and tmax >= tzmin

def segment_bound_intersection(lb: np.ndarray, ub: np.ndarray, point: np.ndarray, 
                             idir: np.ndarray, length: float) -> Tuple[bool, float, float]:
    """
    Check intersection between a line segment and AABB.
    
    Args:
        lb: Lower bound of box
        ub: Upper bound of box
        point: Segment start point
        idir: Inverted direction vector
        length: Segment length
        
    Returns:
        Tuple containing:
        - bool: True if intersection exists
        - float: Enter distance
        - float: Exit distance
    """
    result_enter = 0.0
    result_exit = length
    
    # Handle x dimension
    if idir[0] >= 0:
        tmin = (lb[0] - point[0]) * idir[0]
        tmax = (ub[0] - point[0]) * idir[0]
    else:
        tmin = (ub[0] - point[0]) * idir[0]
        tmax = (lb[0] - point[0]) * idir[0]
    
    # Handle y dimension
    if idir[1] >= 0:
        tymin = (lb[1] - point[1]) * idir[1]
        tymax = (ub[1] - point[1]) * idir[1]
    else:
        tymin = (ub[1] - point[1]) * idir[1]
        tymax = (lb[1] - point[1]) * idir[1]
    
    if tmin > tymax or tmax < tymin:
        return False, result_enter, result_exit
    
    tmin = max(tmin, tymin)
    tmax = min(tmax, tymax)
    
    # Handle z dimension
    if idir[2] >= 0:
        tzmin = (lb[2] - point[2]) * idir[2]
        tzmax = (ub[2] - point[2]) * idir[2]
    else:
        tzmin = (ub[2] - point[2]) * idir[2]
        tzmax = (lb[2] - point[2]) * idir[2]
    
    if tmin > tzmax or tmax < tzmin:
        return False, result_enter, result_exit
    
    tmin = max(tmin, tzmin)
    tmax = min(tmax, tzmax)
    
    # Check if segment intersects
    if tmin <= 0 and tmax >= length:  # Inside box
        return True, result_enter, result_exit
    
    if tmax < 0 or tmin > length:  # Outside box
        return False, result_enter, result_exit
    
    # Partial intersection
    if tmin > 0:
        result_enter = tmin
    if tmax < length:
        result_exit = tmax
    
    return True, result_enter, result_exit

class TriangleGrid:
    """
    A spatial partitioning structure that distributes triangles into a 3D grid for efficient
    spatial queries. The grid divides the space into cells, and each cell maintains a list
    of triangles that intersect it.
    
    Features:
    - Variable grid dimensions (different width/height/depth)
    - Efficient spatial queries using grid cells
    - Ordered cell traversal starting from immediate neighbors (26 cells)
    - Ray intersection testing
    - Distance field generation
    """
    
    def __init__(self, scene_min: np.ndarray, scene_max: np.ndarray, 
                 grid_x: int, grid_y: int, grid_z: int, 
                 triangles: List['PreparedTriangle']):
        """
        Initialize the TriangleGrid with scene bounds and dimensions.
        
        Args:
            scene_min: Minimum scene bounds (3D point)
            scene_max: Maximum scene bounds (3D point)
            grid_x: Number of grid cells in X dimension
            grid_y: Number of grid cells in Y dimension
            grid_z: Number of grid cells in Z dimension
            triangles: List of PreparedTriangle objects to distribute in the grid
        """
        self.scene_min = scene_min
        self.grid_step = max(
            (scene_max[0] - scene_min[0]) / grid_x,
            (scene_max[1] - scene_min[1]) / grid_y,
            (scene_max[2] - scene_min[2]) / grid_z
        )
        
        self.grid_x = grid_x
        self.grid_y = grid_y
        self.grid_z = grid_z
        
        # Initialize statistics
        self.triangle_count = len(triangles)
        self.triangle_instances = 0
        self.cells_used = 0
        
        # Initialize grid
        self.grid = [None] * (grid_x * grid_y * grid_z)
        
        # Generate ordered cell offsets for neighbor search
        self.cell_offsets = generate_ordered_offsets(grid_x, grid_y, grid_z)
        # Calculate offset lengths for early exit check
        self.offset_lengths = np.sqrt(np.sum(self.cell_offsets * self.cell_offsets, axis=1))
        
        # Distribute triangles to grid cells
        for triangle in triangles:
            # Get tile coordinates for lower and upper triangle points
            lb = (triangle.lower_bound - scene_min) / self.grid_step
            from_x = max(int(np.floor(lb[0])), 0)
            from_y = max(int(np.floor(lb[1])), 0)
            from_z = max(int(np.floor(lb[2])), 0)
            
            ub = (triangle.upper_bound - scene_min) / self.grid_step
            to_x = min(int(np.ceil(ub[0])), grid_x - 1)
            to_y = min(int(np.ceil(ub[1])), grid_y - 1)
            to_z = min(int(np.ceil(ub[2])), grid_z - 1)
            
            instances = 0
            
            # Check each potential grid cell
            for z in range(from_z, to_z + 1):
                for y in range(from_y, to_y + 1):
                    for x in range(from_x, to_x + 1):
                        # Check if triangle plane intersects the box
                        tile_start = np.array([x, y, z], dtype=np.float32) * self.grid_step + scene_min
                        tile_end = tile_start + np.array([self.grid_step] * 3)
                        
                        if not triangle.plane_intersects_aabb(tile_start, tile_end):
                            continue
                        
                        index = x + y * grid_x + z * grid_x * grid_y
                        
                        if self.grid[index] is None:
                            self.grid[index] = []
                            self.cells_used += 1
                        
                        self.grid[index].append(triangle)
                        instances += 1
            
            if instances == 0:
                print(f"Triangle got no instances {lb} - {ub}")
            
            self.triangle_instances += instances
        
        # Sort triangles by area in descending order
        for cell in self.grid:
            if cell is not None:
                cell.sort(key=lambda x: -x.area)
       
    def find_triangles(self, point: np.ndarray) -> Tuple[float, np.ndarray, int]:
        """
        Find the closest triangle to a point and compute signed distance.
        
        Args:
            point: Query point (3D)
            
        Returns:
            Tuple containing:
            - Signed distance to closest triangle
            - Barycentric coordinates on closest triangle
            - ID of closest triangle (-1 if none found)
        """
        distance = float('inf')
        min_distance_sqrd = float('inf')
        triangle_id = -1
        closest_triangle = None
        weights = np.zeros(3, dtype=np.float32)
        result = np.zeros(3, dtype=np.float32)
        
        # Convert point to grid coordinates
        local_point = (point - self.scene_min) / self.grid_step
        point_x = int(np.floor(local_point[0]))
        point_y = int(np.floor(local_point[1]))
        point_z = int(np.floor(local_point[2]))
        
        triangles = set()  # Track processed triangles
        local_dist = float('inf')
        lb = np.zeros(3)
        ub = np.zeros(3)
        
        early_exit = False
        
        # Check cells in order of increasing distance
        for i, (offset, offset_length) in enumerate(zip(self.cell_offsets, self.offset_lengths)):
            if offset_length > local_dist:
                break
                
            # Check if cell is outside grid
            x = point_x + offset[0]
            y = point_y + offset[1]
            z = point_z + offset[2]
            
            if not (0 <= x < self.grid_x and 0 <= y < self.grid_y and 0 <= z < self.grid_z):
                continue
                
            index = x + y * self.grid_x + z * self.grid_x * self.grid_y
            
            # Early exit for isolated points (after checking 26 neighbors)
            if i >= 26 and local_dist == float('inf'):
                early_exit = True
                distance = offset_length * self.grid_step
                break
            
            # Skip empty cells
            if self.grid[index] is None:
                continue
                
            # Process triangles in cell
            for triangle in self.grid[index]:
                if triangle not in triangles:
                    triangles.add(triangle)
                    
                    if distance != float('inf'):
                        if not triangle.intersects_aabb(lb, ub):
                            continue
                        if not triangle.intersects_sphere(point, distance):
                            continue
                    
                    temp_result, temp_weights = triangle.closest_point_to_triangle(point)
                    dir = point - temp_result
                    dist = np.dot(dir, dir)
                    
                    if dist < min_distance_sqrd:
                        min_distance_sqrd = dist
                        distance = np.sqrt(dist)
                        
                        lb = point - distance
                        ub = point + distance
                        
                        weights = temp_weights
                        result = temp_result
                        closest_triangle = triangle
            
            if distance != float('inf'):
                local_dist = distance / self.grid_step + np.sqrt(3) / 2
            
            if early_exit:
                break
        
        # Determine sign using ray casting
        if not early_exit and closest_triangle is not None:
            direction = point - result
            if np.any(np.isnan(direction)):
                direction = point - np.array([0.5, 0.5, 0.5])
            direction = direction / np.linalg.norm(direction)
            
            test_directions = [
                direction,
                np.array([0, 0, 1]),
                np.array([1, 0, 0]),
                np.array([0, 1, 0]),
                np.array([0, 0, -1]),
                np.array([-1, 0, 0]),
                np.array([0, -1, 0])
            ]
            
            sign = 0
            for dir in test_directions:
                sign += 1 if self.count_intersections(point, dir) % 2 == 0 else -1
                if abs(sign) >= len(test_directions) // 2 + 1:
                    break
            
            sign = 1 if sign >= 0 else -1
            distance *= sign
        
        triangle_id = closest_triangle.id if closest_triangle is not None else -1
        return distance, weights, triangle_id
    
    def ray_bound_intersection(lb: np.ndarray, ub: np.ndarray, point: np.ndarray, 
                         idir: np.ndarray) -> bool:
        """
        Determines Ray-Box intersection using the algorithm by 
        Amy Williams, Steve Barrus, R. Keith Morley, and Peter Shirley.
        
        Args:
            lb: Lower bound of box
            ub: Upper bound of box
            point: Ray origin
            idir: Inverted ray direction (1/dir)
            
        Returns:
            True if ray intersects box, False otherwise
        """
        # Handle x dimension
        if idir[0] >= 0:
            tmin = (lb[0] - point[0]) * idir[0]
            tmax = (ub[0] - point[0]) * idir[0]
        else:
            tmin = (ub[0] - point[0]) * idir[0]
            tmax = (lb[0] - point[0]) * idir[0]
        
        # Handle y dimension
        if idir[1] >= 0:
            tymin = (lb[1] - point[1]) * idir[1]
            tymax = (ub[1] - point[1]) * idir[1]
        else:
            tymin = (ub[1] - point[1]) * idir[1]
            tymax = (lb[1] - point[1]) * idir[1]
        
        if tmin > tymax or tmax < tymin:
            return False
        
        tmin = max(tmin, tymin)
        tmax = min(tmax, tymax)
        
        # Handle z dimension
        if idir[2] >= 0:
            tzmin = (lb[2] - point[2]) * idir[2]
            tzmax = (ub[2] - point[2]) * idir[2]
        else:
            tzmin = (ub[2] - point[2]) * idir[2]
            tzmax = (lb[2] - point[2]) * idir[2]
        
        return tmin <= tzmax and tmax >= tzmin

    def segment_bound_intersection(lb: np.ndarray, ub: np.ndarray, point: np.ndarray, 
                                idir: np.ndarray, length: float) -> Tuple[bool, float, float]:
        """
        Check intersection between a line segment and AABB.
        
        Args:
            lb: Lower bound of box
            ub: Upper bound of box
            point: Segment start point
            idir: Inverted direction vector
            length: Segment length
            
        Returns:
            Tuple containing:
            - bool: True if intersection exists
            - float: Enter distance
            - float: Exit distance
        """
        result_enter = 0.0
        result_exit = length
        
        # Handle x dimension
        if idir[0] >= 0:
            tmin = (lb[0] - point[0]) * idir[0]
            tmax = (ub[0] - point[0]) * idir[0]
        else:
            tmin = (ub[0] - point[0]) * idir[0]
            tmax = (lb[0] - point[0]) * idir[0]
        
        # Handle y dimension
        if idir[1] >= 0:
            tymin = (lb[1] - point[1]) * idir[1]
            tymax = (ub[1] - point[1]) * idir[1]
        else:
            tymin = (ub[1] - point[1]) * idir[1]
            tymax = (lb[1] - point[1]) * idir[1]
        
        if tmin > tymax or tmax < tymin:
            return False, result_enter, result_exit
        
        tmin = max(tmin, tymin)
        tmax = min(tmax, tymax)
        
        # Handle z dimension
        if idir[2] >= 0:
            tzmin = (lb[2] - point[2]) * idir[2]
            tzmax = (ub[2] - point[2]) * idir[2]
        else:
            tzmin = (ub[2] - point[2]) * idir[2]
            tzmax = (lb[2] - point[2]) * idir[2]
        
        if tmin > tzmax or tmax < tzmin:
            return False, result_enter, result_exit
        
        tmin = max(tmin, tzmin)
        tmax = min(tmax, tzmax)
        
        # Check if segment intersects
        if tmin <= 0 and tmax >= length:  # Inside box
            return True, result_enter, result_exit
        
        if tmax < 0 or tmin > length:  # Outside box
            return False, result_enter, result_exit
        
        # Partial intersection
        if tmin > 0:
            result_enter = tmin
        if tmax < length:
            result_exit = tmax
        
        return True, result_enter, result_exit

    def process_ray(self, from_point: np.ndarray, to_point: np.ndarray, action) -> bool:
        """
        Process cells along a ray path, calling action for each cell.
        
        Args:
            from_point: Start point in grid coordinates
            to_point: End point in grid coordinates
            action: Callback function taking cell index as parameter
            
        Returns:
            True if processing completed, False otherwise
        """
        # Get grid cell coordinates
        from_tile = np.floor(np.maximum(from_point, 0)).astype(np.int32)
        to_tile = np.floor(np.minimum(to_point, [self.grid_x - 1, self.grid_y - 1, self.grid_z - 1])).astype(np.int32)
        
        # Calculate step direction
        step = np.sign(to_tile - from_tile).astype(np.int32)
        
        # Handle case when ray stays in one cell
        if np.all(step == 0):
            if np.all((0 <= from_tile) & (from_tile < [self.grid_x, self.grid_y, self.grid_z])):
                index = from_tile[0] + from_tile[1] * self.grid_x + from_tile[2] * self.grid_x * self.grid_y
                action(index)
                return True
            return False
        
        # Calculate normalized direction
        dir = to_tile - from_tile
        dir_length = np.linalg.norm(dir)
        if dir_length > 0:
            dir = dir / dir_length
        
        # Calculate deltas and initial intersection points
        delta = np.where(np.abs(dir) > np.finfo(float).eps,
                        1.0 / np.abs(dir),
                        float('inf'))
        
        # Calculate initial intersection points
        point = np.full(3, float('inf'))
        for i in range(3):
            if delta[i] != float('inf'):
                point[i] = (from_tile[i] - from_point[i]) / dir[i]
                if dir[i] > 0:
                    point[i] += delta[i]
        
        # Initialize current position
        current = from_tile.copy()
        index = current[0] + current[1] * self.grid_x + current[2] * self.grid_x * self.grid_y
        
        # Process cells along ray
        max_steps = max(self.grid_x, self.grid_y, self.grid_z) + 2
        
        while max_steps > 0:
            max_steps -= 1
            
            # Check if current cell is valid
            if np.all((0 <= current) & (current < [self.grid_x, self.grid_y, self.grid_z])):
                action(index)
            else:
                break
            
            # Find next cell crossing
            next_cross = np.argmin(point)
            
            if current[next_cross] == to_tile[next_cross]:
                break
                
            point[next_cross] += delta[next_cross]
            current[next_cross] += step[next_cross]
            index += step[next_cross] * (self.grid_x if next_cross == 1 else 
                                    1 if next_cross == 0 else 
                                    self.grid_x * self.grid_y)
        
        return False

    def count_intersections(self, point: np.ndarray, dir: np.ndarray) -> int:
        """
        Count number of triangle intersections along a ray.
        
        Args:
            point: Ray origin
            dir: Ray direction (normalized)
            
        Returns:
            Number of intersections along the ray
        """
        count = 0
        idir = np.where(dir != 0, 1.0 / dir, float('inf'))
        
        # Convert to grid coordinates
        local_point = (point - self.scene_min) / self.grid_step
        grid_max = np.array([self.grid_x, self.grid_y, self.grid_z])
        length_max = np.linalg.norm(grid_max)
        
        # Check bounds intersection
        intersects, bound_enter, bound_exit = segment_bound_intersection(
            np.zeros(3), grid_max, local_point, idir, length_max)
        
        if not intersects:
            return 0
        
        local_end_point = local_point + dir * bound_exit
        
        # Track processed triangles
        triangles = set()
        
        def process_cell(index):
            nonlocal count
            if self.grid[index] is None:
                return
                
            for triangle in self.grid[index]:
                if triangle not in triangles:
                    triangles.add(triangle)
                    
                    if not ray_bound_intersection(triangle.lower_bound, triangle.upper_bound, 
                                            point, idir):
                        continue
                    
                    if triangle.intersects_ray(point, dir):
                        count += 1
        
        self.process_ray(local_point, local_end_point, process_cell)
        return count

    def dispatch(self, lower_bound: np.ndarray, pixels_to_scene: float, 
                scene_to_pixels: float, sx: int, sy: int, sz: int) -> np.ndarray:
        """
        Generate distance field data for a grid of points.
        
        Args:
            lower_bound: Lower bound of the volume to process
            pixels_to_scene: Scale factor from pixels to scene units
            scene_to_pixels: Scale factor from scene to pixels units
            sx, sy, sz: Volume dimensions in pixels
            
        Returns:
            Array of shape (sx*sy*sz, 4) containing:
            - Distance value
            - Barycentric u coordinate
            - Barycentric v coordinate
            - Triangle ID
        """
        max_count = sx * sy * sz
        result = np.zeros((max_count, 4), dtype=np.float32)
        
        for i in range(max_count):
            iz = i // (sx * sy)
            iy = (i % (sx * sy)) // sx
            ix = (i % (sx * sy)) % sx
            
            point = lower_bound + np.array([ix, iy, iz]) * pixels_to_scene
            
            distance, weights, triangle_id = self.find_triangles(point)
            pixel_distance = distance * scene_to_pixels
            
            result[i] = [pixel_distance, weights[0], weights[1], triangle_id]
        
        return result