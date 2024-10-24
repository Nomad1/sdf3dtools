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
from typing import Tuple

class PreparedTriangle:
    """
    A class for representing and analyzing triangles in 3D space using NumPy arrays.
    
    This class pre-calculates various properties of a triangle (normal, area, bounds)
    and provides methods for geometric queries like intersection tests with rays,
    axis-aligned bounding boxes (AABB), and spheres. Compatible with common graphics
    and ML frameworks that use NumPy arrays.
    
    Vertices and vectors are represented as NumPy arrays of shape (3,).
    """
    
    @classmethod
    def from_vertex_array(cls, id: int, vertices: np.ndarray, i1: int, i2: int, i3: int) -> 'PreparedTriangle':
        """
        Create a triangle from an array of vertices and three indices.
        
        Args:
            id: Unique identifier for the triangle
            vertices: Array of vertex positions of shape (N, 3) where N is number of vertices
            i1: Index of first vertex
            i2: Index of second vertex
            i3: Index of third vertex
            
        Returns:
            New PreparedTriangle instance
        """
        return cls(id, vertices[i1], vertices[i2], vertices[i3])
    
    def __init__(self, id: int, a: np.ndarray, b: np.ndarray, c: np.ndarray):
        """
        Initialize a PreparedTriangle with vertices and calculate its properties.
        
        Args:
            id: Unique identifier for the triangle
            a: First vertex position as numpy array of shape (3,)
            b: Second vertex position as numpy array of shape (3,)
            c: Third vertex position as numpy array of shape (3,)
        """
        self.id = id
        self.a = a.astype(np.float32)
        self.b = b.astype(np.float32)
        self.c = c.astype(np.float32)
        
        # Calculate normal vector
        self.n = np.cross(b - a, c - a)
        
        # Calculate area and normalize normal vector
        self.area = np.sum(self.n * self.n)  # squared length of normal
        self.normal_length = max(np.sqrt(self.area), 1.0e-20)
        self.n = self.n / self.normal_length
        self.area /= 2
        
        # Calculate center and radius
        center = (a + b + c) / 3.0
        self.radius = np.sqrt(max(
            max(
                np.sum((center - a) ** 2),
                np.sum((center - b) ** 2)
            ),
            np.sum((center - c) ** 2)
        ))
        
        # Calculate bounds
        self.lower_bound = np.minimum(np.minimum(a, b), c)
        self.upper_bound = np.maximum(np.maximum(a, b), c)

    def closest_point_to_triangle(self, p: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Find the closest point on the triangle to a given point in 3D space.
        
        Args:
            p: Point to find closest position to, shape (3,)
            
        Returns:
            Tuple containing:
            - The closest point on the triangle, shape (3,)
            - Barycentric coordinates of the closest point (weights), shape (3,)
        """
        snom = np.dot(p - self.a, self.b - self.a)
        sdenom = np.dot(p - self.b, self.a - self.b)
        
        tnom = np.dot(p - self.a, self.c - self.a)
        tdenom = np.dot(p - self.c, self.a - self.c)
        
        unom = np.dot(p - self.b, self.c - self.b)
        udenom = np.dot(p - self.c, self.b - self.c)
        
        # Check vertices
        if snom <= 0.0 and tnom <= 0.0:
            weights = np.array([1.0, 0.0, 0.0], dtype=np.float32)
            return self.a, weights
            
        if sdenom <= 0.0 and unom <= 0.0:
            weights = np.array([0.0, 1.0, 0.0], dtype=np.float32)
            return self.b, weights
            
        if tdenom <= 0.0 and udenom <= 0.0:
            weights = np.array([0.0, 0.0, 1.0], dtype=np.float32)
            return self.c, weights
        
        # Check edges
        coords_pab = np.dot(self.n, np.cross(self.a - p, self.b - p))
        if coords_pab <= 0 and snom >= 0.0 and sdenom >= 0.0:
            nab = snom / (snom + sdenom)
            weights = np.array([1.0 - nab, nab, 0.0], dtype=np.float32)
            return self.a + (self.b - self.a) * nab, weights
            
        coords_pbc = np.dot(self.n, np.cross(self.b - p, self.c - p))
        if coords_pbc <= 0 and unom >= 0.0 and udenom >= 0.0:
            nbc = unom / (unom + udenom)
            weights = np.array([0.0, 1.0 - nbc, nbc], dtype=np.float32)
            return self.b + (self.c - self.b) * nbc, weights
            
        coords_pca = np.dot(self.n, np.cross(self.c - p, self.a - p))
        if coords_pca <= 0 and tnom >= 0.0 and tdenom >= 0.0:
            nca = tnom / (tnom + tdenom)
            weights = np.array([nca, 0.0, 1.0 - nca], dtype=np.float32)
            return self.a + (self.c - self.a) * nca, weights
        
        # Point is inside triangle
        denom = coords_pab + coords_pbc + coords_pca
        u = coords_pbc / denom
        v = coords_pca / denom
        weights = np.array([u, v, 1.0 - u - v], dtype=np.float32)
        return self.a * weights[0] + self.b * weights[1] + self.c * weights[2], weights
    
    def intersects_ray(self, p: np.ndarray, dir: np.ndarray) -> bool:
        """
        Test if a ray intersects the triangle using Möller–Trumbore algorithm.
        
        Args:
            p: Ray origin, shape (3,)
            dir: Ray direction (should be normalized), shape (3,)
            
        Returns:
            True if the ray intersects the triangle, False otherwise
        """
        ba = self.b - self.a
        ca = self.c - self.a
        
        h = np.cross(dir, ca)
        proj = np.dot(ba, h)
        
        if abs(proj) < np.finfo(np.float32).eps:
            return False
            
        proj = 1.0 / proj
        
        pa = p - self.a
        u = np.dot(pa, h) * proj
        
        if u < 0.0 or u > 1.0:
            return False
            
        q = np.cross(pa, ba)
        v = np.dot(dir, q) * proj
        
        if v < 0.0 or u + v > 1.0:
            return False
            
        t = np.dot(ca, q) * proj
        
        return t >= 0.0
    
    def intersects_aabb(self, lb: np.ndarray, ub: np.ndarray) -> bool:
        """
        Test if the triangle intersects an axis-aligned bounding box (AABB).
        
        Args:
            lb: Lower bound corner of the AABB, shape (3,)
            ub: Upper bound corner of the AABB, shape (3,)
            
        Returns:
            True if the triangle intersects the AABB, False otherwise
        """
        return np.all(ub >= self.lower_bound) and np.all(lb <= self.upper_bound)
    
    def intersects_sphere(self, point: np.ndarray, radius: float) -> bool:
        """
        Test if the triangle intersects a sphere.
        
        Args:
            point: Center of the sphere, shape (3,)
            radius: Radius of the sphere
            
        Returns:
            True if the triangle intersects the sphere, False otherwise
        """
        center = (self.a + self.b + self.c) / 3.0
        distance = np.linalg.norm(point - center)
        return distance < radius + self.radius
    
    def plane_intersects_aabb(self, lb: np.ndarray, ub: np.ndarray) -> bool:
        """
        Test if the triangle's plane intersects an axis-aligned bounding box (AABB).
        
        Args:
            lb: Lower bound corner of the AABB, shape (3,)
            ub: Upper bound corner of the AABB, shape (3,)
            
        Returns:
            True if the triangle's plane intersects the AABB, False otherwise
        """
        if not self.intersects_aabb(lb, ub):
            return False
            
        center = (ub + lb) / 2.0  # Compute AABB center
        e = ub - center  # Compute positive extents
        
        # Compute the projection interval radius of b onto L(t) = b.c + t * p.n
        r = np.sum(e * np.abs(self.n))
        
        # Compute distance of box center from plane
        s = np.dot(self.n, center) - np.dot(self.n, self.a)
        
        # Intersection occurs when distance s falls within [-r,+r] interval
        return abs(s) <= r