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
import trimesh
from typing import List, Tuple, Dict, Optional
import time
import math
import argparse
import sys
import os
from triangle_grid import TriangleGrid
from prepared_triangle import PreparedTriangle

class ModelProcessor:
    """
    Handles loading and processing of 3D models for distance field generation.
    Uses trimesh for GLTF/GLB file loading and works with TriangleGrid for
    distance field generation.
    """
    
    def __init__(self):
        """Initialize the model processor."""
        pass
    
    def load_and_process(self, filename: str, output_file: str, lod0_pixels: int = 256,
                        top_lod_cell_size: int = 8, scale: float = 1.0,
                        selected_indices: Optional[List[int]] = None) -> Dict:
        """
        Load a 3D model file and generate its distance field.
        
        Args:
            filename: Path to GLTF/GLB file
            output_file: Path to save the distance field data
            lod0_pixels: Maximum pixel size for LOD 0 in any dimension
            top_lod_cell_size: Cell size for highest LOD
            scale: Scale factor to apply to the model
            selected_indices: List of mesh indices to process (None for all)
            
        Returns:
            Dictionary with processing metadata
        """
        start_time = time.time()
        print(f"[{time.time() - start_time:.2f}] Processing file {filename}")
        
        # Load the model using trimesh
        scene = trimesh.load(filename)
        print(f"[{time.time() - start_time:.2f}] File loaded")
        
        # Process scene and collect triangles
        triangles, scene_min, scene_max = self.prepare_scene(
            scene, scale, selected_indices)
        
        # Calculate scene dimensions
        max_side = max(
            scene_max[2] - scene_min[2],
            scene_max[1] - scene_min[1],
            scene_max[0] - scene_min[0]
        )
        
        # Calculate conversion factors and dimensions
        padded_top_lod_cell_size = top_lod_cell_size
        top_lod_cell_size -= 1
        
        scene_to_pixels = lod0_pixels / max_side
        pixels_to_scene = 1.0 / scene_to_pixels
        
        # Calculate grid dimensions
        sx = int(math.ceil((scene_max[0] - scene_min[0]) * scene_to_pixels))
        sy = int(math.ceil((scene_max[1] - scene_min[1]) * scene_to_pixels))
        sz = int(math.ceil((scene_max[2] - scene_min[2]) * scene_to_pixels))
        
        # Calculate padding
        padx = top_lod_cell_size - (sx % top_lod_cell_size) if sx % top_lod_cell_size != 0 else top_lod_cell_size
        pady = top_lod_cell_size - (sy % top_lod_cell_size) if sy % top_lod_cell_size != 0 else top_lod_cell_size
        padz = top_lod_cell_size - (sz % top_lod_cell_size) if sz % top_lod_cell_size != 0 else top_lod_cell_size
        
        sx += padx + 1
        sy += pady + 1
        sz += padz + 1
        
        # Calculate bounds with padding
        padding = np.array([padx, pady, padz]) * pixels_to_scene * 0.5
        lower_bound = scene_min - padding
        upper_bound = scene_max + padding
        
        print(f"[{time.time() - start_time:.2f}] File preprocessed. "
              f"X: {sx}, Y: {sy}, Z: {sz}, maximum distance: {max_side}")
        
        # Create triangle grid and generate distance field
        triangle_grid = TriangleGrid(scene_min, scene_max, sx, sy, sz, triangles)
        
        print(f"[{time.time() - start_time:.2f}] Triangle grid ready. ")
        
        distance_data = triangle_grid.dispatch(
            lower_bound,
            pixels_to_scene,
            scene_to_pixels / padded_top_lod_cell_size,
            sx, sy, sz
        )
        
        # Save distance field data to file
        distance_data.astype(np.float32).tofile(output_file)
        print(f"[{time.time() - start_time:.2f}] Distance field saved to {output_file}")
        
        metadata = {
            'processing_time': time.time() - start_time,
            'grid_dimensions': (sx, sy, sz),
            'scene_bounds': {
                'min': scene_min,
                'max': scene_max
            },
            'scale_factors': {
                'scene_to_pixels': scene_to_pixels,
                'pixels_to_scene': pixels_to_scene
            },
            'triangle_count': len(triangles)
        }
        
        return metadata
    
    def prepare_scene(self, scene: trimesh.Scene, scale: float,
                     selected_indices: Optional[List[int]] = None) -> Tuple[List[PreparedTriangle], np.ndarray, np.ndarray]:
        """
        Process scene geometry and prepare triangles.
        
        Args:
            scene: Loaded trimesh scene
            scale: Scale factor to apply
            selected_indices: List of mesh indices to process (None for all)
            
        Returns:
            Tuple containing:
            - List of PreparedTriangle objects
            - Scene minimum bounds
            - Scene maximum bounds
        """
        triangles = []
        scene_min = np.array([float('inf')] * 3)
        scene_max = np.array([float('-inf')] * 3)
        
        # Convert scene to mesh if it's not already
        if isinstance(scene, trimesh.Scene):
            meshes = [mesh for mesh in scene.geometry.values()]
        else:
            meshes = [scene]
        
        # Filter meshes based on selection criteria
        if selected_indices is not None:
            meshes = [mesh for i, mesh in enumerate(meshes) if i in selected_indices]
        
        triangle_idx = 0
        for mesh in meshes:
            vertices = mesh.vertices * scale
            faces = mesh.faces
            
            # Update scene bounds
            mesh_min = np.min(vertices, axis=0)
            mesh_max = np.max(vertices, axis=0)
            scene_min = np.minimum(scene_min, mesh_min)
            scene_max = np.maximum(scene_max, mesh_max)
            
            # Create triangles
            for face in faces:
                triangle = PreparedTriangle.from_vertex_array(
                    triangle_idx,
                    vertices,
                    face[0], face[1], face[2]
                )
                triangles.append(triangle)
                triangle_idx += 1
        
        return triangles, scene_min, scene_max

def print_model_info(filename: str):
    """Print all model names with their zero-based indices."""
    scene = trimesh.load(filename)
    
    if isinstance(scene, trimesh.Scene):
        print("\nAvailable meshes in the model:")
        print("Index | Name")
        print("-" * 30)
        for i, (name) in enumerate(scene.geometry.items()):
            print(f"{i:5d} | {name}")
        print()
    else:
        print("\nModel contains a single mesh")
        print("Index | Name")
        print("-" * 30)
        print(f"    0 | main_mesh")
        print()

def main():
    parser = argparse.ArgumentParser(description='Process 3D models to generate distance fields.')
    parser.add_argument('input_file', help='Input GLTF/GLB file path')
    parser.add_argument('output_file', help='Output distance field file path')
    parser.add_argument('--list', action='store_true', help='List available meshes and exit')
    parser.add_argument('--scale', type=float, default=1.0, help='Scale factor to apply to the model')
    parser.add_argument('--pixels', type=int, default=256, help='Maximum pixel size for LOD 0')
    parser.add_argument('--top-lod-cell-size', type=int, default=8, help='Cell size for highest LOD')
    parser.add_argument('--indices', type=int, nargs='*', help='Indices of meshes to process')
    
    args = parser.parse_args()
    
    # Check if input file exists
    if not os.path.exists(args.input_file):
        print(f"Error: Input file '{args.input_file}' does not exist")
        sys.exit(1)
    
    # Just print model info if --list is specified
    if args.list:
        print_model_info(args.input_file)
        sys.exit(0)
    
    # Create output directory if it doesn't exist
    output_dir = os.path.dirname(args.output_file)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Process the model
    processor = ModelProcessor()
    metadata = processor.load_and_process(
        args.input_file,
        args.output_file,
        lod0_pixels=args.pixels,
        top_lod_cell_size=args.top_lod_cell_size,
        scale=args.scale,
        selected_indices=args.indices
    )
    
    # Print summary
    print("\nProcessing Summary:")
    print(f"Time taken: {metadata['processing_time']:.2f} seconds")
    print(f"Grid dimensions: {metadata['grid_dimensions']}")
    print(f"Triangle count: {metadata['triangle_count']}")

if __name__ == "__main__":
    main()