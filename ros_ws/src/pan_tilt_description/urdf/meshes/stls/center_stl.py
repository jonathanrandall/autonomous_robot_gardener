#!/usr/bin/env python3

from stl import mesh
import numpy as np
import sys


def get_bounding_box_dimensions(stl_mesh):
    """Return bounding-box dimensions (dx, dy, dz), and (min, max) coordinates.

    Args:
        stl_mesh: instance of stl.mesh.Mesh

    Returns:
        dims: numpy array [dx, dy, dz]
        bbox_min: numpy array [min_x, min_y, min_z]
        bbox_max: numpy array [max_x, max_y, max_z]
    """
    vertices = stl_mesh.vectors.reshape(-1, 3)
    bbox_min = vertices.min(axis=0)
    bbox_max = vertices.max(axis=0)
    dims = bbox_max - bbox_min
    return dims, bbox_min, bbox_max

# Get input and output filenames
if len(sys.argv) > 1:
    input_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else input_file.replace('.stl', '_fixed.stl')
else:
    input_file = 'pi_stand_final_v2-Body_v2.stl'
    output_file = 'pi_stand_final_v2-Body_v2_fixed.stl'

# Load the STL file
stl_mesh = mesh.Mesh.from_file(input_file)

# Get bounding-box dims and show them
dims, bbox_min, bbox_max = get_bounding_box_dimensions(stl_mesh)
print(f'Bounding box min: {bbox_min}, max: {bbox_max}, dims (x,y,z): {dims}')

# Calculate bounding box center and center the mesh
bbox_center = (bbox_max + bbox_min) / 2
print(f'Bounding box center before: {bbox_center}')
stl_mesh.vectors -= bbox_center

# Verify new bounding box/dimensions after centering
dims_new, bbox_min_new, bbox_max_new = get_bounding_box_dimensions(stl_mesh)
print(f'Bounding box min after: {bbox_min_new}, max after: {bbox_max_new}, dims after: {dims_new}')

# Save the centered mesh
stl_mesh.save(output_file)

print(f'STL file centered successfully!')
print(f'Output saved to: {output_file}')

# from stl import mesh
# import numpy as np
# import sys

# # Get input and output filenames
# if len(sys.argv) > 1:
#     input_file = sys.argv[1]
#     output_file = sys.argv[2] if len(sys.argv) > 2 else input_file.replace('.stl', '_fixed.stl')
# else:
#     input_file = 'PAN-TILT_BASE_V1.stl'
#     output_file = 'PAN-TILT_BASE_V1_fixed.stl'

# # Load the STL file
# stl_mesh = mesh.Mesh.from_file(input_file)

# # Get all vertices (reshape from faces to individual vertices)
# vertices = stl_mesh.vectors.reshape(-1, 3)

# # Calculate bounding box min and max
# bbox_min = vertices.min(axis=0)
# bbox_max = vertices.max(axis=0)

# # Compute translation so the bottom sits at z=0
# translation = np.array([0, 0, -bbox_min[2]])

# print(f"Original bounding box min: {bbox_min}, max: {bbox_max}")
# print(f"Translation applied: {translation}")

# # Apply translation
# stl_mesh.vectors += translation  # move mesh up so bottom is at z=0

# # Verify new bounding box
# vertices_new = stl_mesh.vectors.reshape(-1, 3)
# bbox_min_new = vertices_new.min(axis=0)
# bbox_max_new = vertices_new.max(axis=0)
# print(f"New bounding box min: {bbox_min_new}, max: {bbox_max_new}")

# # Save the corrected mesh
# stl_mesh.save(output_file)

# print(f'STL file bottom-aligned successfully!')
# print(f'Output saved to: {output_file}')
