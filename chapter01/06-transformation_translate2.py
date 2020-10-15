import open3d as o3d
from open3d.open3d_pybind.geometry import TriangleMesh
import numpy as np
import copy

# 创建坐标系
mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()

# 拷贝一份，平移到(2,2,2)
mesh_mv = copy.deepcopy(mesh).translate((2, 2, 2), relative=False)

print(f'Center of mesh: {mesh.get_center()}')
print(f'Center of translated mesh: {mesh_mv.get_center()}')

o3d.visualization.draw_geometries([mesh, mesh_mv])

# ------------------------------------------------------------------------------
