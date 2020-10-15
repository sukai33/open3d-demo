import open3d as o3d
from open3d.open3d_pybind.geometry import TriangleMesh
import numpy as np
import copy

# 创建坐标系
mesh  = o3d.geometry.TriangleMesh.create_coordinate_frame()

print(type(mesh))

# 拷贝一份，x轴平移1.3米
mesh_tx: TriangleMesh = copy.deepcopy(mesh).translate((1.3, 0, 0))

# 拷贝一份，y轴平移1.8米
mesh_ty: TriangleMesh = copy.deepcopy(mesh).translate((0, 1.8, 0))

print(f'Center of mesh: {mesh.get_center()}')
print(f'Center of mesh tx: {mesh_tx.get_center()}')
print(f'Center of mesh ty: {mesh_ty.get_center()}')

# 绘制原始坐标轴，平移后的坐标系
o3d.visualization.draw_geometries([mesh, mesh_tx, mesh_ty])

# ------------------------------------------------------------------------------

