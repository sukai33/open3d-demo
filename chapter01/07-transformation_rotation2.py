"""
Open3D的几何变换也可以通过旋转方法来旋转。
它以旋转矩阵R作为第一个参数。由于可以通过多种方式对3D旋转进行参数化，
因此Open3D提供了方便的功能，可以将不同的参数化转换为旋转矩阵：

1. 使用get_rotation_matrix_from_xyz（其中xyz也可以采用yzx zxy，xzy，zyx和yxz的形式）用欧拉角转换

2. 使用get_rotation_matrix_from_axis_angle 用Axis-angle表示形式进行转换

3. 使用get_rotation_matrix_from_quaternion 用四元数进行转换

在下面的代码中，我们使用欧拉角旋转。
"""

import open3d as o3d
from open3d.open3d_pybind.geometry import Geometry3D
import numpy as np
import copy

mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
mesh_r: Geometry3D = copy.deepcopy(mesh).translate((2,0,0))

# 绕原坐标系中心
# mesh_r.rotate(mesh.get_rotation_matrix_from_xyz((np.pi/2, 0, np.pi / 6)), center=(0, 0, 0))

print(mesh_r.get_center())
# 绕自己的中心
mesh_r.rotate(mesh.get_rotation_matrix_from_xyz((np.pi/2, 0, np.pi/6)), center=mesh_r.get_center())

o3d.visualization.draw_geometries([mesh, mesh_r])