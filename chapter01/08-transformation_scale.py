"""

"""

import open3d as o3d
import copy

mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
# 平移
mesh_s = copy.deepcopy(mesh).translate((2, 0, 0))
# 缩放
mesh_s.scale(0.5, center=mesh_s.get_center())

o3d.visualization.draw_geometries([mesh, mesh_s])
