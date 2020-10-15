"""

"""

from open3d.open3d_pybind.geometry import Geometry3D
import open3d as o3d
import copy

mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
mesh_s: Geometry3D = copy.deepcopy(mesh).translate((2, 1, 0))
mesh_s.scale(0.5, center=(0, 0, 0))
# mesh_s.scale(0.5, center=mesh_s.get_center())
o3d.visualization.draw_geometries([mesh, mesh_s])
