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
import time
from open3d.open3d_pybind.geometry import Geometry3D
import numpy as np
import copy

def main():
    mesh: Geometry3D = o3d.geometry.TriangleMesh.create_coordinate_frame()
    mesh_r = copy.deepcopy(mesh)
    # 创建欧拉角旋转矩阵
    x_rotation = np.pi / 2  # 90°  1.07
    z_rotation = np.pi / 6  # 30°  0.52
    one_degree = np.pi / 180
    mesh_r.scale(0.8, center=(0,0,0))
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(mesh)
    vis.add_geometry(mesh_r)

    x_step = x_rotation / one_degree
    rotate_around_axis(mesh, mesh_r, (one_degree, 0, 0), vis, x_step)

    z_step = z_rotation / one_degree
    rotate_around_axis(mesh, mesh_r, (0, 0, one_degree), vis, z_step)

    vis.run()

    vis.destroy_window()
    # o3d.visualization.draw_geometries([mesh, mesh_r])


def rotate_around_axis(mesh, mesh_r, rotation, vis, x_step):
    for i in range(0, int(x_step)):
        R = mesh.get_rotation_matrix_from_xyz(rotation)
        mesh_r.rotate(R, center=(0, 0, 0))
        vis.update_geometry(mesh_r)
        vis.poll_events()
        vis.update_renderer()
        time.sleep(0.05)


if __name__ == '__main__':
    main()


