"""

http://www.open3d.org/docs/release/tutorial/Basic/icp_registration.html

本教程演示了ICP（迭代最近点）注册算法。 多年来，它一直是研究和工业中几何配准的主体。
输入是两个点云和一个初始转换，该转换将源点云与目标点云大致对齐。 输出是经过精炼的变换，使两点云紧密对齐。
辅助函数draw_registration_result在注册过程中可视化对齐。
在本教程中，我们显示了两个ICP变体，点对点ICP和点对平面ICP [Rusinkiewicz2001]。

http://www.open3d.org/docs/release/tutorial/Basic/icp_registration.html#Point-to-plane-ICP

"""
import open3d as o3d
import numpy as np
import copy

"""
下面的函数将目标点云和源点云可视化，并通过路线转换对其进行转换。 目标点云和源点云分别用青色和黄色绘制。 两点云重叠的越紧密，对齐的结果就越好。

由于函数transform和paint_uniform_color更改了点云，因此我们调用copy.deepcopy进行复制并保护原始点云。
"""
def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


# 下面的代码从两个文件读取源点云和目标点云。 给出了一个粗略的变换。
# 初始对齐通常是通过全局配准算法获得的。 有关示例，请参见全局注册。

source = o3d.io.read_point_cloud("../TestData/ICP/cloud_bin_0.pcd")
target = o3d.io.read_point_cloud("../TestData/ICP/cloud_bin_1.pcd")
threshold = 0.02
trans_init = np.asarray([[0.862, 0.011, -0.507, 0.5],
                         [-0.139, 0.967, -0.215, 0.7],
                         [0.487, 0.255, 0.835, -1.4],
                         [0.0, 0.0, 0.0, 1.0]])
draw_registration_result(source, target, trans_init)

"""
函数validate_registration计算两个主要指标。 

fitness用于测量重叠区域（内部对应数/目标中的点数）。 越高越好。 
inlier_rmse度量所有内联对应点的RMSE。 越低越好。
"""
print("Initial alignment")
evaluation = o3d.registration.evaluate_registration(source, target, threshold, trans_init)
print(evaluation)


print("Apply point-to-plane ICP")
reg_p2l = o3d.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.registration.TransformationEstimationPointToPlane())
print(reg_p2l)
print("Transformation is:")
print(reg_p2l.transformation)
draw_registration_result(source, target, reg_p2l.transformation)

"""
点到面ICP在30次迭代中达到紧密对齐（匹配分0.620972和inlier_rmse 0.006581）。

而点到点的 fitness = 0.621123. inlier_rmse = 0.006583. 需要1000次
"""