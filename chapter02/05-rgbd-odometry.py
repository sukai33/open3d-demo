"""
RGBD Odometry
http://www.open3d.org/docs/release/tutorial/Basic/rgbd_odometry.html

An RGBD odometry finds the camera movement between two consecutive RGBD image pairs. The input are two instances of RGBDImage. The output is the motion in the form of a rigid body transformation. Open3D implements the method of [Steinbrucker2011] and [Park2017].
RGBD测距法可以找到相机在两个连续的RGBD图像对之间的移动。
输入是RGBDImage的两个实例。 输出是刚体变换形式的运动。
Open3D实现了[Steinbrucker2011]和[Park2017]的方法。

该代码块调用两种不同的RGBD测距方法。 第一个来自[Steinbrucker2011]。
 它将对齐图像的照片一致性降至最低。 第二个来自[Park2017]。 除了照片一致性外，它还实现了几何约束。
 两种功能以相似的速度运行。 但是[Park2017]在我们对基准数据集的测试中更为准确。 推荐。

OdometryOption（）中的几个参数：
1. minimum_correspondence_ratio：对齐后，测量两个RGBD图像的重叠率。 如果两个RGBD图像的重叠区域小于指定比例，则里程计模块将其视为失败情况。
2. max_depth_diff：在深度图像域中，如果两个对齐的像素的深度差小于指定值，则将它们视为对应关系。 值越大，搜索越积极，但结果却不稳定。
3. min_depth和max_depth：小于或大于指定深度值的像素将被忽略。

"""

import open3d as o3d
import numpy as np

pinhole_camera_intrinsic = o3d.io.read_pinhole_camera_intrinsic("../TestData/camera_primesense.json")

print(pinhole_camera_intrinsic.intrinsic_matrix)

source_color = o3d.io.read_image("../TestData/RGBD/color/00000.jpg")
source_depth = o3d.io.read_image("../TestData/RGBD/depth/00000.png")
target_color = o3d.io.read_image("../TestData/RGBD/color/00001.jpg")
target_depth = o3d.io.read_image("../TestData/RGBD/depth/00001.png")
source_rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(source_color, source_depth)
target_rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(target_color, target_depth)

# Open3D假定彩色图像和深度图像已同步并注册在同一坐标系中。 通常可以通过打开RGBD摄像机设置中的同步和注册功能来完成此操作。
target_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(target_rgbd_image, pinhole_camera_intrinsic)

# Compute odometry from two RGBD image pairs

option = o3d.odometry.OdometryOption()
odo_init = np.identity(4)
print(option)

[success_color_term, trans_color_term, info] = o3d.odometry.compute_rgbd_odometry(
         source_rgbd_image, target_rgbd_image, pinhole_camera_intrinsic,
         odo_init, o3d.odometry.RGBDOdometryJacobianFromColorTerm(), option)

[success_hybrid_term, trans_hybrid_term, info] = o3d.odometry.compute_rgbd_odometry(
         source_rgbd_image, target_rgbd_image, pinhole_camera_intrinsic,
         odo_init, o3d.odometry.RGBDOdometryJacobianFromHybridTerm(), option)

# RGBD图像对被转换为点云并一起渲染。 请注意，代表第一张(source) RGBD图像的点云是使用测距法估算的变换来变换的。 转换之后，两个点云都对齐了。

if success_color_term:
    print("Using RGB-D Odometry")
    print(trans_color_term)
    source_pcd_color_term = o3d.geometry.PointCloud.create_from_rgbd_image(source_rgbd_image, pinhole_camera_intrinsic)
    source_pcd_color_term.transform(trans_color_term)

    o3d.visualization.draw_geometries([target_pcd, source_pcd_color_term])

if success_hybrid_term:
    print("Using Hybrid RGB-D Odometry")
    print(trans_hybrid_term)
    source_pcd_hybrid_term = o3d.geometry.PointCloud.create_from_rgbd_image(source_rgbd_image, pinhole_camera_intrinsic)
    source_pcd_hybrid_term.transform(trans_hybrid_term)

    o3d.visualization.draw_geometries([target_pcd, source_pcd_hybrid_term])