"""
This section shows how to read and visualize an RGBDImage from the TUM dataset [Strum2012].

This tutorial is almost the same as the tutorial processing Redwood dataset above. The only difference is that we use conversion function create_rgbd_image_from_tum_format to parse depth images in the TUM dataset.

本节说明如何从TUM数据集[Strum2012]中读取和可视化RGBDImage。
本教程与上述处理Redwood数据集的教程几乎相同。 唯一的区别是我们使用转换函数create_rgbd_image_from_tum_format解析TUM数据集中的深度图像。
"""
import open3d as o3d
import matplotlib.pyplot as plt

print("Read TUM dataset")
color_raw = o3d.io.read_image("../TestData/RGBD/other_formats/TUM_color.png")
depth_raw = o3d.io.read_image("../TestData/RGBD/other_formats/TUM_depth.png")
rgbd_image = o3d.geometry.RGBDImage.create_from_tum_format(color_raw, depth_raw)
print(rgbd_image)

"""
The color image is converted into a grayscale image, stored in float ranged in [0, 1]. 
The depth image is stored in float, representing the depth value in meters.
"""
plt.subplot(1, 2, 1)
plt.title('TUM grayscale image')
plt.imshow(rgbd_image.color)
plt.subplot(1, 2, 2)
plt.title('TUM depth image')
plt.imshow(rgbd_image.depth)
plt.show()

"""
RGBD图片可以通过相机内参转换成点云图

Here we use PinholeCameraIntrinsicParameters.PrimeSenseDefault as default camera parameter. 
It has image resolution 640x480, focal length (fx, fy) = (525.0, 525.0), and optical center (cx, cy) = (319.5, 239.5).
An identity matrix is used as the default extrinsic parameter. 
pcd.transform applies an up-down flip transformation on the point cloud for better visualization purpose.
"""

# 针孔相机内参
intrinsic = o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
# 通过RGBD图片创建点云图
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
# Flip it, otherwise the pointcloud will be upside down
pcd.transform([
    [1,  0, 0, 0],
    [0, -1, 0, 0],
    [0, 0, -1, 0],
    [0, 0,  0, 1]])

o3d.visualization.draw_geometries([pcd])