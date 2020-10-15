"""
In this section we show how to read and visualize an RGBDImage from the Redwood dataset [Choi2015].

The Redwood format stored depth in a 16-bit single channel image. The integer value represents the depth measurement in millimeters. It is the default format for Open3D to parse depth images.

Redwood格式将深度存储在16位单通道图像中。 整数值表示以mm毫米为单位的深度测量值。 这是Open3D解析深度图像的默认格式。

"""
import open3d as o3d
import matplotlib.pyplot as plt

print("Read Redwood dataset")
color_raw = o3d.io.read_image("../TestData/RGBD/color/00000.jpg")
depth_raw = o3d.io.read_image("../TestData/RGBD/depth/00000.png")
rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw)

print(rgbd_image)

"""
The color image is converted into a grayscale image, stored in float ranged in [0, 1]. 
The depth image is stored in float, representing the depth value in meters.
"""
plt.subplot(1, 2, 1)
plt.title('Redwood grayscale image')
plt.imshow(rgbd_image.color)
plt.subplot(1, 2, 2)
plt.title('Redwood depth image')
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