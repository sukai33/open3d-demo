"""
This section shows how to read and visualize an RGBDImage from the NYU dataset [Silberman2012].
This tutorial is almost the same as the tutorial processing Redwood dataset above, with two differences. First, NYU images are not in standard jpg or png formats. Thus, we use mpimg.imread to read the color image as a numpy array and convert it to an Open3D Image. An additional helper function read_nyu_pgm is called to read depth images from the special big endian pgm format used in the NYU dataset. Second, we use a different conversion function create_rgbd_image_from_nyu_format to parse depth images in the SUN dataset.

本节说明如何从NYU数据集中读取和可视化RGBDImage [Silberman2012]。

本教程与上述处理Redwood数据集的教程几乎相同，但有两个区别。 首先，NYU图像不是标准的jpg或png格式。
因此，我们使用mpimg.imread来将彩色图像读取为numpy数组并将其转换为Open3D图像。
调用了一个附加的辅助函数read_nyu_pgm，以从NYU数据集中使用的特殊大尾数pgm格式读取深度图像。
其次，我们使用不同的转换函数create_rgbd_image_from_nyu_format来解析SUN数据集中的深度图像。
"""
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import re

# This is special function used for reading NYU pgm format
# as it is written in big endian byte order.
def read_nyu_pgm(filename, byteorder='>'):
    with open(filename, 'rb') as f:
        buffer = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
    except AttributeError:
        raise ValueError("Not a raw PGM file: '%s'" % filename)
    img = np.frombuffer(buffer,
                        dtype=byteorder + 'u2',
                        count=int(width) * int(height),
                        offset=len(header)).reshape((int(height), int(width)))
    img_out = img.astype('u2')
    return img_out


print("Read NYU dataset")
# Open3D does not support ppm/pgm file yet. Not using o3d.io.read_image here.
# MathplotImage having some ISSUE with NYU pgm file. Not using imread for pgm.
color_raw = mpimg.imread("../TestData/RGBD/other_formats/NYU_color.ppm")
depth_raw = read_nyu_pgm("../TestData/RGBD/other_formats/NYU_depth.pgm")
color = o3d.geometry.Image(color_raw)
depth = o3d.geometry.Image(depth_raw)
rgbd_image = o3d.geometry.RGBDImage.create_from_nyu_format(color, depth)
print(rgbd_image)

plt.subplot(1, 2, 1)
plt.title('NYU grayscale image')
plt.imshow(rgbd_image.color)
plt.subplot(1, 2, 2)
plt.title('NYU depth image')
plt.imshow(rgbd_image.depth)
plt.show()

intrinsic = o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)

# Flip it, otherwise the pointcloud will be upside down
pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
o3d.visualization.draw_geometries([pcd])