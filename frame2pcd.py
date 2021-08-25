from glob import glob
import cv2
import open3d as o3d
import numpy as np


def pcd_save(color, depth, name, resolution):
    if resolution == 3072:
        pinhole_camera = o3d.camera.PinholeCameraIntrinsic(4096, 3072,
                                                           1944.7218017578125,
                                                           1944.041015625,
                                                           2042.5177001953125,
                                                           1562.6834716796875)
    if resolution == 2160:
        pinhole_camera = o3d.camera.PinholeCameraIntrinsic(3840, 2160,
                                                           1823.1766357421875,
                                                           1822.5384521484375,
                                                           1914.8291015625,
                                                           1104.9844970703125)
    if resolution == 720:
        pinhole_camera = o3d.camera.PinholeCameraIntrinsic(1280, 720,
                                                           607.7255859375,
                                                           607.5128173828125,
                                                           637.94305419921875,
                                                           367.99484252929688)


    image_depth = o3d.geometry.Image(depth)
    image_rgb = o3d.geometry.Image(color)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(image_rgb, image_depth, depth_scale=9000,
                                                              convert_rgb_to_intensity=False)

    temp = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_camera)
    o3d.io.write_point_cloud(name, temp)
    # o3d.visualization.draw_geometries([temp])


base_folder = "images_data/images"
folders_path = glob(base_folder + "/*")
folders = [i.split("\\")[1] for i in folders_path]
folders_azure = [i for i in folders if i.__contains__("color")]

base_folder_azure = [base_folder + "/" + i for i in folders_azure]
print(base_folder_azure)
print(len(base_folder_azure))

for file in base_folder_azure:
    print(file)
    point_cloud_name = file.replace("color", "pointCloud").replace("png", "ply")
    depth_name = file.replace("color", "depth").replace("png", "npy")
    color = cv2.imread(file)
    color = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
    depth = np.load(depth_name)
    print(depth.shape)
    pcd_save(color, depth, point_cloud_name, depth.shape[0])
    # break
