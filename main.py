import sys
import cv2
from PyQt5.QtCore import QLine, QRectF, QPoint, QDir
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import QtGui, QtCore, QtWidgets
from arayuz.Gui import Ui_MainWindow
import argparse
import os
import time
import open3d as o3d
import datetime
import json
import pyrealsense2 as rs
import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import ( NavigationToolbar2QT as NavigationToolbar)
import matplotlib.pyplot as plt


class ViewerWithCallback(QtCore.QThread):

    changePcd = QtCore.pyqtSignal(object)

    def __init__(self, config, device, align_depth_to_color):
        super().__init__()
        self.isAzureRunnig = False
        self.flag_exit = False
        self.config = config
        self.align_depth_to_color = align_depth_to_color
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        print("thread")
        self.isAzure = True
        self.isFrameSave = False
        self.isSeriesSave = False
        self.azure_run()
        self.is_3d_view = False
        self.frame_name = ""
        self.series_count = 0
        self.colorizer = rs.colorizer()
        self.colorizer.set_option(rs.option.max_distance, 3)  # 0=Dynamic, 1=Fixed, 2=Near, 3=Far
        print("thread2")

    def find_device_that_supports_advanced_mode(self):
        ctx = rs.context()
        devices = ctx.query_devices()
        for dev in devices:
            print("Found device:", dev.get_info(rs.camera_info.name))
        return dev

    def azure_run(self):
        if not self.isAzureRunnig:
            self.sensor = o3d.io.AzureKinectSensor(self.config)
            if not self.sensor.connect(device):
                raise RuntimeError('Failed to connect to sensor')
            # self.isAzure = True
            self.isAzureRunnig = True


    def escape_callback(self, vis):
        self.flag_exit = True
        return False

    def get_intrinsic_matrix_azure(self):

        out = o3d.camera.PinholeCameraIntrinsic(3840, 2160,
                                                           1823.1766357421875,
                                                           1822.5384521484375,
                                                           1914.8291015625,
                                                           1104.9844970703125)
        return out


    def run(self):
        t_now = None
        series_save = False
        list_depth = []
        list_color = []
        counter = 0
        plt.figure(5)
        plot_azure = True
        ax1 = plt.subplot(1, 2, 1)
        ax2 = plt.subplot(1 ,2, 2, sharex=ax1, sharey=ax1)
        counter_kayit=0
        self.vis_geometry_added = False
        while not self.flag_exit:
            if self.isAzureRunnig:
                rgbd = self.sensor.capture_frame(True)
                if rgbd is None:
                    continue
                depth = rgbd.depth
                color = rgbd.color
                depth_array = np.array(depth)
                color_array = np.array(color)

                color_array = cv2.cvtColor(color_array, cv2.COLOR_BGR2RGB)

                if plot_azure:
                    fig1 = ax1.imshow(depth_array)
                    fig2 = ax2.imshow(color_array)
                    plot_azure = False

                else:
                    fig1.set_data(depth_array)
                    fig2.set_data(color_array[:,:,::-1])
                    plt.pause(0.01)

                if self.isFrameSave:
                    self.pcd_save( color_array, depth_array)
                    print("pcd, color image, depth image saved")
                    self.isFrameSave = False

                if self.isSeriesSave:
                    counter_kayit = 0
                    series_save = True
                    self.isSeriesSave = False
                    t_now = time.time() + 3

                if series_save and len(list_color) < self.series_count:
                    print(self.series_count)
                    list_color.append(color_array)
                    print(len(list_color), " - ", series_save)
                    list_depth.append(depth_array)
                    counter_kayit += 1

                elif series_save:
                    print("************")
                    if self.frame_name == "":

                        if self.isAzure:
                            now = datetime.datetime.now()
                            self.frame_name = now.strftime("%m-%d-%Y_%H-%M-%S") + "_azure_"
                        else:
                            now = datetime.datetime.now()
                            self.frame_name = now.strftime("%m-%d-%Y_%H-%M-%S") + "_lidar_"
                    else:

                        if self.isAzure:
                            self.frame_name = self.frame_name + "_azure"
                        else:
                            self.frame_name = self.frame_name + "_lidar"
                    img_path = "../plane_fit/images_data/images_data/series_save/"
                    path = os.path.join(img_path, self.frame_name)
                    os.mkdir(path)

                    cnt = 0
                    for i in range(len(list_color)):
                        cv2.imwrite(path + "/color_" + str(cnt) + ".png", list_color[i])
                        np.save(path + "/depth_" + str(cnt) + ".npy", list_depth[i])
                        cnt = cnt + 1
                    series_save = False
                    print(str(len(list_color)) + " pcd file saved in " + self.frame_name + " folder")
                    list_color.clear()
                    list_depth.clear()
                data = {"pcd": "null"}
                self.changePcd.emit(data)
            else:
                print("paused frame")

    def pcd_save(self, color_img, depth_img):
        if self.frame_name == "":
            if self.isAzure:
                now = datetime.datetime.now()
                self.frame_name = now.strftime("%m-%d-%Y_%H-%M-%S") + "_azure_"
            else:
                now = datetime.datetime.now()
                self.frame_name = now.strftime("%m-%d-%Y_%H-%M-%S") + "_lidar_"
        # o3d.io.write_point_cloud("data/pcd/" + self.frame_name + ".pcd", pcd)
        cv2.imwrite("images_data/images/color_" + self.frame_name + ".png", color_img)
        np.save("images_data/images/depth_" + self.frame_name + ".npy", depth_img)
        # cv2.imwrite("data/images/depth_" + self.frame_name + ".png", depth_img)
        self.frame_name = ""

    def stop(self):
        self.flag = False

    def close_azure(self):
        if self.isAzureRunnig:
            del self.sensor
            self.isAzure = False
            self.isAzureRunnig = False


class Main(QMainWindow, Ui_MainWindow):

    def __init__(self,config, device):
        super().__init__()
        self.setupUi(self)
        self.config = config
        self.device = device
        self.align_depth_to_color = True
        print("main")
        self.th = ViewerWithCallback(self.config, self.device, self.align_depth_to_color)
        self.th.changePcd.connect(self.set_pcd)
        self.th.start()
        self.active_btn_style = "border:2px solid rgb(255,255,255);\nbackground-color: rgb(59, 129, 48);\ncolor: rgb(255, 255, 255);"
        self.deactive_btn_style = "border:2px solid rgb(255,255,255);\nbackground-color: rgb(100, 100, 100);\ncolor: rgb(255, 255, 255);"
        # self.btn_open3d.clicked.connect(self.open3d_click)
        # self.btn_matplotlib.clicked.connect(self.matplotlib_click)
        self.btn_capture.clicked.connect(self.capture_frame)
        self.btn_series.clicked.connect(self.series_frame)

    @QtCore.pyqtSlot(object)
    def set_pcd(self, data):
        print("-----------------")

    def capture_frame(self):
        self.th.frame_name = self.txt_name.toPlainText()
        self.th.isFrameSave = True

    def series_frame(self):
        self.th.frame_name = self.txt_name.toPlainText()
        self.th.series_count = self.spinBox.value()
        self.th.isSeriesSave = True


if __name__ == "__main__":
    config = o3d.io.read_azure_kinect_sensor_config('config.json')
    print(config)
    device = 0
    isMain = True
    app = QApplication(sys.argv)
    window = Main(config, device)
    window.show()
    sys.exit(app.exec_())