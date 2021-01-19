import numpy as np
import cv2
import cv2.aruco as aruco
import pyrealsense2 as rs
import copy
import threading
from datetime import datetime
import sys, time
from threading import Timer
import math
from scipy.linalg import expm, norm
diff_w = np.asarray([0.04,-0.022,0.285])
#diff_w = np.asarray([0.0,0.0,0.0])
def M(axis, theta):
    return expm(np.cross(np.eye(3), axis/norm(axis)*theta))
class Wheelchair():
    def __init__(self):
        #To do: change num1, num2 to a list of num
        self.aruco_num1 = 1
        self.aruco_num2 = 15
        self.in_camera = False
        self.num_xyz = {'num1': None , 'num2': None }
    def wheelchair_dec(self,ids,corners,depth_img):
        num = int(len(ids))
        for id in range(num):
            if ids[id] == self.aruco_num1:
                self.num_xyz['num1'] = get_xyz(corners[id], depth_img)
                self.in_camera = True
            if ids[id] == self.aruco_num2:
                self.num_xyz['num2'] = get_xyz(corners[id], depth_img)        
def unit_vector(vector):
    """ Returns the unit vector of the vector."""
    return vector / np.linalg.norm(vector)
class obj():
    def __init__(self,num1,num2,num3):
        self.aruco_num1 = num1
        self.aruco_num2 = num2
        self.aruco_num3 = num3
        self.in_camera = False
        self.xyz1 = None
        self.xyz2 = None
        self.xyz3 = None
    def obj_dec(self,ids,corners,depth_img):
        for id in range(len(ids)):
            if ids[id] == self.aruco_num1:
                self.xyz1 = get_xyz(corners[id], depth_img)
                self.in_camera = True
            if ids[id] == self.aruco_num2:
                self.xyz2 = get_xyz(corners[id], depth_img)
            if ids[id] == self.aruco_num3:
                self.xyz3 = get_xyz(corners[id], depth_img)
    def compute_obj2wheelchair_base(self,wheelchair):
        w_Q1 = np.asarray(wheelchair.num_xyz['num1'])
        w_Q2 = np.asarray(wheelchair.num_xyz['num2'])
        o_Q1 = np.asarray(self.xyz1)
        o_Q2 = np.asarray(self.xyz2)
        axis_x_c = unit_vector( w_Q2 - w_Q1 )
        axis_z_c = unit_vector( o_Q2 - o_Q1 )
        axis_y_c = - np.cross(axis_z_c,axis_x_c)
        RT = np.array([axis_x_c, axis_y_c, axis_z_c])
        RT = np.asmatrix(RT)
        RT_i = np.linalg.inv(RT)
        o2w = o_Q2 - w_Q2
        o2w = o2w.dot(RT_i)
        o_Q3 = np.asarray(self.xyz3)
        axis_x_o = unit_vector( o_Q2 - o_Q3 )  #objet 
        axis_z_o = unit_vector( o_Q2 - o_Q1 )
        axis_y_o = - np.cross( axis_z_o,axis_x_o)
        RT_o = np.array([axis_x_o, axis_y_o, axis_z_o])
        RT_o = np.asmatrix(RT_o)
        RT_o_i = np.linalg.inv(RT_o)
        o_axis = o_Q3.dot(RT_o_i)
        o_axis[0,1] = o_axis[0,1]+0.15
        o_axis[0,2] = o_axis[0,2]-0.15
        obj_c = o_axis.dot(RT_o)
        obj_w = obj_c - w_Q2
        obj2w = obj_w.dot(RT_i)
        obj2w = - obj2w - diff_w 
        #print(obj2w)
        return obj2w
def get_xyz(conner,depth_img):
    camera_cx = 340.745
    camera_cy = 245.132
    camera_fx = 615.376
    camera_fy = 614.775
    conner_p = conner[0]
    n,m = int(np.mean(conner_p[:,0])),int(np.mean(conner_p[:,1]))
    z = depth_img[m,n]/1000.0
    x = (n - camera_cx) * z / camera_fx
    y = (m - camera_cy) * z / camera_fy
    return np.array([x,y,z])
def aruco_init():
    global pipe, find_wheel_chair
    cap = cv2.VideoCapture(0)
    pipe = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    profile = pipe.start(config)
    find_wheel_chair = 0
def aruco_fun_compute():
    global pipe
    axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)
    frames = pipe.wait_for_frames()
    align_to = rs.stream.color
    align = rs.align(align_to)
    aligned_frames = align.process(frames)
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    color_img = np.array(color_frame.get_data())
    color_img_temp = copy.deepcopy(color_img)
    depth_img = np.array(depth_frame.get_data())
    frame = color_img_temp 
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    #print(ids)
    wheelchair = Wheelchair()
    obj1 = obj(13,5,6)
    if ids is not None and len(ids) > 0:
        wheelchair.wheelchair_dec(ids,corners,depth_img)
        obj1.obj_dec(ids,corners,depth_img)
    if wheelchair.in_camera and obj1.in_camera:
        print("let us see the distance in camera")
        pipe.stop()
        return obj1.compute_obj2wheelchair_base(wheelchair)
    pipe.stop()
    return [0.0,0.0,0.0]
def aruco_fun():
    global pipe
    axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)
    frames = pipe.wait_for_frames()
    align_to = rs.stream.color
    align = rs.align(align_to)
    aligned_frames = align.process(frames)
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    color_img = np.array(color_frame.get_data())
    color_img_temp = copy.deepcopy(color_img)
    depth_img = np.array(depth_frame.get_data())
    frame = color_img_temp 
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    wheelchair = Wheelchair()
    obj1 = obj(5,13,6)
    if ids is not None and len(ids) > 0:
        wheelchair.wheelchair_dec(ids,corners,depth_img)
        obj1.obj_dec(ids,corners,depth_img)
    if wheelchair.in_camera and obj1.in_camera:
        print("let us see the distance in camera")
        obj1.compute_obj2wheelchair_base(wheelchair)
    gray = aruco.drawDetectedMarkers(gray, corners)
    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
def get_obj2w():
    aruco_init()
    obj2w = aruco_fun_compute()
    return obj2w