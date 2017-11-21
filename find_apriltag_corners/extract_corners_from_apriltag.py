#!/usr/bin/env python


import json
import cv2
import os
import copy
from numpy import array as npa
import numpy as np

###
#from ctypes import cdll, c_void_p, c_int
#_dll = cdll.LoadLibrary(os.environ['ARC_BASE'] + '/catkin_ws/devel/lib/libikfast_python.so')
#_ikfastAndFindBest = _dll['ikfastAndFindBest']
#_ikfastAndFindBest.argtypes = [c_void_p, c_void_p, c_void_p, c_void_p, c_void_p]
###
from ctypes import cdll, c_void_p, c_int,c_char_p
_dll = cdll.LoadLibrary(os.environ["HOME"] + '/spartan/modules/spartan/mcubeUROP/find_apriltag_corners/build/libfind_corners_helper_python.so')
_find_corners_helper = _dll['find_corners_pythonhelper']
_find_corners_helper.argtypes = [c_void_p, c_void_p]

# save_dir = os.environ["HOME"] + "/software/find_apriltag_corners/camera_calib/"
save_dir = os.environ["SPARTAN_SOURCE_DIR"] + "/sandbox/camera_calib/"
with open(save_dir+'data.json') as data_file:    
    data = json.load(data_file)
  
image = []
image_viz = []
refPt = (0,0)
color = (0,255,255)
color2 = (0,0,255)

#************************template calling cpp from python************************
#def fastik(targetpose, q0, weight = [1,1,1,2,1,1]):
#    _targetpose = np.array(_convertToFloatList(targetpose))
#    _q0 = np.array(_convertToFloatList(q0))
#    _weight = np.array(_convertToFloatList(weight))
#    _solution = np.array([0.0] * 6)
#    _hassol = np.array([0])
#
#    _ikfastAndFindBest(_solution.ctypes.data, _targetpose.ctypes.data, _weight.ctypes.data, _q0.ctypes.data, _hassol.ctypes.data)
#***************************************

def _convertToFloatList(lst):
    return [float(x) for x in lst]

#************************template calling cpp from python************************
def find_corners_cpp(filename):
    _corner_list = np.array(_convertToFloatList([0.0, 0.0] )) # only return centers
    #_corner_list = np.array(_convertToFloatList([0.0, 0.0] * 5)) # return centers and corners
    _filename = c_char_p(str(filename))
    _find_corners_helper(_corner_list.ctypes.data, _filename)
    return _corner_list
#***************************************

newdata = []
for d in data:
    try:
        image = cv2.imread(save_dir + d['pic_path'])
        cv2.namedWindow("image")
        cv2.imshow('image',image)


        while True:
            # display the image and wait for a keypress
            key = cv2.waitKey(3) & 0xFF
            if key == ord("n"):
                break
    except:
        continue
    

    corner_list = find_corners_cpp(save_dir + d['pic_path'])
    print(corner_list)
    d["cross2d"] = corner_list.tolist()[-2:]
    
    if corner_list[0] < 0.001:
        continue
    #corners = np.float32([refPt])
    #d["cross2d"] = corners.tolist()[0]
    newdata.append(d)



import json
with open(str(save_dir)+'data.extracted2d.json', 'w') as outfile:
    json.dump(newdata, outfile)



