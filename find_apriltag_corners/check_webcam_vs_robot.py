#!/usr/bin/env python
# Get an array of 
# [3D apriltag pose from vicon in world frame, apriltag 3D position from detector in webcam frame]

# want to find the mapping between webcam frame wrt world.

# from ik.helper import *
# from ik.roshelper import *
# from apriltags.msg import AprilTagDetections
import tf
import tf.transformations as tfm
#from rigid_transform_3D import rigid_transform_3D
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
#from robot_comm.srv import *
#import config.helper as helper


# system
import os

# spartan
import spartan.utils.ros_utils as rosUtils

# ros
import rospy
import geometry_msgs.msg

pts_vicon = []
pts_apriltag = []

# import pdb; pdb.set_trace()
rospy.init_node('robot_vs_webcam')
listener = tf.TransformListener()


# back left pose (0.244, -0.495, 0.472)
# front left pose (0.696, -0.333, 0.460)
limits = [0.244, 0.69, -0.495, +0.495, 0.30, 0.50]  #[xmin, xmax, ymin, ymax, zmin, zmax]
nseg = [6, 6, 3]
nrotate = 1
ori = [0.19079229, -0.67772497,  0.16148602,  0.69152688] # observer
cam_id = 'observer'  
# image_topic = cam_id + '/image_raw'

#position = [0.51102088,  0.01480124,  0.50132378]
#quat = [ 0.19079229, -0.67772497,  0.16148602,  0.69152688] # w,x,y,z

image_topic = "/camera_xtion_ebay/ir/image"
# image_topic = "/camera_xtion_ebay/rgb/image_raw"

globalacc = 2             # some big number means no limit, in m/s^2

#setCartRos = rospy.ServiceProxy('/robot2_SetCartesian', robot_SetCartesian)
#setZone = rospy.ServiceProxy('/robot2_SetZone', robot_SetZone)
#setAcc = rospy.ServiceProxy('/robot2_SetAcc', robot_SetAcc)
#setSpeed = rospy.ServiceProxy('/robot2_SetSpeed', robot_SetSpeed)


jointNames = ['iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3', 'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7']
robotService = rosUtils.RobotService(jointNames)


ntfretry = 10
def lookupTransform(homeFrame, targetFrame, listener):
    #while not rospy.is_shutdown():
    for i in range(ntfretry):
        try:
            t = rospy.Time(0)
            (trans,rot) = listener.lookupTransform(homeFrame, targetFrame, t)
            return (trans,rot)
        except: #(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print '[lookupTransform] failed to transform'
            print '[lookupTransform] targetFrame %s homeFrame %s, retry %d' % (targetFrame, homeFrame, i)
            rospy.sleep(retryTime)
    print traceback.format_exc()
    return None, None


def lookupTransformList(homeFrame, targetFrame, listener):
    #while not rospy.is_shutdown():
    for i in range(ntfretry):
        try:
            t = rospy.Time(0)
            (trans,rot) = listener.lookupTransform(homeFrame, targetFrame, t)
            return list(trans) + list(rot)
        except: #(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print '[lookupTransform] failed to transform'
            print '[lookupTransform] targetFrame %s homeFrame %s, retry %d' % (targetFrame, homeFrame, i)
            rospy.sleep(retryTime)
    print traceback.format_exc()
    return None

    


def setCart(pos, ori): 											#pos in meters [x, y, z], orientation in quaternions [qw, qx, qy, qz]
    #param = (np.array(pos) * 1000).tolist() + ori
    #print 'setCart', param
    #pause()
    #setCartRos(*param) 											#replace with own ros service for commanding arm pose

    # this should be pose of "iiwa_link_ee"
    print 'setCart ', pos
    poseStamped = geometry_msgs.msg.PoseStamped()
    poseStamped.pose.position.x = pos[0]
    poseStamped.pose.position.y = pos[1]
    poseStamped.pose.position.z = pos[2]
    poseStamped.pose.orientation.w = ori[0]
    poseStamped.pose.orientation.x = ori[1]
    poseStamped.pose.orientation.y = ori[2]
    poseStamped.pose.orientation.z = ori[3]
    ikSuccess = robotService.moveToCartesianPosition(poseStamped)
    return ikSuccess
    
#setZone(0) ## user may have different setZone, setSpeed, setAcc. change accrodingly
#setSpeed(200, 60)
#setAcc(acc=globalacc, deacc=globalacc)
#
bridge = CvBridge()


world_frame_id = "base"


data = []
#save_dir = os.environ["DATA_BASE"] + "/camera_calib/"
# save_dir = os.environ["HOME"] + "/software/find_apriltag_corners/camera_calib/"

save_dir = os.environ["SPARTAN_SOURCE_DIR"] + "/sandbox/camera_calib/"

import shutil
try:
    shutil.rmtree(save_dir)
except:
    pass    
# os.rmdir(save_dir)
cmd = "mkdir -p " + save_dir
# os.mkdir(save_dir)
os.system(cmd)



# helper.make_sure_path_exists(save_dir)

for x in np.linspace(limits[0],limits[1], nseg[0]):
    for y in np.linspace(limits[2],limits[3], nseg[1]):
        for z in np.linspace(limits[4],limits[5], nseg[2]):
            ikSuccess = setCart([x,y,z], ori)
            if not ikSuccess:
                print "ik failed, skipping"
                continue			
            # get robot 3d point
            
            rospy.sleep(0.1)
            cross_poselist = lookupTransformList(world_frame_id,'/cross_tip', listener) # user needs to change this tf, defined in launch file (tf between link 6 and cross/apriltag)
            
            # take picture of camera
            msg = rospy.wait_for_message(image_topic, Image) # user needs to change this topic
            timestr = "%.6f" % msg.header.stamp.to_sec()

            image_name = str(save_dir)+timestr+".jpeg"
            # print "msg ", msg
            print "encoding = ", msg.encoding
            try:
                cv_image = bridge.imgmsg_to_cv2(msg, msg.encoding) # for the IR
            except CvBridgeError, e:
                print(e)
            cv2.imwrite(image_name, cv_image)
            
            data.append({"cross3d": cross_poselist, "pic_path": timestr+".jpeg"})
    
import json
with open(save_dir+'data.json', 'w') as outfile:
    json.dump(data, outfile)


# point3d = [for p["cross3d"] in data]
# point2d = [for p["cross2d"] in data]
