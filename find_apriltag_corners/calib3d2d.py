#!/usr/bin/env python
import numpy as np
from scipy.optimize import least_squares
import tf.transformations as tfm
import cv2
from numpy import array as npa
import os, json

import yaml
import spartan.utils.utils as spartanUtils

# save_dir = os.environ["HOME"] + "/software/find_apriltag_corners/camera_calib/"
save_dir = os.environ["SPARTAN_SOURCE_DIR"] + "/sandbox/camera_calib/"
def quat_to_rod(q):
    # q = [qx, qy, qz, qw]
    rot = tfm.quaternion_matrix(q)[0:3][:, 0:3]
    dst, jacobian = cv2.Rodrigues(rot)
    return dst.T.tolist()[0]


def rod_to_quad(r):
    # q = [qx, qy, qz, qw]
    rotmat , jacobian = cv2.Rodrigues(npa(r))
    rotmat = np.append(rotmat, [[0,0,0]], 0)  
    rotmat = np.append(rotmat, [[0],[0],[0],[1]], 1)  
    q = tfm.quaternion_from_matrix(rotmat)
    return q.tolist()

class Program:
    def __init__(self, point3Ds, point2Ds, x0):
        self.point3Ds = point3Ds
        self.point2Ds = point2Ds
        self.x0 = x0

    def obj_func(self, cam):

        fx, fy = cam[0], cam[1]
        cx, cy = cam[2], cam[3]
        distCoeffs = cam[4], cam[5], cam[6], cam[7], cam[8]
        distCoeffs = (0.0, 0.0, 0.0, 0.0, 0.0)   ## hack no distortion
        tvec = cam[9:12]  # x,y,z
        rvec = cam[12:15]  # rodrigues

        # project
        #point2Ds_p = cv.project(point3Ds, cam)
        cameraMatrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        point2Ds_p, jacobian = cv2.projectPoints(npa(self.point3Ds, dtype=np.float), rvec, tvec, cameraMatrix, distCoeffs)
        #print point2Ds_p
        point2Ds_pp = [list(p[0]) for p in point2Ds_p]
        diff = npa(point2Ds_pp, dtype=np.float) - npa(self.point2Ds, dtype=np.float)
        #import pdb;
        #pdb.set_trace()
        print diff
        diff = diff.flatten(1)
        res = np.linalg.norm(diff)
        print res / 27.0
        return diff

    def run(self):
        
        res_1 = least_squares(self.obj_func, self.x0)
        
        print '--original--'
        trans =  self.x0[9:12]
        rod =  self.x0[12:15]
        q = rod_to_quad(rod)
        print 'pose', list(trans) + list(q)
        print 'fx,fy,cx,cy,distCoeff[5]', self.x0[0:9]
        
        
        print '\n--optimized--'
        trans = res_1.x[9:12]
        rod = res_1.x[12:15]
        q = rod_to_quad(rod)
        print 'pose', list(trans) + list(q)
        print 'fx,fy,cx,cy,distCoeff[5]', res_1.x[0:9]
        
    
        
        transform = tfm.concatenate_matrices(tfm.translation_matrix(trans), tfm.quaternion_matrix(q))
        inversed_transform = tfm.inverse_matrix(transform)
        translation = tfm.translation_from_matrix(inversed_transform)
        quaternion = tfm.quaternion_from_matrix(inversed_transform)
        pose =  translation.tolist() + quaternion.tolist()
        print 'webcam_T_robot:', " ".join('%.8e' % x for x in pose)
        print 'K: ', [res_1.x[0], 0.0, res_1.x[2], 0.0, res_1.x[1], res_1.x[3], 0.0, 0.0, 1.0]
        print 'P: ', [res_1.x[0], 0.0, res_1.x[2], 0.0, 0.0, res_1.x[1], res_1.x[3], 0.0, 0.0, 0.0, 1.0, 0.0]
        #print res_1

        data = dict()
        data['extrinsics'] = dict()
        data['extrinsics']['reference_link_name'] = 'base'
        transformDict = dict()
        transformDict['translation'] = dict()
        transformDict['translation']['x'] = translation.tolist()[0]
        transformDict['translation']['y'] = translation.tolist()[1]
        transformDict['translation']['z'] = translation.tolist()[2]

        transformDict['rotation'] = dict()
        transformDict['rotation']['x'] = quaternion.tolist()[0]
        transformDict['rotation']['y'] = quaternion.tolist()[1]
        transformDict['rotation']['z'] = quaternion.tolist()[2]
        transformDict['rotation']['w'] = quaternion.tolist()[3]

        data['extrinsics']['transform_to_reference_link'] = transformDict

        spartanSourceDir = spartanUtils.getSpartanSourceDir()
        extrinsicsFilename = os.path.join(spartanSourceDir, 'sandbox', 'extrinsics.yaml')
        spartanUtils.saveToYaml(data, extrinsicsFilename)

        intrinsics = dict()
        intrinsics['image_width'] = 640
        intrinsics['image_height'] = 480
        intrinsics['camera_name'] = 'xtion_rgb'
        camMatrix = dict()
        camMatrix['rows'] = 3
        camMatrix['cols'] = 3
        KList = res_1.x.tolist()
        camMatrix['data'] = [KList[0], 0.0, KList[2], 0.0, KList[1], KList[3], 0.0, 0.0, 1.0]

        intrinsics['camera_matrix'] = camMatrix

        distortion_coeff = dict()
        distortion_coeff['rows'] = 1
        distortion_coeff['cols'] = 5
        distortion_coeff['data'] = [0]*5

        intrinsics['distortion_model'] = 'plumb_bob'
        intrinsics['distortion_coefficients'] = distortion_coeff

        intrinsics['rectification_matrix'] = dict()
        intrinsics['rectification_matrix']['rows'] = 3
        intrinsics['rectification_matrix']['cols'] = 3
        intrinsics['rectification_matrix']['data'] = [1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000]

        intrinsics['projection_matrix'] = dict()
        intrinsics['projection_matrix']['rows'] = 3
        intrinsics['projection_matrix']['cols'] = 4
        intrinsics['projection_matrix']['data'] = [KList[0], 0.0, KList[2], 0.0, 0.0, KList[1], KList[3], 0.0, 0.0, 0.0, 1.0, 0.0]


        intrinsicsFilename = os.path.join(spartanSourceDir, 'sandbox', 'intrinsics.yaml')
        spartanUtils.saveToYaml(intrinsics, intrinsicsFilename)
        
        



        return res_1.x
        

if __name__ == '__main__' or __name__ == 'calib3d2d':
    color1 = (0,255,255)
    color2 = (0,0,255)
    color3 = (255,0,255)
#pose [0.7168, -0.07688, 0.3788, 0.499472915586398, 0.8532309406659719, 0.07190354748447386, -0.1317332469235899]
#fx,fy,cx,cy,distCoeff[5] [605.22376, 605.37555, 320.96071, 233.59959, 0.02671554, 0.6672619, -0.006263159, 0.0006014189, -2.923799]

    x0_ext_old = [1.7924193532299717, -0.1019227943559253, 0.7797246010193745] + [0.5828151822317369, 0.5618307868337208, -0.39456081733618475, -0.43473485223638464]
    
    transform = tfm.concatenate_matrices(tfm.translation_matrix(x0_ext_old[0:3]), tfm.quaternion_matrix(x0_ext_old[3:7]))
    inversed_transform = tfm.inverse_matrix(transform)
    translation = tfm.translation_from_matrix(inversed_transform)
    quaternion = tfm.quaternion_from_matrix(inversed_transform)

    x0_ext =  translation.tolist() + quat_to_rod(quaternion.tolist())

    x0_int = [605.22376, 605.37555, 320.96071, 233.59959, 2.671554e-02, 6.672619e-01, -6.263159e-03, 6.014189e-04,
              -2.923799e+00]
    x0 = x0_int + x0_ext
    
    with open(save_dir + 'data.extracted2d.json') as data_file:
        data = json.load(data_file)
        
    point3d = [d["cross3d"][0:3] for d in data]
    point2d = [d["cross2d"] for d in data]
    #print point3d
    p = Program(point3d, point2d, x0)
    cam = p.run()
    
    # show reprojection
    fx, fy = cam[0], cam[1]
    cx, cy = cam[2], cam[3]
    distCoeffs = cam[4], cam[5], cam[6], cam[7], cam[8]
    tvec = cam[9:12]  # x,y,z
    rvec = cam[12:15]  # rodrigues

    # project
    cameraMatrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

    point2Ds_p, jacobian = cv2.projectPoints(npa(point3d, dtype=np.float), rvec, tvec, cameraMatrix, distCoeffs)
    point2Ds_p_nd, jacobian = cv2.projectPoints(npa(point3d, dtype=np.float), rvec, tvec, cameraMatrix, (0.,0.,0.,0.,0.)) # no distortion considerations
    
    avg_eucl_error = 0

    for i, d in enumerate(data):
        image_viz = cv2.imread(save_dir + d['pic_path'])
        #image_unlabled = cv2.imread(save_dir + d['pic_path'])
        #cv2.imshow("image_unlabeled", image_unlabled)

        pt_int = tuple([int(round(p)) for p in d['cross2d']])
        cv2.line(image_viz, (pt_int[0]-2, pt_int[1]), (pt_int[0]+2, pt_int[1]), color1)
        cv2.line(image_viz, (pt_int[0], pt_int[1]-2), (pt_int[0], pt_int[1]+2), color1)
        
        pt_int = tuple([int(round(p)) for p in point2Ds_p_nd[i][0]])
        cv2.line(image_viz, (pt_int[0]-2, pt_int[1]), (pt_int[0]+2, pt_int[1]), color3)
        cv2.line(image_viz, (pt_int[0], pt_int[1]-2), (pt_int[0], pt_int[1]+2), color3)
        
        pt_int = tuple([int(round(p)) for p in point2Ds_p[i][0]])
        #cv2.line(image_viz, (pt_int[0]-2, pt_int[1]), (pt_int[0]+2, pt_int[1]), color2)
        #cv2.line(image_viz, (pt_int[0], pt_int[1]-2), (pt_int[0], pt_int[1]+2), color2)
        cv2.imshow("image", image_viz)

        
        avg_eucl_error += np.linalg.norm(point2Ds_p_nd[i][0]-d['cross2d'])
        
        
        while True:
            # display the image and wait for a keypress
            key = cv2.waitKey(3) & 0xFF
            if key == ord("n"):
                break
	print()
    #cv2.undistortPoints(npa(point3d, dtype=np.float), point2d, K, dist_coef)
