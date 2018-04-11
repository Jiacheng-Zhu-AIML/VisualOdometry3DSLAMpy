'''

Visual Odometry SLAM


'''
import numpy as np
import cv2


class VOCamera:
    def __init__(self, fx, fy, u, v):
        self.fx = fx
        self.fy = fy
        self.u = u
        self.v = v
        self.k_mat = np.array([[fx, 0, u], [0, fy, v], [0, 0, 1]])


class VisualOdometryFrame:
    def __init__(self, image, image_id):
        self.image = image
        self.image_id = image_id
        self.cur_velocity = 0

        self.feature_2d = None
        self.descriptor = None
        self.feature_2d_object = None
        self.feature_2d_marker_list = None
        self.feature_2d_effective = None

        self.feature_3d = None
        self.feature_3d_marker_list = None
        self.feature_3d_effective = None

        self.E_mat = None
        self.mask = None
        self.R_mat = None
        self.T_mat = None

        self.last_frame = None
        self.detector = None


    def ActiveFrame(self, cur_velocity, cv2detector):
        '''
        1. Upload the 2d feature point and
        feature descriptor of this frame
        2. Add the current velocity of this frame
        :param cur_velocity:
        :param cv2detector:
        :return:
        '''
        self.cur_velocity = cur_velocity
        self.detector = cv2detector
        '''
        Detect the key points
        '''
        key_point_obj = cv2detector.detect(self.image, None)
        feature_2d_temp, self.descriptor = cv2detector.compute(self.image, key_point_obj)
        self.feature_2d = np.array([x.pt for x in feature_2d_temp], dtype=np.float32)

    def ActiveFrameFAST(self, cur_velocity, cv2detector):
        '''
        1. Upload the 2d feature point and
        feature descriptor of this frame
        2. Add the current velocity of this frame
        :param cur_velocity:
        :param cv2detector:
        :return:
        '''
        self.cur_velocity = cur_velocity
        self.detector = cv2detector
        '''
        Detect the key points
        '''
        detector = cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)
        #key_point_obj = cv2detector.detect(self.image, None)
        #feature_2d_temp, self.descriptor = cv2detector.compute(self.image, key_point_obj)
        feature_2d_temp = detector.detect(self.image)
        self.feature_2d_object = feature_2d_temp
        self.feature_2d = np.array([x.pt for x in feature_2d_temp], dtype=np.float32)

class VisualOdometryOperator:
    def __init__(self, vo_camera):
        self.camera_obj = vo_camera
        self.vo_frame_list = []

        self.cum_R_mat = None
        self.cum_T_mat = None

        self.last_vo_frame = None
        self.cur_vo_frame = None


        self.feature_3d_relative = None

        self.cur_vehicle_global = [0, 0, 0]
        #SLAM WARNING
        self.feature_3d_global = None

    def ProcessNewFrame(self, vo_frame_obj, method='2d2d'):
        '''
        :param vo_frame_obj:
        :return:
        '''
        if not self.vo_frame_list:
            #This is the first frame
            '''
            Just append first frame
            '''

            vo_frame_obj.E_mat = np.eye(3)
            vo_frame_obj.R_mat = np.eye(3)
            vo_frame_obj.T_mat = np.zeros((3, 1))
            self.vo_frame_list.append(vo_frame_obj)

            '''
            cum_R_mat,cum_T_mat, the R, T matrix according to global coordinate
            '''
            self.cum_R_mat = np.eye(3)
            self.cum_T_mat = np.zeros((3, 1))


        elif len(self.vo_frame_list) >= 1:
            '''
            Connect the second frame and
            Find E_mat, Find R, T
            
            The first 3D features should be generated at here
            '''
            self.last_vo_frame = self.vo_frame_list[-1]
            self.cur_vo_frame = vo_frame_obj
            vo_frame_obj.last_frame = self.vo_frame_list[-1]

            last_frame = self.vo_frame_list[-1]
            cur_frame = vo_frame_obj

            #switch method
            if method == '2d2d':

                feature2d_temp = cur_frame.feature_2d

                e_mat, mask = FindEssentialMatrixOpt(last_frame, cur_frame, self.camera_obj)
                vo_frame_obj.E_mat = e_mat

                r_mat, t_mat = FindRTOpt(e_mat, last_frame, cur_frame, self.camera_obj)
                vo_frame_obj.R_mat = r_mat
                vo_frame_obj.T_mat = t_mat

                scale_cor = cur_frame.cur_velocity
                scale_cor = 1

                #for one in r_mat:
                #    print(one)

                #for one in t_mat:
                #    print(one)

                self.cum_T_mat = self.cum_T_mat + scale_cor*self.cum_R_mat.dot(t_mat)
                self.cum_R_mat = r_mat.dot(self.cum_R_mat)

                if cur_frame.feature_2d.shape[0] < 1500:
                    cur_frame.feature_2d = feature2d_temp


            self.vo_frame_list.append(vo_frame_obj)







'''
Non object functions
Alternative methods for finding E, R, T
or uploading 3d points
'''
opt_flow_para = lk_params = dict(winSize  = (21, 21),
				 #maxLevel = 3,
				 criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

def FindEssentialMatrixOpt(last_frame, cur_frame, camera):
    last_image = last_frame.image
    cur_image = cur_frame.image
    last_feature = last_frame.feature_2d
    kp2, st, err = cv2.calcOpticalFlowPyrLK(last_image, cur_image, last_feature,
                                            None, **opt_flow_para)
    st = st.reshape(st.shape[0])
    kp1 = last_feature[st == 1]
    kp2 = kp2[st == 1]


    last_frame.feature_2d = kp1
    cur_frame.feature_2d = kp2

    E, mask = cv2.findEssentialMat(kp2, kp1, focal=camera.fx, pp=(camera.u, camera.v), method=cv2.RANSAC,
                                   prob=0.999, threshold=1.0)
    return E, mask


def FindRTOpt(e_mat, last_frame, cur_frame, camera):
    last_feature = last_frame.feature_2d
    cur_feature = cur_frame.feature_2d

    _, R, t, mask = cv2.recoverPose(e_mat, cur_feature, last_feature,
                                    focal=camera.fx, pp=(camera.u, camera.v))
    return R, t

















