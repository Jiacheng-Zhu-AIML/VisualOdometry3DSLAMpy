import numpy as np
import cv2

from VisualOdometrySLAM import VOCamera, VisualOdometryFrame, VisualOdometryOperator


f = open('r_t_all.txt', 'w')

vo_cam = VOCamera(1411.5, 1423.11, 555.25, 251.34)

#orb_detector = cv2.ORB_create(fastThreshold=25)
orb_detector = cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)
vo_orb_operator = VisualOdometryOperator(vo_cam)

traj = np.zeros((1800,1200,3), dtype=np.uint8)

for img_id in range(0,3433):
    img = cv2.imread('../imagelist/'+ str(img_id) +'.png', 2)
    # print('img.shape =', img.shape)
    # print('img.dtype =', img.dtype)
    #
    # img = cv2.cvtColor(img,cv2.COLOR_BGR2HLS)
    # img = img[...,2]
    #
    # print('img.shape =', img.shape)
    # print('img.dtype =', img.dtype)

    cur_vo_frame = VisualOdometryFrame(img, img_id)
    #print('cur_vo_frame =', cur_vo_frame)

    cur_vo_frame.ActiveFrameFAST(cur_velocity=1, cv2detector=orb_detector)

    #print('cur_vo_frame.feature_2d =', cur_vo_frame.feature_2d)

    vo_orb_operator.ProcessNewFrame(cur_vo_frame, method='2d2d')

    cur_t = vo_orb_operator.cum_T_mat

    temp_R = cur_vo_frame.R_mat
    temp_T = cur_vo_frame.T_mat

    print('')
    f.write('\n')
    for one in temp_R:
        print(one)
        f.write(str(one))
        f.write('\n')

    for one in temp_T:
        print(one)
        f.write(str(one))
        f.write('\n')

    if(img_id > 2):
        x, y, z = cur_t[0], cur_t[1], cur_t[2]
    else:
        x, y, z = 0., 0., 0.
    draw_x, draw_y = int(x)+1000, -(int(z)) + 600


    cv2.circle(traj, (draw_x,draw_y), 1, (img_id*255/4540,255-img_id*255/4540,0), 1)

    #cv2.rectangle(traj, (10, 20), (600, 60), (0,0,0), -1)
    text = "Coordinates: x=%2fm y=%2fm z=%2fm"%(x,y,z)
    cv2.putText(traj, text, (20,40), cv2.FONT_HERSHEY_PLAIN, 1, (255,255,255), 1, 8)

    img_key_point = cv2.drawKeypoints(img, cur_vo_frame.feature_2d_object, img)
    cv2.imshow('Road facing camera', img_key_point)
    cv2.imshow('Trajectory', traj)
    #cv2.waitKey(1)
    #break

f.close()



#print(vo_orb_operator.vo_frame_list)

print('vo_orb_operator =', vo_orb_operator)


