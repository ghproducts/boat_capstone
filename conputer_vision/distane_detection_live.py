import cv2
import numpy as np
import cv2.aruco as aruco

#with open('document.csv', 'w') as creating_new_csv_file:
#    pass

def load_coefficients(path):
    """ Loads camera matrix and distortion coefficients. """
    # FILE_STORAGE_READ
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

    # note we also have to specify the type to retrieve other wise we only get a
    # FileNode object back instead of a matrix
    camera_matrix = cv_file.getNode("K").mat()
    dist_matrix = cv_file.getNode("D").mat()

    cv_file.release()
    return [camera_matrix, dist_matrix]

# Load the camera calibrated coefficient.
[camera_matrix, dist_matrix] = load_coefficients('pixel6_coef_video')

# Read the video
#path_video = 'test_videos/20-100.mp4'
#video = cv2.VideoCapture(path_video) #for regular video input
video = cv2.VideoCapture(2) #for live input
fps = video.get(cv2. CAP_PROP_FPS)


# Lists to store the processed frames
position_result = []
rotation_result = []
XYZ = []
RPY = []
V_x = []
V_y = []
V_z = []


# Process the video
while (video.isOpened()):

    ret, frame = video.read()
    if ret == False:
        break
    #print(frame.shape)


    # Modify the frames by the calibrated coefficient
    h1, w1 = frame.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_matrix, (h1, w1), 0, (h1, w1))
    dst1 = cv2.undistort(frame, camera_matrix, dist_matrix, None, newcameramtx)
    # x, y, w1, h1 = roi
    # dst1 = dst1[y:y + h1, x:x + w1]
    frame=dst1
    video_width = frame.shape[1]   # float `width`
    video_height = frame.shape[0]  # float `height`
    size = (video_width, video_height)



    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    parameters =  aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,aruco_dict,parameters=parameters)

    if ids is not None:
        #Get rotation matrix rvec；and the  tvec：
        #set the marker size here:
        marker_size = 0.1016 #marker size in [m]
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_matrix)
        # 估计每个标记的姿态并返回值rvet和tvec ---不同
        #rvec为旋转矩阵，tvec为位移矩阵
        # from camera coeficcients
        (rvec-tvec).any() # get rid of that nasty numpy value array error

        #在画面上 标注auruco标签的各轴
        for i in range(rvec.shape[0]):
            result_img = cv2.drawFrameAxes(frame, camera_matrix, dist_matrix, rvec[i, :, :], tvec[i, :, :],0.03)
            aruco.drawDetectedMarkers(frame, corners,ids)


        ###### 显示id标记 #####
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(frame, "Id: " + str(ids), (0,200), font, 2, (0,0,255),5,cv2.LINE_AA)
        distance = ((tvec[0][0][2] + 0.02) * 0.0254) * 100 / 2.8  # Meters
        #distance = tvec[0][0][2]
        cv2.putText(frame, 'distance:' + str(round(distance, 4)) + str('m'), (0, 400), font, 2, (0, 0, 255), 5,
                    cv2.LINE_AA)

        ### find the rotation of the camera relative to marker

        R = cv2.Rodrigues(rvec)[0]
        R_T = R.T
        T = tvec[0].T

        xyz = np.dot(R_T, - T).squeeze()
        XYZ.append(xyz)

        rpy = np.deg2rad(cv2.RQDecomp3x3(R_T)[0])
        RPY.append(rpy)

        V_x.append(np.dot(R_T, np.array([1,0,0])))
        V_y.append(np.dot(R_T, np.array([0,1,0])))
        V_z.append(np.dot(R_T, np.array([0,0,1])))

        np.savetxt('data/XYZ.csv', XYZ, delimiter=',')
        np.savetxt('data/RPY.csv', RPY, delimiter=',')
        np.savetxt('data/V_x.csv', V_x, delimiter=',')
        np.savetxt('data/V_y.csv', V_y, delimiter=',')
        np.savetxt('data/V_z.csv', V_z, delimiter=',')



        #rotVec,_ = cv2.Rodrigues(rvec)
        #print(np.transpose(rotVec))
        position_result.append(tvec[0][0])
        #rotation.append(np.transposedst)
        np.savetxt('position.csv', position_result, delimiter=',')
        #np.savetxt('position.csv', position_result, delimiter=',')

        #with open('document.csv','a') as fd:
            #fd.write(str(distance))


    cv2.imshow("Image", frame)


    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

#np.savetxt('data.csv', list_result, delimiter=',')

cv2.destroyAllWindows()
cap.release()

# Save the video:
#out = cv2.VideoWriter('distance_measured.mp4', cv2.VideoWriter_fourcc(*'mp4v'), fps, size)
#for i in range(len(list_result)):
#    out.write(list_result[i])
#out.release()
#print("Output Saved to distance_measured.mp4")

