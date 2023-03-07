import cv2
import numpy as np
import matplotlib.pyplot as plt
import glob

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

plt.close()
def calibrate(dirpath, video_format, square_size, width=10, height=11):
    # Calibrate the camera from the videos

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Load the videos
    if dirpath[-1:] == '/':
        dirpath = dirpath[:-1]
    paths_videos = glob.glob(dirpath + '/' + '*.' + video_format)
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,6,0)
    objp = np.zeros((height*width, 3), np.float32)
    objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)

    objp = objp * square_size  #square size is in meters

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    print("imported all videos successfully")


    for path_video in paths_videos:

        # Read the video
        video = cv2.VideoCapture(path_video)
        num_frame = 0
        fps = video.get(cv2. CAP_PROP_FPS)

        print("check video", path_video)

        while (video.isOpened()):

            ret, frame = video.read()
            if ret == False:
                break
            num_frame +=1

            if num_frame % 10 != 0:
                continue

            # Find the chess board corners
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, (width, height), None)

            print(num_frame/30, 's', ret)
            # If found, add object points, image points (after refining them)
            if ret:
                print("add a picture {}".format(num_frame))
                objpoints.append(objp)

                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)

                # Draw and display the corners
                img = cv2.drawChessboardCorners(frame, (width, height), corners2, ret)
                #if num_frame % 100 == 0:
                    #cv2_imshow(img)
    print("imported all frames")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    print("Calibration successful")
    return [ret, mtx, dist, rvecs, tvecs]


def save_coefficients(mtx, dist, path):
    """ Save the camera matrix and the distortion coefficients to given path/file. """
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    cv_file.write("K", mtx)
    cv_file.write("D", dist)
    # note you *release* you don't close() a FileStorage object
    cv_file.release()

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
co


dirpath = 'calibrate_video'
video_format = 'mp4'
prefix = 'PXL'
square_size = 0.016
#a = glob.glob(dirpath + '/' + prefix + '*.' + video_format)
ret, mtx, dist, rvecs, tvecs = calibrate(dirpath, video_format, square_size, width=10, height=11)
save_coefficients(mtx, dist, 'pixel6_coef_video')
print("Calibration is finished. RMS: ", ret)

# Load the camera calibrated coefficient.
# [camera_matrix, dist_matrix] = load_coefficients('pixel6_coef_video')