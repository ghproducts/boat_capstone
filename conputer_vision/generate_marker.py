import cv2
import numpy as np
import cv2.aruco as aruco
import matplotlib.pyplot as plt

cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

tagsize = 1000 #size of marker

tag1 = np.zeros((tagsize, tagsize, 1), dtype="uint8")

dictionary = aruco.Dictionary_get(aruco.DICT_5X5_250)
aruco.drawMarker(dictionary, 248, tagsize, tag1, 1)

fig = plt.figure()
plt.imshow(tag1, cmap = plt.cm.gray)
plt.axis('off')
plt.savefig('aruco_marker.png', dpi=1200)