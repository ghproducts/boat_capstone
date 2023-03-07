import cv2
import numpy as np
import cv2.aruco as aruco
import matplotlib.pyplot as plt

tagsize = 1000 #size of marker
dictionary = aruco.Dictionary_get(aruco.DICT_5X5_250)

tag1 = np.zeros((tagsize, tagsize, 1), dtype="uint8")
tag2 = np.zeros((tagsize, tagsize, 1), dtype="uint8")
tag3 = np.zeros((tagsize, tagsize, 1), dtype="uint8")
tag4 = np.zeros((tagsize, tagsize, 1), dtype="uint8")
dictionary = aruco.Dictionary_get(aruco.DICT_5X5_250)
aruco.drawMarker(dictionary, 248, tagsize, tag1, 1)
aruco.drawMarker(dictionary, 200, tagsize, tag2, 1)
aruco.drawMarker(dictionary, 150, tagsize, tag3, 1)
aruco.drawMarker(dictionary, 100, tagsize, tag4, 1)

# combine the images to
plt.subplot(5, 5, 1)
plt.axis('off')
plt.imshow(tag1, cmap = plt.cm.gray)
plt.subplot(5, 5, 5)
plt.axis('off')
plt.imshow(tag2, cmap = plt.cm.gray)
plt.subplot(5, 5, 21)
plt.axis('off')
plt.imshow(tag3, cmap = plt.cm.gray)
plt.subplot(5, 5, 25)
plt.axis('off')
plt.imshow(tag4, cmap = plt.cm.gray)
plt.savefig('landing_pad.png', dpi=1200)