import cv2
import cv2.aruco as aruco
import numpy as np

frame = cv2.imread("aruco_marker_1.png") 

if frame is None:
    print("Image not loaded.")
    exit()

gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_100)
parameters = aruco.DetectorParameters_create()

print("Shape:", gray.shape)

(corners, ids, _) = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
print(ids.flatten())

cv2.imshow("Gray Image", gray)
cv2.waitKey(0)
cv2.destroyAllWindows()

