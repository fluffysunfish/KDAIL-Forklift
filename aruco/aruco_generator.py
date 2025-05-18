import cv2 as cv
import cv2.aruco as aruco
import numpy as np

aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_100)
marker_id = 1
marker_size = 300

marker_image = aruco.drawMarker(aruco_dict, marker_id, marker_size)


border_size = 10
bordered_image = cv.copyMakeBorder(
    marker_image,
    top=border_size,
    bottom=border_size,
    left=border_size,
    right=border_size,
    borderType=cv.BORDER_CONSTANT,
    value=255
)

cv.imwrite(f"aruco_marker_{marker_id}.png", bordered_image)
cv.imshow(f"ArUco Marker {marker_id}", bordered_image)
cv.waitKey(0)
cv.destroyAllWindows()
