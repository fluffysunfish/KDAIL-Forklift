import cv2 as cv
import cv2.aruco as aruco

cap = cv.VideoCapture(0)

aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_100)
parameters = aruco.DetectorParameters_create()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)

        for i, corner in enumerate(corners):
            corners_int = corner.reshape((4, 2)).astype(int)
            
            center_x = int(corners_int[:, 0].mean())
            center_y = int(corners_int[:, 1].mean())

            marker_id = ids[i][0]
            cv.putText(frame, str(marker_id), (center_x - 10, center_y + 10),
                       cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv.imshow("ArUco Marker Detection", frame)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
