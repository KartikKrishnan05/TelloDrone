import cv2
import numpy as np
from djitellopy import Tello

# ArUco marker detection setup
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()

# Initialize Tello drone
tello = Tello()
tello.connect()

# Start video stream
tello.streamon()

# Function to detect ArUco marker and print its size
def detect_aruco_marker(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect markers
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, _ = detector.detectMarkers(gray)
    
    if ids is not None:
        for i in range(len(ids)):
            marker_corners = corners[i][0]
            marker_width = np.linalg.norm(marker_corners[0] - marker_corners[1])
            marker_height = np.linalg.norm(marker_corners[1] - marker_corners[2])
            print(f"Marker {ids[i]} width: {marker_width}, height: {marker_height}")
    
    return None

try:
    # Take off and move down immediately
    tello.takeoff()
    tello.move_down(20)
    print("Drone has taken off and moved down to 20 units")

    # Main loop to detect markers
    while True:
        frame = tello.get_frame_read().frame
        detect_aruco_marker(frame)
        
        # Display the camera feed
        cv2.imshow("Tello Camera Feed", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to exit
            break

finally:
    # Land the drone and cleanup
    tello.land()
    tello.streamoff()
    cv2.destroyAllWindows()