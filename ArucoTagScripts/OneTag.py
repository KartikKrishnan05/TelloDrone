import cv2
import numpy as np
from djitellopy import Tello
import time

# ArUco marker detection setup
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()

# Initialize Tello drone
tello = Tello()
tello.connect()

# Start video stream
tello.streamon()

# Function to detect ArUco marker and get its position
def detect_aruco_marker(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect markers
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, _ = detector.detectMarkers(gray)
    
    if ids is not None:
        for i, marker_id in enumerate(ids):
            if marker_id == 0:  # We're looking for marker ID 0
                marker_corners = corners[i][0]
                center_x = int((marker_corners[0][0] + marker_corners[2][0]) / 2)
                center_y = int((marker_corners[0][1] + marker_corners[2][1]) / 2)
                
                # Calculate marker size (distance from the camera)
                marker_size = np.linalg.norm(marker_corners[0] - marker_corners[2])
                
                # Draw the detected marker on the frame for visualization
                cv2.aruco.drawDetectedMarkers(frame, corners)
                
                return (center_x, center_y, marker_size)
    
    return None



print(f"Battery: {tello.get_battery()}%")

# Takeoff
tello.takeoff()
tello.move_up(130)
print("Drone has taken off")

# Start looking for the marker
original_position = (0, 0)  # Assume the drone starts at 0,0 position
marker_found = False
close_to_marker = False

# Thresholds for when the drone is considered "close enough" to the marker
CLOSE_ENOUGH_MARKER_SIZE = 225  # Marker size threshold (adjust based on your setup)
FORWARD_STEP_SIZE = 20  # Move forward in smaller steps

try:
    while not marker_found:
        # Get the latest frame from the drone's camera
        frame = tello.get_frame_read().frame
        
        # Detect the ArUco marker
        marker_data = detect_aruco_marker(frame)

        # Display the video stream with OpenCV
        cv2.imshow("Tello Camera Feed", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break  # Press 'q' to stop the video stream manually

        if marker_data is not None:
            center_x, center_y, marker_size = marker_data
            print(f"Marker found at position: {center_x, center_y} with size {marker_size}")
            
            # Move towards the marker
            frame_center_x = frame.shape[1] // 2
            frame_center_y = frame.shape[0] // 2
            
            # Align horizontally
            if abs(center_x - frame_center_x) > 30:
                if center_x < frame_center_x:
                    tello.move_left(20)
                    print("Moving left")
                else:
                    tello.move_right(20)
                    print("Moving right")
            
            # Align vertically
            if abs(center_y - frame_center_y) > 30:
                if center_y < frame_center_y:
                    tello.move_up(20)
                    print("Moving up")
                else:
                    tello.move_down(20)
                    print("Moving down")
            
            # Move forward if not yet close enough to the marker
            if marker_size < CLOSE_ENOUGH_MARKER_SIZE:
                tello.move_forward(FORWARD_STEP_SIZE)
                print("Moving forward closer to the marker")
                time.sleep(1)  # Allow time to move
            else:
                print("Close enough to the marker")
                close_to_marker = True
            
            # If drone is close enough to the marker, stop moving forward
            if close_to_marker:
                print("Finish: Reached the ArUco marker with ID 0")
                marker_found = True
        else:
            print("Marker not found, rotating...")
            tello.rotate_clockwise(30)  # Keep searching by rotating

    # After reaching the marker, return to the original position
    print("Returning to original position...")
    tello.move_back(100)  # Adjust this value based on how much the drone moved forward

finally:
    # Land the drone
    tello.land()
    print("Drone has landed")

    # Turn off video stream and close the window
    tello.streamoff()
    cv2.destroyAllWindows()
