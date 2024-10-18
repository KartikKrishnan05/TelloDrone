import cv2
import numpy as np
from djitellopy import Tello
import time

# ArUco marker detection setup
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()

# Enable corner refinement for better detection at angles
parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

# Initialize Tello drone
tello = Tello()
tello.connect()
 
# Start video stream
tello.streamon()

# Function to detect ArUco marker and get its position
def detect_aruco_marker(frame, marker_id):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect markers
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, _ = detector.detectMarkers(gray)
    
    if ids is not None:
        for i, found_id in enumerate(ids):
            if found_id == marker_id:
                marker_corners = corners[i][0]
                center_x = int((marker_corners[0][0] + marker_corners[2][0]) / 2)
                center_y = int((marker_corners[0][1] + marker_corners[2][1]) / 2)
                
                # Calculate marker size (distance from the camera)
                marker_size = np.linalg.norm(marker_corners[0] - marker_corners[2])
                
                # Draw the detected marker on the frame for visualization
                cv2.aruco.drawDetectedMarkers(frame, corners)
                
                return (center_x, center_y, marker_size)
    
    return None

# Function to search for and fly to markers
def search_and_fly_to_marker(marker_id, last_marker_id):
    found_once = False  # Boolean flag to indicate marker was found at least once
    CLOSE_ENOUGH_MARKER_SIZE = 150  # Threshold for being "close enough"
    
    while True:
        frame = tello.get_frame_read().frame
        
        # Detect the ArUco marker
        marker_data = detect_aruco_marker(frame, marker_id)
        
        # Display the video stream with OpenCV
        if frame is not None:
            cv2.imshow("Tello Camera Feed", frame)
        
        if marker_data:
            center_x, center_y, marker_size = marker_data
            print(f"Marker {marker_id} found at position: {center_x}, {center_y}, size {marker_size}")
            
            if not found_once:  # Adjust orientation only once
                frame_center_x = frame.shape[1] // 2
                
                # Rotate once to align horizontally
                if abs(center_x - frame_center_x) > 30:
                    if center_x < frame_center_x:
                        tello.rotate_counter_clockwise(20)
                        print("Rotating left")
                    else:
                        tello.rotate_clockwise(20)
                        print("Rotating right")
                found_once = True  # Set the flag so this adjustment is done only once
            
            # Move forward based on marker distance
            if marker_size < CLOSE_ENOUGH_MARKER_SIZE:
                distance_to_marker = (CLOSE_ENOUGH_MARKER_SIZE - marker_size) * 2  # Calculate distance to the marker
                tello.move_forward(int(distance_to_marker))
                print(f"Moving forward by {distance_to_marker} units toward marker {marker_id}")
                break  # Once forward movement is done, break the loop to go to the next marker
            else:
                print(f"Close enough to marker {marker_id}, moving to next marker.")
                break
        
        else:
            print(f"Marker {marker_id} not found, rotating...")
            tello.rotate_clockwise(15)
        
        # Ensure that the frame stays on screen
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break  # Press 'q' to stop the video stream manually

# Main function to fly through all markers till the last one
def fly_through_markers(last_marker_id):
    for marker_id in range(last_marker_id + 1):  # Loop through marker IDs starting from 0
        search_and_fly_to_marker(marker_id, last_marker_id)

# Takeoff and immediately move closer to the floor
print(f"Battery: {tello.get_battery()}%")
tello.takeoff()
tello.move_down(20)
print("Drone has taken off and moved down closer to the floor")

try:
    # Set the last marker ID (e.g., if the last marker is ID 4)
    last_marker_id = 3

    # Fly through all markers up to the last marker ID
    fly_through_markers(last_marker_id)

finally:
    # Land the drone
    tello.land()
    print("Drone has landed")

    # Turn off video stream and close the window
    tello.streamoff()
    cv2.destroyAllWindows()
