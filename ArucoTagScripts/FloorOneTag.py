import cv2
import numpy as np
from djitellopy import Tello
import time

# ArUco marker detection setup
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)  # Larger dictionary for more robust detection
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
            if found_id == marker_id:  # We're looking for the specific marker ID
                marker_corners = corners[i][0]
                center_x = int((marker_corners[0][0] + marker_corners[2][0]) / 2)
                center_y = int((marker_corners[0][1] + marker_corners[2][1]) / 2)
                
                # Calculate marker size (distance from the camera)
                marker_size = np.linalg.norm(marker_corners[0] - marker_corners[2])
                
                # Draw the detected marker on the frame for visualization
                cv2.aruco.drawDetectedMarkers(frame, corners)
                
                return (center_x, center_y, marker_size)
    
    return None

# Function to search for and fly to markers in sequence
def search_and_fly_to_marker(marker_id, last_marker_id):
    marker_found_once = False  # Initialize the boolean variable
    close_to_marker = False
    CLOSE_ENOUGH_MARKER_SIZE = 125  # Adjust marker size threshold based on your setup
    FORWARD_STEP_SIZE = 20  # Move forward in smaller steps

    # Initialize detection count to track if the marker is lost
    consecutive_not_found = 0

    while True:
        # Get the latest frame from the drone's camera
        frame = tello.get_frame_read().frame
        
        # Detect the ArUco marker
        marker_data = detect_aruco_marker(frame, marker_id=marker_id)

        # Display the video stream with OpenCV
        cv2.imshow("Tello Camera Feed", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break  # Press 'q' to stop the video stream manually

        if marker_data is not None:
            center_x, center_y, marker_size = marker_data
            print(f"Marker {marker_id} found at position: {center_x}, {center_y} with size {marker_size}")
            
            # Reset the consecutive "not found" count since the marker is detected
            consecutive_not_found = 0
            marker_found_once = True  # Set the boolean to True once the marker is found
            
            # Move towards the marker
            frame_center_x = frame.shape[1] // 2
            frame_center_y = frame.shape[0] // 2
            
            # Rotate to align with the marker horizontally
            if abs(center_x - frame_center_x) > 30:
                if center_x < frame_center_x:
                    tello.rotate_counter_clockwise(20)
                    print("Rotating left")
                else:
                    tello.rotate_clockwise(20)
                    print("Rotating right")
            
            # Move forward if not yet close enough to the marker
            if marker_size < CLOSE_ENOUGH_MARKER_SIZE:
                tello.move_forward(FORWARD_STEP_SIZE)
                print("Moving forward closer to the marker")
                time.sleep(1)  # Allow time to move
            else:
                print(f"Close enough to marker {marker_id}")
                close_to_marker = True
            
            # If drone is close enough to the marker, stop moving forward
            if close_to_marker:
                print(f"Finish: Reached the ArUco marker with ID {marker_id}")
                break  # Break out of the loop after reaching the marker

        else:
            # Marker not found
            consecutive_not_found += 1
            print(f"Marker {marker_id} not found, rotating... (Attempt {consecutive_not_found})")

            if marker_found_once and consecutive_not_found > 0:  # Marker was found but now lost
                print(f"Marker {marker_id} lost after detection, moving to the next marker.")
                break  # Move on to the next marker

            tello.rotate_clockwise(30)  # Keep searching by rotating

    # Reset the boolean variable for the next marker
    marker_found_once = False

# Main function to fly through all markers till the last one
def fly_through_markers(last_marker_id):
    for marker_id in range(last_marker_id + 1):  # Loop through marker IDs starting from 0
        search_and_fly_to_marker(marker_id, last_marker_id)

# Takeoff and immediately move closer to the floor
print(f"Battery: {tello.get_battery()}%")
tello.takeoff()
tello.move_down(20)  # Lower the drone's height to improve detection of floor markers
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
