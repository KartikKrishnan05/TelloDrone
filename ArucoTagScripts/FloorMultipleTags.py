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

def search_and_fly_to_marker(marker_id):
    marker_found = False
    close_to_marker = False
    CLOSE_ENOUGH_MARKER_SIZE = 200  # Adjust marker size threshold based on your setup
    FORWARD_STEP_SIZE = 20  # Move forward in smaller steps

    while not marker_found:
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
                marker_found = True
        else:
            print(f"Marker {marker_id} not found, rotating...")
            tello.rotate_clockwise(30)  # Keep searching by rotating

# Takeoff and immediately move closer to the floor
print(f"Battery: {tello.get_battery()}%")
tello.takeoff()
tello.move_down(20)  # Lower the drone's height to improve detection of floor markers
print("Drone has taken off and moved down closer to the floor")

try:
    # First, search for marker with ID 0
    search_and_fly_to_marker(0)

    # After reaching marker 0, immediately search for marker with ID 1
    search_and_fly_to_marker(1)

    # Continue with more markers if necessary (e.g., marker ID 2, 3, etc.)
    search_and_fly_to_marker(2)

    # Continue with more markers if necessary (e.g., marker ID 2, 3, etc.)
    search_and_fly_to_marker(3)

finally:
    # Land the drone
    tello.land()
    print("Drone has landed")

    # Turn off video stream and close the window
    tello.streamoff()
    cv2.destroyAllWindows()
