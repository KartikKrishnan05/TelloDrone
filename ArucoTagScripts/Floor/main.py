import cv2
import numpy as np
from djitellopy import Tello
import time

# ArUco marker detection setup
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()

# Initialize Tello drone
tello = Tello()
tello.connect()

# Start video stream
tello.streamon()

# Distance calculation formula: d = (W_real * f) / w_pixel
def calculate_distance(W_real, f, w_pixel):
    return ((W_real * f) / w_pixel) * 10  # Multiplied by 10 to convert to cm

# Function to detect ArUco marker and calculate its width, height, and distance
def detect_aruco_marker(frame, marker_id, W_real, f):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect markers
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, _ = detector.detectMarkers(gray)
    
    if ids is not None:
        for i, found_id in enumerate(ids):
            if found_id == marker_id:
                marker_corners = corners[i][0]
                
                # Calculate marker width and height
                marker_width = np.linalg.norm(marker_corners[0] - marker_corners[1])
                marker_height = np.linalg.norm(marker_corners[1] - marker_corners[2])
                
                # Calculate distance using width (you could also use height similarly)
                distance = calculate_distance(W_real, f, marker_width)
                
                # Draw the detected marker on the frame for visualization
                cv2.aruco.drawDetectedMarkers(frame, corners)
                
                # Return the position, marker dimensions, and calculated distance
                center_x = int((marker_corners[0][0] + marker_corners[2][0]) / 2)
                center_y = int((marker_corners[0][1] + marker_corners[2][1]) / 2)
                
                return center_x, center_y, marker_width, marker_height, distance
    
    return None

# Function to display battery percentage on the frame
def display_battery_on_frame(frame, battery_level):
    text = f"Battery: {battery_level}%"
    cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

# Function to search for and fly to markers
def search_and_fly_to_marker(marker_id, W_real, f):
    found_once = False  # To track if the marker was found
    counter = 0
    while True:
        frame = tello.get_frame_read().frame
        battery_level = tello.get_battery()  # Get the current battery level
        
        # Detect the ArUco marker and calculate its distance
        marker_data = detect_aruco_marker(frame, marker_id, W_real, f)
        
        # Display battery percentage on the frame
        display_battery_on_frame(frame, battery_level)
        
        # Display the video stream with OpenCV
        if frame is not None:
            cv2.imshow("Tello Camera Feed", frame)
        
        if marker_data:
            center_x, center_y, marker_width, marker_height, distance = marker_data
            print(f"Marker {marker_id} found at position: ({center_x}, {center_y}), "
                  f"width: {marker_width}, height: {marker_height}, distance: {distance:.2f} cm")
            
            frame_center_x = frame.shape[1] // 2  # Center of the frame

            

            # Adjust orientation to align horizontally
            if not found_once:
                
                if abs(center_x - frame_center_x) > 30: 
                    print(f"                 counter: {counter}")
                    # Allow for a small tolerance
                    if center_x < frame_center_x:
                        
                        if counter >= 4:
                            print("plan B1")
                            tello.rotate_counter_clockwise(40)
                            tello.rotate_clockwise(35)
                            found_once = True
                            counter = 0v

                        tello.rotate_counter_clockwise(10)
                        counter = counter + 1
                        print("Rotating left to center marker")
                        
                    else:
                        
                        if counter >= 4:
                            print("plan B2")
                            tello.rotate_clockwise(40)
                            tello.rotate_counter_clockwise(35)
                            found_once = True
                            counter = 0
                        
                        tello.rotate_clockwise(10)
                        counter = counter + 1
                        print("Rotating right to center marker")
                else:
                    print("Marker centered horizontally.")
                    found_once = True  # Set the flag so this adjustment is done only once
            
            # Move towards the marker if the orientation is adjusted
            if found_once:
                if distance > 20:  # Only move if the distance is significant (greater than 20 cm)
                    tello.move_forward(int(distance))
                    print(f"Moving forward by {int(distance)} cm towards marker {marker_id}")
                else:
                    print(f"Close enough to marker {marker_id}, stopping movement.")
                
                # Perform a backflip after reaching the marker
                

                break  # Break out of the loop once the movement is complete
        
        else:
            print(f"Marker {marker_id} not found, rotating...")
            tello.rotate_clockwise(10)
        
        # Ensure that the frame stays on screen
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break  # Press 'q' to stop the video stream manually

# Main function to fly through all markers till the last one
def fly_through_markers(last_marker_id, W_real, f):
    for marker_id in range(last_marker_id + 1):  # Loop through marker IDs starting from 0
        search_and_fly_to_marker(marker_id, W_real, f)

# Takeoff and immediately move closer to the floor
print(f"Battery: {tello.get_battery()}%")
tello.takeoff()
tello.move_down(20)
print("Drone has taken off and moved down closer to the floor")

try:
    # Set the real width of the ArUco tag and the focal length
    W_real = 20  # Real width of the ArUco tag in cm
    f = 77.4     # Estimated focal length in pixels (based on calibration)
    
    # Set the last marker ID (e.g., if the last marker is ID 4)
    last_marker_id = 2

    # Fly through all markers up to the last marker ID
    fly_through_markers(last_marker_id, W_real, f)

finally:
    # Land the drone
    tello.rotate_clockwise(360)
    tello.land()
    print("Drone has landed")

    # Turn off video stream and close the window
    tello.streamoff()
    cv2.destroyAllWindows()