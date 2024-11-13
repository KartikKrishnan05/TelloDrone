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

# Initialize flight log
flight_log = []  # To log movements for reverse flight

# Distance calculation formula: d = (real_width * focal_length) / pixel_width
def calculate_distance(W_real, f, w_pixel):
    return ((W_real * f) / w_pixel) * 10 

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
                
                # Calculate distance using width 
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
                            print("plan B")
                            tello.rotate_counter_clockwise(40)
                            tello.rotate_clockwise(35)
                            found_once = True
                            counter = 0

                        tello.rotate_counter_clockwise(10)
                        flight_log.append(('rotate_ccw', 10))  # Log the rotation
                        counter = counter + 1
                        print("Rotating left to center marker")
                        
                    else:
                        
                        if counter >= 4:
                            print("plan B")
                            tello.rotate_clockwise(40)
                            tello.rotate_counter_clockwise(35)
                            found_once = True
                            counter = 0
                        
                        tello.rotate_clockwise(10)
                        flight_log.append(('rotate_cw', 10))  # Log the rotation
                        counter = counter + 1
                        print("Rotating right to center marker")
                else:
                    print("Marker centered horizontally.")
                    found_once = True  # Set the flag so this adjustment is done only once
            
            # Move towards the marker if the orientation is adjusted
            if found_once:
                if distance > 20:  # Only move if the distance is significant (greater than 20 cm)
                    tello.move_forward(int(distance))
                    flight_log.append(('move_forward', int(distance)))  # Log the forward movement
                    print(f"Moving forward by {int(distance)} cm towards marker {marker_id}")
                else:
                    print(f"Close enough to marker {marker_id}, stopping movement.")
                break  # Break the loop after reaching the marker
        else:
            print(f"Marker {marker_id} not found, rotating...")
            tello.rotate_clockwise(10)
            flight_log.append(('rotate_cw', 10))  # Log the rotation
        
        # Ensure that the frame stays on screen
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break  # Press 'q' to stop the video stream manually

# Function to combine consecutive turns
def combine_consecutive_turns(flight_log):
    optimized_log = []
    current_rotation = None
    accumulated_rotation = 0

    for command, value in flight_log:
        if command == 'rotate_cw' or command == 'rotate_ccw':
            if current_rotation is None:
                # Start tracking a new rotation
                current_rotation = command
                accumulated_rotation = value
            elif current_rotation == command:
                # Add up consecutive turns in the same direction
                accumulated_rotation += value
            else:
                # Log the previous rotation and start a new one
                optimized_log.append((current_rotation, accumulated_rotation))
                current_rotation = command
                accumulated_rotation = value
        else:
            # If the command is not a rotation, finalize the current rotation
            if current_rotation is not None:
                optimized_log.append((current_rotation, accumulated_rotation))
                current_rotation = None
                accumulated_rotation = 0
            # Log the non-rotation command
            optimized_log.append((command, value))

    # After the loop, make sure to log any remaining rotation
    if current_rotation is not None:
        optimized_log.append((current_rotation, accumulated_rotation))

    return optimized_log


def iter_drop_n(data):
    result = []
    first = True

    for command, value in data: 
        if command == 'move_forward' and first:
            first = False
        elif not first:
            result.append((command, value)) 

    return result




# Function to fly back along the logged route
def fly_back():
    print("Turning around and flying back along the recorded route...")
    optimized_log = combine_consecutive_turns(flight_log)  # Optimize consecutive turns
    print(optimized_log)
    new_optimized_log = iter_drop_n(optimized_log)
    new_optimized_log = new_optimized_log[1:]
    tello.rotate_clockwise(180)  # Rotate 180° to face the starting direction
    
    for command, value in reversed(new_optimized_log):
        if command == 'move_forward':
            tello.move_forward(value)  # Fly forward by the logged distance (since we rotated)
            print(f"Flying forward by {value} cm")
        elif command == 'rotate_cw':
            tello.rotate_counter_clockwise(value)  # Reverse clockwise rotation
            print(f"Rotating counterclockwise by {value} degrees")
        elif command == 'rotate_ccw':
            tello.rotate_clockwise(value)  # Reverse counterclockwise rotation
            print(f"Rotating clockwise by {value} degrees")

# Main function to fly through all markers till the last one
def fly_through_markers(first_marker, last_marker_id, W_real, f):
    for marker_id in range(first_marker, last_marker_id + 1):  # Loop through marker IDs starting from 0
        search_and_fly_to_marker(marker_id, W_real, f)
    fly_back()  # Fly back after reaching the last marker

# Takeoff and immediately move closer to the floor
print(f"Battery: {tello.get_battery()}%")
tello.takeoff()
time.sleep(500 / 1000)
tello.move_down(20)
print("Drone has taken off and moved down closer to the floor")

try:
    # Set the real width of the ArUco tag and the focal length
    W_real = 20  # Real width of the ArUco tag in cm
    f = 77.4     # Estimated focal length in pixels (based on calibration)
    
    # Set the last marker ID (e.g., if the last marker is ID 4)
    last_marker_id = 2

    first_maker_id = 0

    # Fly through all markers up to the last marker ID
    fly_through_markers(first_maker_id, last_marker_id, W_real, f)

finally:
    # Land the drone

    fly_through_markers(3, 3, 20, 77.4)

    tello.land()
    print("Drone has landed")

    # Turn off video stream and close the window
    tello.streamoff()
    cv2.destroyAllWindows()

