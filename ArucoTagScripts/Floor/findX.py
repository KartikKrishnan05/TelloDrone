import cv2
import numpy as np
from djitellopy import Tello
import time

# Initialize the Tello drone
tello = Tello()
tello.connect()

# Start the video stream
tello.streamon()

# Function to detect the 'X' on the floor
def find_x_marker(frame):
    # Convert the image to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Apply thresholding to isolate the 'X' (adjust thresholds based on your lighting conditions)
    _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
    
    # Find contours in the thresholded image
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # Loop over contours to find the 'X' based on its shape and size
    for contour in contours:
        # Get the bounding box for the contour
        x, y, w, h = cv2.boundingRect(contour)
        
        # Apply a condition to check if the contour resembles the size of the 'X' marker
        if 50 < w < 200 and 50 < h < 200:  # Adjust based on actual size
            # Draw the bounding box (for debugging)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            center_x = x + w // 2
            center_y = y + h // 2
            return center_x, center_y
    
    return None, None

# Function to centralize drone over the marker
def centralize_drone(center_x, center_y, frame):
    # Get frame center
    frame_center_x = frame.shape[1] // 2
    frame_center_y = frame.shape[0] // 2
    
    # Define a tolerance to avoid over-adjustment
    tolerance = 30
    
    # Horizontal adjustment
    if center_x is not None and abs(center_x - frame_center_x) > tolerance:
        if center_x < frame_center_x:
            tello.move_left(20)  # Move left if 'X' is to the left
        else:
            tello.move_right(20)  # Move right if 'X' is to the right
    
    # Vertical adjustment
    if center_y is not None and abs(center_y - frame_center_y) > tolerance:
        if center_y < frame_center_y:
            tello.move_up(20)  # Move up if 'X' is above
        else:
            tello.move_down(20)  # Move down if 'X' is below

# Main logic to take off and find the marker
def main():
    # Take off
    tello.takeoff()
    
    try:
        while True:
            # Get the video frame from the drone's camera
            frame = tello.get_frame_read().frame
            
            # Resize the frame for better performance
            frame = cv2.resize(frame, (640, 480))
            
            # Find the 'X' marker on the floor
            center_x, center_y = find_x_marker(frame)
            
            # Centralize the drone over the 'X'
            centralize_drone(center_x, center_y, frame)
            
            # Show the video stream (optional)
            cv2.imshow("Drone Camera", frame)
            
            # Exit if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            # Small delay to prevent spamming commands
            time.sleep(0.5)
    
    finally:
        # Land the drone
        tello.land()
        cv2.destroyAllWindows()

# Run the main function
if __name__ == '__main__':
    main()
