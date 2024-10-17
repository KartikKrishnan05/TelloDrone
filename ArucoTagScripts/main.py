from djitellopy import Tello
import cv2
import numpy as np
import time

# Constants
SPEED = 20  # Initial speed
SLOW_SPEED = 5  # Reduced speed when approaching a tag

# Initialize Tello
tello = Tello()
tello.connect()

print(f"Battery: {tello.get_battery()}%")
print("Takeoff...")
tello.takeoff()

# Fly up to double the height (if needed, uncomment the line below)
# tello.move_up(50)

# Start video stream
tello.streamoff()
tello.streamon()

# Setup ArUco dictionary and detector
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()

# ArUco tag sequence (IDs in the order of movement)
aruco_ids = [0, 1, 2, 3]

def detect_tag(tag_id, frame):
    """
    Detects the ArUco marker. Returns True if the target tag is found.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect markers
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None and tag_id in ids:
        print(f"Tag {tag_id} found!")
        return True
    
    return False

def move_in_direction(direction, slow=False):
    """
    Move continuously in the given direction with optional slow speed until the next tag is detected.
    direction: 'left', 'right', 'up', 'down'
    slow: If True, move at a slower speed
    """
    move_speed = SLOW_SPEED if slow else SPEED
    if direction == 'left':
        tello.send_rc_control(-move_speed, 0, 0, 0)  # Move left continuously
    elif direction == 'right':
        tello.send_rc_control(move_speed, 0, 0, 0)  # Move right continuously
    elif direction == 'up':
        tello.send_rc_control(0, 0, move_speed, 0)  # Move up continuously
    elif direction == 'down':
        tello.send_rc_control(0, 0, -move_speed, 0)  # Move down continuously

def search_for_next_tag(tag_id, direction):
    """
    Move in the specified direction until the drone finds the next ArUco tag.
    direction: 'left', 'right', 'up', 'down'
    """
    tag_found = False

    while not tag_found:
        frame = tello.get_frame_read().frame
        frame = cv2.resize(frame, (640, 480))

        # Move in the given direction while scanning
        move_in_direction(direction)

        # Try to detect the tag
        tag_found = detect_tag(tag_id, frame)

        # Display the frame with the detected markers
        cv2.imshow('Tello ArUco Detection', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Stop the drone after finding the tag
    tello.send_rc_control(0, 0, 0, 0)
    return tag_found

def execute_movement_sequence():
    """
    Defines the drone's movements after finding each tag.
    """
    for idx, tag_id in enumerate(aruco_ids):
        print(f"Searching for tag {tag_id}...")

        if tag_id == 0:
            # Search for Tag 0 (starting position)
            search_for_next_tag(tag_id, 'up')
        elif tag_id == 1:
            # After Tag 0: Move back, then right to find Tag 1
            print("Moving back and right to find Tag 1")
            search_for_next_tag(tag_id, 'right')
        elif tag_id == 2:
            # After Tag 1: Move back, then down to find Tag 2
            print("Moving back and down to find Tag 2")
            search_for_next_tag(tag_id, 'down')
        elif tag_id == 3:
            # After Tag 2: Move back, then left to find Tag 3
            print("Moving back and left to find Tag 3")
            search_for_next_tag(tag_id, 'left')

        # After reaching the last tag, perform a 360° spin and land
        if tag_id == 3:
            print("Performing 360° spin and landing...")
            tello.rotate_clockwise(360)
            time.sleep(2)
            tello.land()

# Start the main loop
execute_movement_sequence()

# Cleanup
tello.streamoff()
cv2.destroyAllWindows()
