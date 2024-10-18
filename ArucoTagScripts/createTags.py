from PIL import Image
import numpy as np
import cv2
import os

# Function to create an ArUco marker image
def create_aruco_marker(id, marker_size, save_path):
    # Get the predefined dictionary for ArUco markers
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

    # Generate the marker
    marker_img = np.zeros((marker_size, marker_size), dtype=np.uint8)
    cv2.aruco.generateImageMarker(aruco_dict, id, marker_size, marker_img, 1)

    # Convert to a PIL Image
    marker_image = Image.fromarray(marker_img)

    # Create the directory if it doesn't exist
    os.makedirs(save_path, exist_ok=True)

    # Save the marker image
    file_name = f"{save_path}/aruco_marker_{id}.png"
    marker_image.save(file_name)
    print(f"Saved ArUco marker ID {id} as {file_name}")

# Configuration
marker_size = 200  # Size of the ArUco marker image
save_path = "./aruco_tags_6x6"  # Folder to save the tags
num_markers = 4  # Number of markers to create

# Generate and save ArUco markers
for i in range(num_markers):
    create_aruco_marker(i, marker_size, save_path)
