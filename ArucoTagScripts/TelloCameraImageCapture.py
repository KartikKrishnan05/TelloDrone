from djitellopy import Tello
import cv2
import pygame
import numpy as np
import glob
import time

# Speed and Frames per second
S = 60
FPS = 120

# Initialize screen width and height for pygame
width = 640  # WIDTH OF THE IMAGE
height = 480  # HEIGHT OF THE IMAGE
startCounter = 1  # 0 FOR FLIGHT 1 FOR TESTING

# Image paths
cameraCalibPath = "./cameraCalibration/"
imgPath = cameraCalibPath + "images/"

# Set up pygame window
pygame.init()
pygame.display.set_caption("Tello Camera Calibration")
screen = pygame.display.set_mode([640, 480])

# CONNECT TO TELLO
tello = Tello()
tello.connect()

print("Tello battery status: ", tello.get_battery())
print("Tello temperature status: ", tello.get_temperature())

# Stream video from the drone
tello.streamoff()
tello.streamon()

# Timer for updating frame
pygame.time.set_timer(pygame.USEREVENT + 1, 1000 // FPS)

# Prepare for camera calibration
chessboardSize = (9, 7)
frameSize = (width, height)

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points, like (0,0,0), (1,0,0), (2,0,0), ... (6,5,0)
objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboardSize[0], 0:chessboardSize[1]].T.reshape(-1, 2)

size_of_chessboard_squares_mm = 21
objp = objp * size_of_chessboard_squares_mm

# Arrays to store object points and image points from all the images
objpoints = []  # 3D point in real world space
imgpoints = []  # 2D points in image plane

i = 0  # Image counter for saving images

def capture_image():
    """ Capture image from the drone feed and save for calibration """
    global i
    frame_read = tello.get_frame_read()
    myFrame = frame_read.frame
    img = cv2.resize(myFrame, (width, height))
    
    screen.fill([0, 0, 0])
    
    # Display battery status on the frame
    text = "Battery: {}%".format(tello.get_battery())
    cv2.putText(img, text, (5, height - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    frame = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    frame = np.rot90(frame)
    frame = np.flipud(frame)

    frame = pygame.surfarray.make_surface(frame)
    screen.blit(frame, (0, 0))
    pygame.display.update()

    return img

def process_images_for_calibration():
    """ Process all saved images for camera calibration """
    global objpoints, imgpoints
    images = glob.glob(imgPath + '*.jpg')

    for image in images:
        img = cv2.imread(image)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, chessboardSize, None)

        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners)

            # Draw and display the corners
            cv2.drawChessboardCorners(img, chessboardSize, corners2, ret)
            cv2.imshow('Image', img)
            cv2.waitKey(500)
            cv2.destroyAllWindows()

def calibrate_camera():
    """ Perform the camera calibration """
    ret, cameraMatrix, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, frameSize, None, None)
    
    with open(cameraCalibPath + "CameraCalibrationParameters.txt", "w+") as f:
        print("\n\nCamera Matrix:\n", cameraMatrix)
        f.write("Camera Matrix:\n")
        f.write(str(cameraMatrix))
        print("\n\nDistortion Parameters:\n", dist)
        f.write("\n\nDistortion Parameters:\n")
        f.write(str(dist))

    # Save calibration data to .json
    cv_file = cv2.FileStorage(cameraCalibPath + "CamCalibParam.json", cv2.FILE_STORAGE_WRITE)
    cv_file.write("Camera_Matrix", cameraMatrix)
    cv_file.write("Distortion_Parameters", dist)
    cv_file.release()

    # Calculate reprojection error
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], cameraMatrix, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error
    totalError = mean_error / len(objpoints)
    print("\nTotal error: {}".format(totalError))

def main():
    global i

    frame_read = tello.get_frame_read()
    should_stop = False
    while not should_stop:
        for event in pygame.event.get():
            if event.type == pygame.USEREVENT + 1:
                img = capture_image()

            elif event.type == pygame.QUIT:
                should_stop = True

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    should_stop = True
                elif event.key == pygame.K_s:
                    # Save the image when 's' key is pressed
                    cv2.imwrite(imgPath + "calibImg" + str(i + 1) + ".jpg", img)
                    print("Image taken", i + 1)
                    i += 1
                elif event.key == pygame.K_c:
                    # Perform calibration when 'c' key is pressed
                    process_images_for_calibration()
                    calibrate_camera()
                elif event.key == pygame.K_t:
                    # Takeoff when 't' key is pressed
                    tello.takeoff()
                elif event.key == pygame.K_l:
                    # Land when 'l' key is pressed
                    tello.land()

        if frame_read.stopped:
            break

        time.sleep(1 / FPS)

    tello.end()
    pygame.quit()

if __name__ == '__main__':
    main()
