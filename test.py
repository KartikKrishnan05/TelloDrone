from djitellopy import Tello
import cv2
import pygame
import numpy as np
import time

# Speed of the drone
S = 60
FPS = 120  # Frames per second of the pygame window display

class FrontEnd(object):
    """ Maintains the Tello display and moves it through the keyboard keys.
        Press escape key to quit.
        The controls are:
            - T: Takeoff
            - L: Land
            - Arrow keys: Forward, backward, left and right.
            - A and D: Counter clockwise and clockwise rotations (yaw)
            - W and S: Up and down.
            - R: Start recording video
            - F: Stop recording video
    """

    def __init__(self):
        # Init pygame
        pygame.init()

        # Create pygame window
        pygame.display.set_caption("Tello Downward Camera Stream")
        self.screen = pygame.display.set_mode([960, 720])

        # Init Tello object that interacts with the Tello drone
        self.tello = Tello()

        # Drone velocities between -100~100
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        self.speed = 10
        self.send_rc_control = False

        # Set downward camera mode
        self.downward_camera = True  # We will switch to the downward camera

        # Create update timer
        pygame.time.set_timer(pygame.USEREVENT + 1, 1000 // FPS)

        # Video recording attributes
        self.recording = False  # Flag to indicate recording status
        self.video_writer = None  # OpenCV VideoWriter object for recording

    def run(self):
        self.tello.connect()
        self.tello.set_speed(self.speed)

        # In case streaming is on. This happens when we quit this program without the escape key.
        self.tello.streamoff()
        self.tello.streamon()

        # Switch to the downward-facing camera
        if self.downward_camera:
            self.tello.set_video_direction(Tello.CAMERA_DOWNWARD)

        frame_read = self.tello.get_frame_read()

        should_stop = False
        while not should_stop:
            for event in pygame.event.get():
                if event.type == pygame.USEREVENT + 1:
                    self.update()
                elif event.type == pygame.QUIT:
                    should_stop = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        should_stop = True
                    else:
                        self.keydown(event.key)
                elif event.type == pygame.KEYUP:
                    self.keyup(event.key)

            if frame_read.stopped:
                break

            self.screen.fill([0, 0, 0])

            frame = frame_read.frame
            # Display battery status on the frame
            text = "Battery: {}%".format(self.tello.get_battery())
            cv2.putText(frame, text, (5, 720 - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame = np.rot90(frame)
            frame = np.flipud(frame)

            # If recording is on, write the frame to the video file
            if self.recording and self.video_writer is not None:
                self.video_writer.write(cv2.cvtColor(frame_read.frame, cv2.COLOR_BGR2RGB))  # Write the original frame

            frame = pygame.surfarray.make_surface(frame)
            self.screen.blit(frame, (0, 0))
            pygame.display.update()

            time.sleep(1 / FPS)

        # Call it always before finishing. To deallocate resources.
        if self.recording:
            self.stop_recording()  # Stop recording if still on when exiting

        self.tello.end()

    def keydown(self, key):
        """ Update velocities based on key pressed """
        if key == pygame.K_UP:  # Forward velocity
            self.for_back_velocity = S
        elif key == pygame.K_DOWN:  # Backward velocity
            self.for_back_velocity = -S
        elif key == pygame.K_LEFT:  # Left velocity
            self.left_right_velocity = -S
        elif key == pygame.K_RIGHT:  # Right velocity
            self.left_right_velocity = S
        elif key == pygame.K_w:  # Up velocity
            self.up_down_velocity = S
        elif key == pygame.K_s:  # Down velocity
            self.up_down_velocity = -S
        elif key == pygame.K_a:  # Yaw counter clockwise velocity
            self.yaw_velocity = -S
        elif key == pygame.K_d:  # Yaw clockwise velocity
            self.yaw_velocity = S
        elif key == pygame.K_r:  # Start recording
            self.start_recording()
        elif key == pygame.K_f:  # Stop recording
            self.stop_recording()

    def keyup(self, key):
        """ Update velocities based on key released """
        if key == pygame.K_UP or key == pygame.K_DOWN:  # Zero forward/backward velocity
            self.for_back_velocity = 0
        elif key == pygame.K_LEFT or key == pygame.K_RIGHT:  # Zero left/right velocity
            self.left_right_velocity = 0
        elif key == pygame.K_w or key == pygame.K_s:  # Zero up/down velocity
            self.up_down_velocity = 0
        elif key == pygame.K_a or key == pygame.K_d:  # Zero yaw velocity
            self.yaw_velocity = 0
        elif key == pygame.K_t:  # Takeoff
            self.tello.takeoff()
            self.send_rc_control = True
        elif key == pygame.K_l:  # Land
            self.tello.land()
            self.send_rc_control = False

    def update(self):
        """ Send velocities to Tello """
        if self.send_rc_control:
            self.tello.send_rc_control(self.left_right_velocity, self.for_back_velocity, self.up_down_velocity, self.yaw_velocity)

    def start_recording(self):
        """ Start recording the video stream """
        if not self.recording:
            print("Recording started")
            self.recording = True
            # Initialize the VideoWriter with codec, frame rate, and size
            self.video_writer = cv2.VideoWriter('drone_recording.avi', cv2.VideoWriter_fourcc(*'XVID'), FPS, (960, 720))

    def stop_recording(self):
        """ Stop recording the video stream """
        if self.recording:
            print("Recording stopped")
            self.recording = False
            # Release the video writer
            if self.video_writer is not None:
                self.video_writer.release()
                self.video_writer = None


def main():
    frontend = FrontEnd()
    frontend.run()


if __name__ == '__main__':
    main()
