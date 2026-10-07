import sys

from djitellopy import Tello

# Usage: python change_nt_type.py <router-ssid> <router-password>
if len(sys.argv) != 3:
    sys.exit("usage: python change_nt_type.py <router-ssid> <router-password>")
wifi_ssid, wifi_password = sys.argv[1], sys.argv[2]

# Initialize Tello
tello = Tello()

# Connect to the Tello drone
tello.connect()


tello.takeoff()
tello.land()

# Send the 'ap' command to switch the drone to STA mode
tello.send_control_command(f"ap {wifi_ssid} {wifi_password}")

# Close the connection (the drone will reboot and connect to your router)
tello.end()

print("Tello is switching to STA mode and connecting to the router.")
