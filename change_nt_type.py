from djitellopy import Tello

# Replace with your Wi-Fi network SSID and password
wifi_ssid = "TP_LinkF4D0"  # Your router's SSID (network name)
wifi_password = "05229611"  # Your router's password

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
