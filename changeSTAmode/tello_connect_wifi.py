import socket
import time

def connect_tello_to_router(router_ssid, router_password):
    # IP and port of the Tello
    tello_ip = '192.168.10.1'
    tello_port = 8889
    local_port = 9000  # Port for receiving Tello responses

    # Create a socket for communication with the Tello
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', local_port))  # Bind to any available port

    # Function to send a command and wait for a response
    def send_command(command):
        print("Sending command: " + command.strip())
        sock.sendto(command.encode('utf-8'), (tello_ip, tello_port))
        try:
            response, _ = sock.recvfrom(1024)  # Wait for a response (1KB buffer size)
            print("Response: " + response.decode('utf-8').strip())
            return response.decode('utf-8').strip()
        except socket.timeout:
            print("No response received.")
            return None

    # Set socket timeout for waiting on a response
    sock.settimeout(5)

    # 1. Send 'command' to initiate SDK mode
    if send_command('command\n') != 'ok':
        print("Failed to initiate SDK mode.")
        sock.close()
        return

    # 2. Send Wi-Fi credentials
    wifi_command = 'ap %s %s\n' % (router_ssid, router_password)
    if send_command(wifi_command) != 'ok':
        print("Failed to send Wi-Fi credentials.")
    
    # Close the socket
    sock.close()

# Example usage
connect_tello_to_router('TP-Link_F4D0', '05229611')