# TelloDrone

Autonomous indoor flight with a DJI Tello drone using **ArUco markers** for navigation. The drone takes off on its own, finds markers with its camera, flies from one marker to the next, and returns — a demo of how drones could collect data from stations in a **Wireless Sensor Network**.

## Why

In a wireless sensor network, stations spread over a large area (farms, forests, cities) measure things like temperature or humidity. Sending that data over long distances costs a lot of energy and drains their batteries. Instead, a drone can fly to each station on a schedule — say once a day — and pick up the data at short range. The stations save power, batteries last longer, and the drone becomes a mobile data link.

In this project, ArUco markers stand in for the sensor stations.

## How it works

1. The drone streams video over Wi-Fi; OpenCV detects ArUco markers in each frame.
2. The marker's position in the image tells the drone which way to correct (left/right, up/down), and the marker's size in pixels gives the distance using the pinhole camera model: `distance = real_width × focal_length / pixel_width`.
3. If the next marker isn't visible, the drone rotates in place to search for it.
4. Once it reaches a marker it moves on to the next ID; at the end it flies back and lands.

## Scripts

| Path | What it does |
| --- | --- |
| `ArucoTagScripts/Floor/main.py` | Main demo: fly to markers on the floor and measure distance to them |
| `ArucoTagScripts/Floor/FloorOneTag.py`, `FloorMultipleTags.py` | Find and fly to one marker / a sequence of markers on the floor |
| `ArucoTagScripts/Floor/mainFlightBack.py` | Marker route with a return flight, flight log and battery display |
| `ArucoTagScripts/Floor/findX.py` | Centre the drone over an "X" mark on the floor |
| `ArucoTagScripts/Wall/OneTag.py`, `TagsOnWall.py` | The same with markers mounted on a wall |
| `ArucoTagScripts/ArucoTag/createTags.py` | Generate printable markers (4×4 and 6×6 dictionaries in `aruco_tags_*`) |
| `ArucoTagScripts/ArucoTag/getSize.py` | Measure a marker's pixel size to calibrate the distance formula |
| `keyboard_control.py` | Fly manually with the keyboard (pygame window with live video) |
| `changeSTAmode/` | Put the Tello on an existing Wi-Fi router (station mode) and scan the network to find it, for multi-drone setups |
| `checkcv2.py` | Check that your OpenCV build includes the ArUco module |

## Getting started

```bash
pip install djitellopy opencv-contrib-python numpy pygame
```

1. Print the markers from `ArucoTagScripts/ArucoTag/aruco_tags_6x6/` and place them around the room.
2. Turn on the Tello and connect your computer to its Wi-Fi.
3. Run a flight script, e.g.

   ```bash
   python ArucoTagScripts/Floor/main.py
   ```

To connect the drone to your own router instead:

```bash
python changeSTAmode/tello_connect_wifi.py "<router-ssid>" "<router-password>"
```

> Fly in an open space and keep a hand near the keyboard — the scripts send real movement commands.

## Team

Kartik Krishnan and Emirhan Afsin, 2024.
