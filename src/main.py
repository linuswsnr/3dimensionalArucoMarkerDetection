import json
import cv2
import cv2.aruco as aruco
import numpy as np
from datetime import datetime
import matplotlib.pyplot as plt     
import matplotlib.animation as animation
from matplotlib.lines import Line2D

from utils import setup_camera_stream, get_frame, get_marker_detections, get_camera_dict
from process_positions import process_positions
import params as params
import paho.mqtt.client as mqtt
import threading

json_lock = threading.Lock()

default_marker_list = [
    {
        "id": 1,
        "Others": [{"detected_id": "",
                    "Position":[{"rvecs": []}, {"tvecs": []}]}],
        "time": ""
    },
    {
        "id": 2,
        "Others": [{"detected_id": "",
                    "Position":[{"rvecs": []}, {"tvecs": []}]}],
        "time": ""
    },
    {
        "id": 3,
        "Others": [{"detected_id": "",
                    "Position":[{"rvecs": []}, {"tvecs": []}]}],
        "time": ""
    },
    {
        "id": 4,
        "Others": [{"detected_id": "",
                    "Position":[{"rvecs": []}, {"tvecs": []}]}],
        "time": ""
    },
    {
        "id": 5,
        "Others": [{"detected_id": "",
                    "Position":[{"rvecs": []}, {"tvecs": []}]}],
        "time": ""
    },
    {
        "id": 6,
        "Others": [{"detected_id": "",
                    "Position":[{"rvecs": []}, {"tvecs": []}]}],
        "time": ""
    }
]

with open('src/marker_positions_rvecs_tvecs.json', 'w') as file:
    json.dump(default_marker_list, file, indent=4)

def on_message(client, userdata, msg):
    camera_id = int(str(msg.topic)[-1])  # ID als int
    new_data = json.loads(msg.payload.decode())  # erwartet Dict mit 'Others' und 'time'
    with open('src/marker_positions_rvecs_tvecs.json', 'r') as file:
        marker_positions = json.load(file)
    for camera_dict in marker_positions:
        if camera_dict['id'] == camera_id:
            camera_dict['Others'] = new_data.get('Others', camera_dict['Others'])
            camera_dict['time'] = new_data.get('time', camera_dict['time'])
            print(f"Updated camera {camera_id} with new data.")
            break
    with open('src/marker_positions_rvecs_tvecs.json', 'w') as f:
        json.dump(marker_positions, f, indent=4)  

# Client erstellen und verbinden
client = mqtt.Client()
client.on_message = on_message
client.connect(params.BROKER, params.PORT, 60)

clients = [(params.TOPIC_1, 1), (params.TOPIC_2, 1), (params.TOPIC_3, 1), (params.TOPIC_4, 1), (params.TOPIC_5, 1), (params.TOPIC_6, 1)]
clients.pop(params.CAMERA_ID-1)

# Topic abonnieren
client.subscribe(clients)
client.loop_start()


# Load predefined dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)
# start the camera stream
cap = setup_camera_stream()

markers = []
prev_second = datetime.now().second

while True:
    now = datetime.now()

    # get the current frame from the camera
    frame, photo_timestamp = get_frame(cap)     
    cv2.imshow("ESP32 Cam Stream", frame)

    # detect markers in the current frame
    detected_markers = get_marker_detections(frame, photo_timestamp)

    # update detected markers
    for marker in detected_markers:
        if marker.detected_id in [m.detected_id for m in markers]:
            # update existing marker
            existing_marker = next(m for m in markers if m.detected_id == marker.detected_id)
            existing_marker.update_position()
        else:
            markers.append(marker)
            marker.update_position()
    detected_markers = []
    
    # update markers
    for marker in markers:
        time_diff = (now - marker.timestamp).total_seconds()
        if time_diff > 5:
            marker.delete_position()
            markers.remove(marker)

    # mqtt update and redraw the network every second
    if now.second != prev_second:           
        camera_dict = get_camera_dict(params.CAMERA_ID)
        client.publish(params.TOPIC_5, json.dumps(camera_dict))

        process_positions()
        
        prev_second = now.second

    if cv2.waitKey(10) & 0xFF == ord('q'):
        break
