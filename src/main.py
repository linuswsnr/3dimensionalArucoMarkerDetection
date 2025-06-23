"""
Authors: Linus Wasner, Lukas Bauer
Date: 2025-06-10
Project: 3dimensionalArucoMarkerDetection
Lekture: Echtzeitsysteme, Masterprogram advanced driver assistance systems, University of Applied Sciences Kempten

main script for the ArUco marker detection system.
This script initializes the camera, detects ArUco markers, updates their positions, and publishes the data via MQTT.
It also triggers the visualization of marker positions and updates the JSON file which contains marker data from all cameras.
"""

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

# Initialize the JSON file with default marker positions if it does not exist
with open('src/marker_positions_rvecs_tvecs.json', 'w') as file:
    json.dump(default_marker_list, file, indent=4)

def on_message(client, userdata, msg):
    """
    Callback function for MQTT messages.
    Updates the marker positions in the JSON file based on the received message.
    """
    try:
        camera_id = int(str(msg.topic)[-1])
        new_data = json.loads(msg.payload.decode())
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
    except Exception as e:
        print(f"Error processing message: {e}")

def visualize_camera_positions(df=None, ax=None, fig=None):
    """
    Visualizes or updates the positions and viewing directions of the cameras in the XZ-plane.
    If ax is given, the plot is updated in the same window.
    Args:
        df (pd.DataFrame): DataFrame with columns ['id', 'x', 'z', 'dir_x', 'dir_z'].
        ax (matplotlib.axes.Axes, optional): Axes to update. If None, a new figure is created.
        fig (matplotlib.figure.Figure, optional): Figure to update. If None, a new figure is created.
    """
    ax.cla()  # clear previous plot content

    windowsize = 0.5
    marker_text_distance = windowsize / 15
    cam_text_distance = windowsize / 50
    arrow_length = windowsize / 10
    arrow_head_width = windowsize / 30
    arrow_head_length = windowsize / 20

    # draw global origin
    cube = plt.Rectangle((-params.MARKERLENGTH / 2, -params.MARKERLENGTH / 2), params.MARKERLENGTH, params.MARKERLENGTH, color='grey', alpha=1, zorder=4)
    ax.add_patch(cube)
    ax.text(marker_text_distance, 0, "M1", fontsize=9, ha='center', va='center', color='black')
    ax.text(0, -marker_text_distance, "M2", fontsize=9, ha='center', va='center', color='black')
    ax.text(-marker_text_distance, 0, "M3", fontsize=9, ha='center', va='center', color='black')
    ax.text(0, marker_text_distance, "M0", fontsize=9, ha='center', va='center', color='black')

    if df is not None:
        # draw cameras
        for _, row in df.iterrows():
            x, z = row['x'], row['z']
            dx, dz = row['dir_x'], row['dir_z']
            cam_id = row['id']
            ax.plot(x, z, 'bo')
            ax.arrow(x, z, dx * arrow_length, dz * arrow_length, head_width=arrow_head_width, head_length=arrow_head_length, fc='r', ec='r', zorder=4)
            ax.text(x + cam_text_distance, z + cam_text_distance, f"Cam {int(cam_id)}", fontsize=9)

    ax.set_xlim(-windowsize, windowsize)
    ax.set_ylim(-windowsize, windowsize)
    ax.set_xlabel("X")
    ax.set_ylabel("Z")
    ax.set_title("Global Camera Positions and Directions")
    ax.grid(False, zorder=5)
    ax.set_aspect('equal')

    plt.tight_layout()
    plt.show()

#--------------------------------------------------------------------------------#
# MQTT Setup
#--------------------------------------------------------------------------------#
# Client erstellen und verbinden
client = mqtt.Client()
client.on_message = on_message
client.connect(params.BROKER, params.PORT, 60)

# subscribe topics of all cameras except the current one
clients = [(params.TOPIC_1, 1), (params.TOPIC_2, 1), (params.TOPIC_3, 1), (params.TOPIC_4, 1), (params.TOPIC_5, 1), (params.TOPIC_6, 1)]
clients.pop(params.CAMERA_ID-1)
client.subscribe(clients)
client.loop_start()


### should be moved to params.py
# Load predefined dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

# start the camera stream
cap = setup_camera_stream()

markers = []
prev_second = datetime.now().second

# initialize the plot
plt.ion()
fig, ax = plt.subplots(figsize=(6, 6))

while True:
    now = datetime.now()

    # 1. get the current frame from the own camera
    frame, photo_timestamp = get_frame(cap)     
    cv2.imshow("ESP32 Cam Stream", frame)

    # 2. detect markers in the current frame
    detected_markers = get_marker_detections(frame, photo_timestamp)

    # 3. update detected marker objects
    for marker in detected_markers:
        if marker.detected_id in [m.detected_id for m in markers]:
            # update existing marker
            existing_marker = next(m for m in markers if m.detected_id == marker.detected_id)
            existing_marker.update_position()
        else:
            markers.append(marker)
            marker.update_position()

    # 4. clear list with marker objects 
    detected_markers = [] 
    
    # 5. remove markers that have not been updated for more than 5 seconds
    for marker in markers:
        time_diff = (now - marker.timestamp).total_seconds()
        if time_diff > 2:
            marker.delete_position()
            markers.remove(marker)

    # 6. mqtt update and redraw the network every second
    if now.second != prev_second:           
        camera_dict = get_camera_dict(params.CAMERA_ID)
        client.publish(params.TOPIC_5, json.dumps(camera_dict))

        global_camera_poses_positions = process_positions()
        print("global_camera_poses_positions:\n", global_camera_poses_positions)

        visualize_camera_positions(global_camera_poses_positions, ax, fig)
        fig.canvas.draw()    
        fig.canvas.flush_events() 
        
        prev_second = now.second

