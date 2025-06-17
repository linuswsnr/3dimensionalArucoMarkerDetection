import numpy as np
import matplotlib.pyplot as plt  
import cv2 
from collections import defaultdict, deque  
import json

def redraw_marker_camera_network():


    # Zuordnung: Welche Marker gehören zu welcher Kamera
    marker_to_camera = {
        10: 1, 11: 1, 12: 1, 13: 1,
        20: 2, 21: 2, 22: 2, 23: 2,
        30: 3, 31: 3, 32: 3, 33: 3,
        40: 4, 41: 4, 42: 4, 43: 4,
        50: 5, 51: 5, 52: 5, 53: 5,
        60: 6, 61: 6, 62: 6, 63: 6
    }

    def get_camera_position_from_marker(rvec, tvec, marker_global_pos):
        rvec = np.array(rvec, dtype=np.float64)
        tvec = np.array(tvec, dtype=np.float64).reshape((3, 1)) / 100.0
        R, _ = cv2.Rodrigues(rvec)
        cam_pos = marker_global_pos.reshape((3, 1)) - R @ tvec
        return cam_pos

    # Initialisiere bekannte Positionen
    camera_positions = {}  # id -> np.array([x, y, z])
    marker_positions = {}  # id -> np.array([x, y, z])
    edges = defaultdict(list)

    with open ('Lukas_Linus/marker_positions_rvecs_tvecs.json', 'r') as file:
        data = json.load(file)

    # Verknüpfungen zwischen Kamera und Marker
    for entry in data:
        cam_id = entry["id"]
        for other in entry["Others"]:
            rvecs = other["Position"][0]["rvecs"]
            tvecs = other["Position"][1]["tvecs"]
            detected_id = other["detected_id"]
            if rvecs and tvecs and detected_id != "":
                try:
                    rvecs_np = np.array(rvecs, dtype=np.float64)
                    tvecs_np = np.array(tvecs, dtype=np.float64)
                    edges[f"C{cam_id}"].append((f"M{detected_id}", rvecs_np, tvecs_np))
                    edges[f"M{detected_id}"].append((f"C{cam_id}", rvecs_np, tvecs_np))
                except:
                    continue

    # Setze Marker 0 als Ursprung
    marker_positions["M0"] = np.array([0.0, 0.0, 0.0])
    queue = deque(["M0"])

    # Rekursive Bestimmung
    # Rekursive Bestimmung
    while queue:
        current = queue.popleft()
        if current.startswith("M"):
            marker_id = current
            for neighbor, rvecs, tvecs in edges[marker_id]:
                if neighbor not in camera_positions:
                    cam_pos = get_camera_position_from_marker(rvecs, tvecs, marker_positions[marker_id])
                    camera_positions[neighbor] = cam_pos
                    queue.append(neighbor)

                    # Wenn Kamera-Position bekannt wird, dann Marker dieser Kamera ebenfalls setzen
                    try:
                        cam_num = int(neighbor[1:])  # "C2" -> 2
                        for marker_id_local, marker_cam in marker_to_camera.items():
                            if marker_cam == cam_num:
                                marker_label = f"M{marker_id_local}"
                                if marker_label not in marker_positions:
                                    marker_positions[marker_label] = cam_pos.copy()
                                    queue.append(marker_label)
                    except:
                        pass

        elif current.startswith("C"):
            cam_id = current
            for neighbor, rvecs, tvecs in edges[cam_id]:
                if neighbor not in marker_positions:
                    cam_pos = camera_positions[cam_id]
                    rvec = np.array(rvecs, dtype=np.float64)
                    tvec = np.array(tvecs, dtype=np.float64).reshape((3, 1)) / 100.0
                    R, _ = cv2.Rodrigues(rvec)
                    marker_pos = cam_pos.reshape((3, 1)) + R @ tvec
                    marker_positions[neighbor] = marker_pos.flatten()
                    queue.append(neighbor)

    return camera_positions, marker_positions  

