# Komplett korrigierter Code mit JSON-Einbindung und richtiger Interpretation

import json
import numpy as np
import matplotlib.pyplot as plt
import cv2
import math
import params

# mehr nachkommastellen für bessere Genauigkeit
np.set_printoptions(precision=10, suppress=True)

def load_valid_marker_data(path):
    with open(path, 'r') as f:
        raw_data = json.load(f)

    camera_views = {}
    for entry in raw_data:
        cam_id = entry['id']
        valid_detections = []
        for other in entry['Others']:
            try:
                detected_id = int(other['detected_id'])
                rvec = other['Position'][0]['rvecs']
                tvec = other['Position'][1]['tvecs']
                if len(rvec) == 3 and len(tvec) == 3:
                    valid_detections.append({
                        'detected_id': detected_id,
                        'rvec': rvec,
                        'tvec': tvec
                    })
            except (ValueError, KeyError, IndexError, TypeError):
                continue
        camera_views[cam_id] = valid_detections
    return camera_views

def find_anchor_camera(camera_views, camera_priority_id, anchor_ids={0, 1, 2, 3}):
    """
    Findet die Anker-Kamera, die einen Nullpunktmarker sieht.

    Args:
        camera_views (dict): Mapping von Kamera-IDs zu Marker-Detektionen.
        camera_priority_id (int): Bevorzugte Kamera-ID.
        anchor_ids (set): Set von Marker-IDs des Nullpunktwürfels.

    Returns:
        tuple: (Kamera-ID, Detektionseintrag) oder (None, None), wenn keine Ankerkamera gefunden wird.
    """
    # 1. Zuerst prüfen, ob die priorisierte Kamera einen Anker-Marker sieht
    detections = camera_views.get(camera_priority_id, [])
    for det in detections:
        if det['detected_id'] in anchor_ids:
            return camera_priority_id, det

    # 2. Falls nicht, alle anderen Kameras durchsuchen
    for cam_id, detections in camera_views.items():
        for det in detections:
            if det['detected_id'] in anchor_ids:
                return cam_id, det

    # 3. Kein Anker gefunden
    return None, None

def rvec_tvec_to_matrix(rvec, tvec):
    """Erzeugt eine 4x4-Transformationsmatrix aus rvec und tvec."""
    rvec = np.array(rvec, dtype=np.float64).reshape(3, 1)
    tvec = np.array(tvec, dtype=np.float64).reshape(3, 1)
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.array(tvec).reshape(3)
    return T

###############


# Daten einlesen
json_path = "src\\marker_positions_rvecs_tvecs.json"

camera_views = load_valid_marker_data(json_path)
print("Kamera-Ansichten geladen:", camera_views)

cam_id, anchor_detection = find_anchor_camera(camera_views, camera_priority_id=5)
if cam_id is not None:
    print(f"Kamera {cam_id} erkennt Marker {anchor_detection['detected_id']}")
else:
    print("Keine Kamera erkennt einen Anker-Marker.")


# 1. Lade Daten
camera_views = load_valid_marker_data("src/marker_positions_rvecs_tvecs.json")

# 2. Finde Anker-Kamera (z. B. bevorzugt Kamera 5)
anchor_cam_id, anchor_detection = find_anchor_camera(camera_views, params.CAMERA_ID, params.ANCHOR_MARKER_IDS)

if anchor_cam_id is None:
    raise ValueError("Keine Ankerkamera erkannt.")

# 3. Berechne globale Pose der Ankerkamera
Transformer_matrix_marker_to_cam = rvec_tvec_to_matrix(anchor_detection['rvec'], anchor_detection['tvec'])
Transformer_matrix_cam_to_marker = np.linalg.inv(Transformer_matrix_marker_to_cam)

anchor_marker_id = anchor_detection['detected_id']
Transformer_matrix_marker_to_global = params.ANCHOR_MARKER_WORLD_POSES[anchor_marker_id]
Transformer_matrix_cam_to_global = Transformer_matrix_marker_to_global @ Transformer_matrix_cam_to_marker

solved_cameras = {anchor_cam_id: Transformer_matrix_cam_to_global}
unsolved_cameras = set(camera_views.keys()) - {anchor_cam_id}

print(f"Ankerkamera: {anchor_cam_id}, sieht Marker {anchor_marker_id}")
print("T_cam_to_global:")
print(Transformer_matrix_cam_to_global)
print("________________")
print("T_marker_to_global:")
for row in Transformer_matrix_marker_to_global:
    print([f"{v:.6f}" for v in row])

print("\nT_cam_to_marker (invertiert):")
for row in Transformer_matrix_cam_to_marker:
    print([f"{v:.6f}" for v in row])

print("\nT_cam_to_global (Resultat):")
for row in Transformer_matrix_cam_to_global:
    print([f"{v:.6f}" for v in row])