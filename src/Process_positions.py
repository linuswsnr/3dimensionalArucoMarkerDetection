# Komplett korrigierter Code mit JSON-Einbindung und richtiger Interpretation

import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import cv2
import math
import params

# mehr nachkommastellen für bessere Genauigkeit
np.set_printoptions(precision=10, suppress=True)

def build_camera_pose_dataframe(solved_cameras_dict):
    """
    Erstellt einen DataFrame mit Position und Blickrichtung der Kameras.

    Args:
        solved_cameras_dict (dict): {camera_id: 4x4-Transformationsmatrix}

    Returns:
        pd.DataFrame: Enthält id, x, z, dir_x, dir_z, angle_rad, angle_deg
    """
    data = []
    for cam_id, T in solved_cameras_dict.items():
        x = T[0, 3]
        z = T[2, 3]
        dir_x = T[0, 2]
        dir_z = T[2, 2]
        angle_rad = np.arctan2(dir_z, dir_x)
        angle_deg = np.degrees(angle_rad)
        data.append({
            'id': cam_id,
            'x': x,
            'z': z,
            'dir_x': dir_x,
            'dir_z': dir_z,
            'angle_rad': angle_rad,
            'angle_deg': angle_deg
        })
    return pd.DataFrame(data)

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

def solve_all_cameras(camera_views, solved_cameras, marker_to_camera_map):
    """
    Berechnet rekursiv die globalen Posen aller Kameras, die Marker anderer Kameras sehen.

    Args:
        camera_views (dict): Kamera-ID → Liste von Marker-Detektionen (rvec, tvec)
        solved_cameras (dict): Kamera-ID → globale 4x4-Transformationsmatrix
        marker_to_camera_map (dict): Marker-ID → Kamera-ID

    Returns:
        dict: Aktualisiertes solved_cameras mit allen berechenbaren Kameras
    """
    unsolved_cameras = set(camera_views.keys()) - set(solved_cameras.keys())
    progress = True

    while progress and unsolved_cameras:
        progress = False
        for cam_id in list(unsolved_cameras):
            detections = camera_views.get(cam_id, [])
            for det in detections:
                marker_id = det['detected_id']
                other_cam_id = marker_to_camera_map.get(marker_id)

                # Nur Marker verwenden, deren Kamera bereits gelöst ist
                if other_cam_id in solved_cameras:
                    # 1. Transformation Marker → aktuelle Kamera
                    T_marker_to_cam = rvec_tvec_to_matrix(det['rvec'], det['tvec'])
                    T_cam_to_marker = np.linalg.inv(T_marker_to_cam)

                    # 2. Transformation Marker → global (von anderer Kamera)
                    T_marker_to_global = solved_cameras[other_cam_id]

                    # 3. Neue Kamera berechnen
                    T_cam_to_global = T_marker_to_global @ T_cam_to_marker

                    # 4. Speichern und Kamera als gelöst markieren
                    solved_cameras[cam_id] = T_cam_to_global
                    unsolved_cameras.remove(cam_id)
                    progress = True
                    print(f"Kamera {cam_id} gelöst über Marker {marker_id} (Kamera {other_cam_id})")
                    break  # nächste Kamera prüfen

    if unsolved_cameras:
        print("Warnung: Nicht alle Kameras konnten gelöst werden:", unsolved_cameras)

    return solved_cameras

def visualize_camera_positions(df):
    """
    Visualisiert die Positionen und Blickrichtungen der Kameras in der XZ-Ebene.

    Args:
        df (pd.DataFrame): Muss Spalten enthalten: id, x, z, dir_x, dir_z
    """
    windowsize = 2
    marker_text_distance = windowsize / 10
    cam_text_distance = windowsize / 15
    arrow_length = windowsize / 10
    arrow_head_width = windowsize / 30
    arrow_head_length = windowsize / 20

    fig, ax = plt.subplots(figsize=(6, 6))
    
    # Nullpunktwürfel zeichnen
    cube = plt.Rectangle((-params.MARKERLENGTH / 2, -params.MARKERLENGTH / 2), params.MARKERLENGTH, params.MARKERLENGTH, color='grey', alpha=1, label='Nullpunktwürfel', zorder=4)
    ax.add_patch(cube)
    ax.text(marker_text_distance, 0, "M1", fontsize=9, ha='center', va='center', color='black')
    ax.text(0, -marker_text_distance, "M2", fontsize=9, ha='center', va='center', color='black')
    ax.text(-marker_text_distance, 0, "M3", fontsize=9, ha='center', va='center', color='black')
    ax.text(0, marker_text_distance, "M0", fontsize=9, ha='center', va='center', color='black')

    # Positionen und Blickrichtungen einzeichnen
    for _, row in df.iterrows():
        x, z = row['x'], row['z']
        dx, dz = row['dir_x'], row['dir_z']
        cam_id = row['id']

        ax.plot(x, z, 'bo')  # Kameraposition als blauer Punkt
        ax.arrow(x, z, dx * arrow_length, dz * arrow_length, head_width=arrow_head_width, head_length=arrow_head_length, fc='r', ec='r', zorder=4)  # Blickrichtung
        ax.text(x + cam_text_distance, z + cam_text_distance, f"Cam {int(cam_id)}", fontsize=9)

    # Achsenformat
    ax.set_xlim(-windowsize, windowsize)
    ax.set_ylim(-windowsize, windowsize)
    ax.set_xlabel("X")
    ax.set_ylabel("Z")
    ax.set_title("Globale Camera Positions and Directions")
    ax.grid(False, zorder=5)  
    ax.set_aspect('equal')
    plt.tight_layout()
    plt.show()



#--------------------------------------------------------------------------------#
# Hauptprogramm
#--------------------------------------------------------------------------------#


# 1. Lade Daten
camera_views = load_valid_marker_data("src/marker_positions_rvecs_tvecs.json")
print("\n1. Kamera-Ansichten geladen:", camera_views)

# 2. leere Kamera-Ansichten entfernen
camera_views = {k: v for k, v in camera_views.items() if v}
print("\n2. Kamera-Ansichten nach Entfernen leerer Einträge:", camera_views)

# 3. Finde Anker-Kamera (z. B. bevorzugt Kamera 5)
anchor_cam_id, anchor_detection = find_anchor_camera(camera_views, params.CAMERA_ID, params.ANCHOR_MARKER_IDS)
print(f"\n3. Anker-Kamera gefunden: {anchor_cam_id}, Marker-Detektion: {anchor_detection}")
if anchor_cam_id is None:
    raise ValueError("Keine Ankerkamera erkannt.")
    exit(1)

# 4. Berechne globale Pose der Ankerkamera, update unsolved cameras and solved cameras
Transformer_matrix_marker_to_cam = rvec_tvec_to_matrix(anchor_detection['rvec'], anchor_detection['tvec'])
Transformer_matrix_cam_to_marker = np.linalg.inv(Transformer_matrix_marker_to_cam)

anchor_marker_id = anchor_detection['detected_id']

Transformer_matrix_marker_to_global = params.ANCHOR_MARKER_WORLD_POSES[anchor_marker_id]
Transformer_matrix_cam_to_global = Transformer_matrix_marker_to_global @ Transformer_matrix_cam_to_marker
Transformer_matrix_global_to_cam = np.linalg.inv(Transformer_matrix_cam_to_global)

solved_cameras = {anchor_cam_id: Transformer_matrix_global_to_cam}




print("Transformationsmatrix Anker Kamera zu Nullpunktwürfel:")
print(Transformer_matrix_global_to_cam)


# 5. Iteriere über alle Kameras und berechne deren globale Posen
solved_cameras = solve_all_cameras(camera_views, solved_cameras, params.MARKER_TO_CAMERA)



# 10. Dataframe erstellen mit allen Kamoerapositionen und deren Blickrichtungen
global_camera_poses_positions = build_camera_pose_dataframe(solved_cameras)
print("\n10. DataFrame mit Kamerapositionen und Blickrichtungen:")
print(global_camera_poses_positions)
print("\nSolved cameras:\n", solved_cameras)

visualize_camera_positions(global_camera_poses_positions)
