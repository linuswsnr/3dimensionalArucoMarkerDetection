

import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import cv2
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
        dir_x = T[0, 2]*-1
        dir_z = T[2, 2]
        #dir_x = T[0, 2]
        #dir_z = T[1, 2]
        #dir_x = T[1, 2]
        #dir_z = T[2, 2]
        #angle_rad = np.arctan2(dir_z, dir_x)
        angle_rad = np.arctan2(dir_x, dir_z)
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
    print(data)
    return pd.DataFrame(data)

def flip_y_axis_rotation_(Matrix):
    """
    Kehrt nur die Rotation um die Y-Achse einer 4x4-Transformationsmatrix um.
    """
    Matrix_flipped = Matrix.copy()
    Matrix_flipped[:3, 0] *= -1  # invertiere X-Achse
    Matrix_flipped[:3, 2] *= -1  # invertiere Z-Achse
    return Matrix_flipped

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
    """
    Erzeugt eine 4x4-Transformationsmatrix aus rvec und tvec.
    """
    rvec = np.array(rvec, dtype=np.float64).reshape(3, 1)
    tvec = np.array(tvec, dtype=np.float64).reshape(3, 1)
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.array(tvec).reshape(3)
    return T

def try_solve_cameras_from_solved(camera_views, solved_cameras):
    """ 
    Versucht, Kameras zu lösen, die bereits eine bekannte Kamera sehen.
    Args:   
        camera_views (dict): Mapping von Kamera-IDs zu Marker-Detektionen.
        solved_cameras (dict): {camera_id: 4x4-Transformationsmatrix}
    Returns:
        dict: Aktualisierte solved_cameras mit neuen Kameras.   
    """
    for cam_id in list(solved_cameras.keys()):
        for detection in camera_views[cam_id]:
            detection_id = detection['detected_id']
            if detection_id // 10 in set(solved_cameras.keys()) or detection_id // 10 == 0:
                continue  # Marker gehört zu einer Kamera, die bereits gelöst ist
            else:
                Transformer_matrix_marker_to_cam = rvec_tvec_to_matrix(detection['rvec'], detection['tvec']) # Marker (an neuer Kamera) aus Sicht bekannter Kamera
                Transformer_matrix_global_to_cam = solved_cameras[cam_id] @ Transformer_matrix_marker_to_cam @ params.ANCHOR_MARKER_WORLD_POSES[detection_id % 10]

                solved_cameras[detection_id // 10] = Transformer_matrix_global_to_cam


    return solved_cameras

def try_solve_cameras_from_unsolved(camera_views, solved_cameras):
    unsolved_cameras = set(camera_views.keys()) - set(solved_cameras.keys())
    for cam_id in list(unsolved_cameras):
        for detection in camera_views[cam_id]:
            detection_id = detection['detected_id']
            if detection_id // 10 not in unsolved_cameras or detection_id % 10 == 0:
                continue  # Marker gehört zu einer Kamera, die bereits gelöst ist
            else:
                try:
                    Transformer_matrix_marker_to_cam = rvec_tvec_to_matrix(detection['rvec'], detection['tvec']) # Marker (an neuer Kamera) aus Sicht bekannter Kamera
                    Transformer_matrix_marker_to_cam = np.linalg.inv(Transformer_matrix_marker_to_cam)
                    Transformer_matrix_global_to_cam = solved_cameras[cam_id] @ Transformer_matrix_marker_to_cam @ params.ANCHOR_MARKER_WORLD_POSES[detection_id % 10]

                    solved_cameras[detection_id // 10] = Transformer_matrix_global_to_cam
                    unsolved_cameras.remove(cam_id)
                except:
                    pass
    return solved_cameras

def visualize_camera_positions(df):
    """
    Visualisiert die Positionen und Blickrichtungen der Kameras in der XZ-Ebene.

    Args:
        df (pd.DataFrame): Muss Spalten enthalten: id, x, z, dir_x, dir_z
    """
    windowsize = 1 # in meters for the axes
    marker_text_distance = windowsize / 15
    cam_text_distance = windowsize / 50
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

def process_positions():
    """
    Hauptfunktion zum Verarbeiten der Kamerapositionen und Marker-Detektionen.
    wird von main.py aufgerufen.
    Führt die folgenden Schritte aus:
    1. Lädt die Kameradaten aus marker_positions_rvecs_tvecs
    2. Findet die Anker-Kamera, die einen Nullpunktmarker sieht
    3. Berechnet die globale Pose der Anker-Kamera und aktualisiert solved_cameras
    4. Iteriert über alle Kameras und berechnet deren globale Posen
    5. Erstellt einen DataFrame mit allen Kamerapositionen und deren Blickrichtungen
    6. Visualisiert die Kamerapositionen und Blickrichtungen in der XZ-Ebene
    """
    try:
        # 1. Lade Daten
        camera_views = load_valid_marker_data("src/marker_positions_rvecs_tvecs.json")
        print("\n1. Kamera-Ansichten geladen:", camera_views)

        # 2. Finde Anker-Kamera (z. B. bevorzugt Kamera 5)
        anchor_cam_id, anchor_detection = find_anchor_camera(camera_views, params.CAMERA_ID, params.ANCHOR_MARKER_IDS)
        if anchor_cam_id is None:
            raise ValueError("Keine Ankerkamera erkannt.")
            exit(1)

        # 3. Berechne globale Pose der Ankerkamera, update solved cameras
        anchor_marker_id = anchor_detection['detected_id']

        Transformer_matrix_marker_to_cam = rvec_tvec_to_matrix(anchor_detection['rvec'], anchor_detection['tvec']) # Marker (an neuer Kamera) aus Sicht bekannter Kamera
        #Transformer_matrix_marker_to_cam = np.linalg.inv(Transformer_matrix_marker_to_cam)
        Transformer_matrix_global_to_cam = Transformer_matrix_marker_to_cam @ params.ANCHOR_MARKER_WORLD_POSES[anchor_marker_id]
        Transformer_matrix_global_to_cam = np.linalg.inv(Transformer_matrix_global_to_cam)

        # if anchor_detection['rvec'][1] < 0 and anchor_detection['tvec'][0] < 0:
        #     Transformer_matrix_global_to_cam[0, 3] *= -1

        # if anchor_detection['rvec'][1] < 0 and anchor_detection['tvec'][0] > 0:
        #     Transformer_matrix_global_to_cam[0, 3] *= -1

        # if anchor_detection['rvec'][1] > 0 and anchor_detection['tvec'][0] < 0:
        #     Transformer_matrix_global_to_cam[0, 3] *= -1

        Transformer_matrix_global_to_cam[0, 3] *= -1
        # Transformer_matrix_global_to_cam = flip_y_axis_rotation_(Transformer_matrix_global_to_cam) 


        # angle_rad = np.radians(-90)
        # cos_a = np.cos(angle_rad)
        # sin_a = np.sin(angle_rad)
        # R_y = np.array([
        #     [ cos_a, 0.0, sin_a, ],
        #     [   0.0, 1.0,   0.0, ],
        #     [-sin_a, 0.0, cos_a, ]
        # ])
        # Transformer_matrix_global_to_cam[:3, :3] = R_y @ Transformer_matrix_global_to_cam[:3, :3] 


        print(Transformer_matrix_global_to_cam)

        solved_cameras = {anchor_cam_id: Transformer_matrix_global_to_cam}

        # 4. Iteriere über alle Kameras und berechne deren globale Posen
        for _ in range(5):  # Max 5 Iterationen
            newly = try_solve_cameras_from_solved(camera_views, solved_cameras)
            solved_cameras.update(newly)
            newly = try_solve_cameras_from_unsolved(camera_views, solved_cameras)
            solved_cameras.update(newly)

        # 5. Dataframe erstellen mit allen Kamoerapositionen und deren Blickrichtungen
        global_camera_poses_positions = build_camera_pose_dataframe(solved_cameras)

        # 6. Visualisiere Kamerapositionen und Blickrichtungen
        visualize_camera_positions(global_camera_poses_positions)
        return
    
    except Exception as e:
        print(f"Fehler beim Verarbeiten der Kamerapositionen: {e}")
        return
