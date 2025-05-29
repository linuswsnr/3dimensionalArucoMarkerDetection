import cv2
import cv2.aruco as aruco
import numpy as np


# 3D-Koordinaten der Marker-Eckpunkte (bei flachem Marker auf der XY-Ebene)
def get_marker_3d_points(marker_size):
    half_size = marker_size / 2
    return np.array([
        [-half_size, half_size, 0],
        [half_size, half_size, 0],
        [half_size, -half_size, 0],
        [-half_size, -half_size, 0]
    ], dtype=np.float32)


# Berechnung der Pose mit `solvePnP`
def estimate_pose(corners, marker_size, camera_matrix, dist_coeffs):
    marker_3d_points = get_marker_3d_points(marker_size)
    success, rvecs, tvecs = cv2.solvePnP(marker_3d_points, corners[0], camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE)

    if success:
        return np.array(rvecs, dtype=np.float32), np.array(tvecs, dtype=np.float32)
    else:
        return None, None


# Marker-Größe in Metern
marker_size = 0.03

# Replace with the updated IP address of your ESP32
ip_address = '192.168.0.156'
url = f'http://{ip_address}:81/stream'

# Variablen für Kamerakalibrierung (initialisiert mit Platzhaltern)
c, Lx, Ly = -0.00152, 0.0000022, 0.0000022
fx, fy, cx, cy = c/Lx, -c/Ly, 803.66, 601.33  # Brennweiten & optischer Mittelpunkt (Anpassen nach Kalibrierung)
k1, k2, p1, p2, k3 = 0.1, -0.05, 0, 0, 0  # Verzerrungswerte (Ersetzen nach Kalibrierung)

# Kameramatrix und Verzerrungskoeffizienten mit Variablen
camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0, 0, 1]], dtype=np.float32)

dist_coeffs = np.array([k1, k2, p1, p2, k3], dtype=np.float32)


def main():
    cap = cv2.VideoCapture(url)

    if not cap.isOpened():
        print("Error: Cannot open video stream")
        exit()
    else:
        print("Success: Starting video stream")

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        corners, ids, _ = detector.detectMarkers(frame)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

            for i in range(len(ids)):
                rvecs, tvecs = estimate_pose(corners[i], marker_size, camera_matrix, dist_coeffs)

                # Zeichnet die Achsen des Markers
                if rvecs is not None and tvecs is not None:
                    if rvecs.size == 3 and tvecs.size == 3:
                        rvecs = rvecs.reshape((3, 1))
                        tvecs = tvecs.reshape((3, 1))

                        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs, tvecs, marker_size)
                    else:
                        print("⚠ Warnung: Unerwartete Pose-Schätzung, Werte nicht wie erwartet!")
                else:
                    print(f"⚠ Warnung: Pose-Schätzung für Marker {ids[i][0]} fehlgeschlagen!")

                print(f"Marker {ids[i][0]} - Abstand: {tvecs.flatten() if tvecs is not None else 'N/A'}, Orientierung: {rvecs.flatten() if rvecs is not None else 'N/A'}")

        cv2.imshow('ESP32 ArUco Detection', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
