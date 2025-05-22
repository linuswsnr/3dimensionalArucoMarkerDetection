import datetime
import time
import cv2
import cv2.aruco as aruco
import os
import numpy as np

# Stremauflösung im Arduiono C Code auf
# 800 * 600
# ändern für Kalibrierungsbilder

# Replace with the IP address of your ESP32
USE_LAPTOP_CAM = False
IP_ADDRESS = '192.168.137.75'
URL = f'http://{IP_ADDRESS}:81/stream'

SCREENSHOT_DIR = 'C:/dev/Echtzeitsysteme/screenshots_valentin'
CALIBRATION_FILE_PATH = 'calibration.npz'

# Marker size in m
ARUCO_MARKER_SIZE = 0.1



def main():
    # Load the camera calibration data
    if os.path.exists(CALIBRATION_FILE_PATH):
        calibration_data = np.load(CALIBRATION_FILE_PATH)
        camera_matrix = calibration_data['camera_matrix']
        dist_coeffs = calibration_data['dist_coeffs']
    else:
        print('No calibration file found.')
        # Fallback for ESP32-CAM (640x480)
        fx = 580  # Approx focal length in pixels
        fy = 580
        cx = 320  # Image center
        cy = 240
        camera_matrix = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ], dtype=np.float32)
        dist_coeffs = np.zeros((5, 1), dtype=np.float32)  # assume no distortion

    print('Connecting...')
    # Open video stream
    if USE_LAPTOP_CAM:
        cap = cv2.VideoCapture(0)
    else:
        cap = cv2.VideoCapture(URL)

    if not cap.isOpened():
        print("Error: Cannot open video stream")
        exit()
    else:
        print("Success: Starting video stream")

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    found_markers: dict[int, dict[str, float]]

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Failed to grab frame. Retrying in 1 second...")
            time.sleep(1)
            continue

        # Using a greyscale picture, also for faster detection
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Aruco Markers
        corners, ids, _ = detector.detectMarkers(gray_frame)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

            rot_vecs, transl_vecs, _ = aruco.estimatePoseSingleMarkers(
                corners, ARUCO_MARKER_SIZE, camera_matrix, dist_coeffs
            )

            found_markers = {}
            for marker_id, rot_vec, transl_vec in zip(ids, rot_vecs, transl_vecs):
                marker_id = marker_id[0]
                # Draw axis
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rot_vec, transl_vec, 0.05)
                found_markers[marker_id] = {}

                # Distance in meters
                distance = np.linalg.norm(transl_vec[0])
                found_markers[marker_id]['dist'] = distance

                # Rotation matrix → Euler angles
                rot_matrix, _ = cv2.Rodrigues(rot_vec)
                yaw = np.arctan2(rot_matrix[1, 0], rot_matrix[0, 0])  # Rotation around Z-axis
                angle_deg = np.degrees(yaw)
                found_markers[marker_id]['angle'] = angle_deg

            for i, (marker_id, data) in enumerate(found_markers.items()):

                dist_text = f'Distance {data['dist']:.2f}m'
                angle_text=text =f'Angle {data['angle']:.2f}°'
                print('Marker found. ID:', marker_id)
                print('\t', dist_text)
                print('\t', angle_text)

                text = f"ID {marker_id}: Dist={data['dist']:.2f}m, Angle={data['angle']:.1f}"
                position = (10, 30 + i * 30)
                cv2.putText(
                    frame, text, position,
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=0.7,
                    color=(0, 255, 0),  # Green text
                    thickness=2
                )
            print('-------')

        # Show the frames
        cv2.imshow('ESP32 Stream', frame)

        key = cv2.waitKey(1) & 0xFF
        # Screenshot on 's' key
        if key == ord('s'):
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"screenshot{timestamp}.jpg"
            filepath = os.path.join(SCREENSHOT_DIR, filename)
            cv2.imwrite(filepath, frame)
            print(f"Saved {filename}")

        # Exit on 'q' key
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()



if __name__ == '__main__':
    main()
