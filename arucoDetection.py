import cv2
import cv2.aruco as aruco

# Replace with the IP address of your ESP32
ip_address = '192.168.2.108'
url = f'http://{ip_address}:81/stream'

def main():
    # Open video stream
    cap = cv2.VideoCapture(url)

    if not cap.isOpened():
        print("Error: Cannot open video stream")
        exit()
    else:
        print("Success: Starting video stream")

    # Load predefined dictionary
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Detect markers
        corners, ids, _ = detector.detectMarkers(frame)

        # Draw markers and IDs
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

        # Show the frame
        cv2.imshow('ESP32 ArUco Detection', frame)

        # Exit on 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
