import cv2

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

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Show the frame
        cv2.imshow('ESP32 Stream', frame)

        # Exit on 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()



if __name__ == '__main__':
    main() 