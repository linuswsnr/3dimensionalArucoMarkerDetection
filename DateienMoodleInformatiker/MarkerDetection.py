import cv2
import cv2.aruco as aruco

class ArucoDetection:
    def __init__(self):
        self.aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, parameters)
        
    def detectInImage(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)

        return corners, ids, rejected, image

    def showImage(self, corners, ids, image):
        aruco.drawDetectedMarkers(image, corners, ids)
        cv2.imshow("ArUco Marker Detection", image)

    def __del__(self):
        cv2.destroyAllWindows()
