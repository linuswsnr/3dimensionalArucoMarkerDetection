import cv2
import numpy as np
import glob
import os
import matplotlib.pyplot as plt

def calibrate_and_plot(
    image_dir: str,
    checkerboard: tuple
):
    """
    Liest alle PNG's in image_dir ein, findet die Checker‑Corners,
    kalibriert die Kamera und plottet alle Bilder untereinander
    mit den gefundenen Ecken.
    """
    nx, ny = checkerboard
    # 1) Objektpunkte vorbereiten (0,0,0), (1,0,0), ...
    objp = np.zeros((nx*ny, 3), np.float32)
    objp[:, :2] = np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)

    objpoints = []
    imgpoints = []
    drawn_imgs = []

    # 2) Bilder einlesen
    images = glob.glob(os.path.join(image_dir, '*.png'))
    if not images:
        raise FileNotFoundError(f"Keine pngs in {image_dir}!")

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Checker‑Ecken finden
        ret, corners = cv2.findChessboardCorners(
            gray,
            (nx, ny),
            cv2.CALIB_CB_ADAPTIVE_THRESH 
            + cv2.CALIB_CB_NORMALIZE_IMAGE
            + cv2.CALIB_CB_FAST_CHECK
        )
        if not ret:
            print(f"Warnung: Ecken in {os.path.basename(fname)} nicht gefunden.")
            # wir plotten trotzdem das Originalbild ohne Ecken
            drawn_imgs.append(img)
            continue

        # Subpixel‑Feinsuche
        corners2 = cv2.cornerSubPix(
            gray, corners, (11,11), (-1,-1), criteria
        )
        objpoints.append(objp)
        imgpoints.append(corners2)

        # Ecken einzeichnen
        img_with_corners = cv2.drawChessboardCorners(
            img, (nx, ny), corners2, ret
        )
        drawn_imgs.append(img_with_corners)

    # 3) Kalibrierung
    img_shape = gray.shape[::-1]  # (width, height)
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, img_shape, None, None
    )

    print("Kalibrierungs-Error (reprojection error):", ret)
    print("Kamera-Matrix:\n", mtx)
    print("Distortion Coefficients:\n", dist.ravel())

    # 4) Mit Matplotlib alle Bilder untereinander plotten
    n = len(drawn_imgs)
    fig, axes = plt.subplots(n, 1, figsize=(6, 4*n))
    if n == 1:
        axes = [axes]  # damit zip unten funktioniert
    for ax, img in zip(axes, drawn_imgs):
        # OpenCV BGR → RGB
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        ax.imshow(img_rgb)
        ax.axis('off')
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    IMAGE_DIR = './espCamHS-Denoised'     # Pfad zu den Bildern
    CHECKER_X = 7              # innere Ecken X
    CHECKER_Y = 10             # innere Ecken Y

    calibrate_and_plot(IMAGE_DIR, (CHECKER_X, CHECKER_Y))
