import numpy as np  # Importiert NumPy für mathematische Operationen mit Arrays
import matplotlib.pyplot as plt  # Zum Erstellen von Diagrammen und Plots
import cv2  # OpenCV für Bildverarbeitung und Funktionen wie Rodrigues-Transformation
from collections import defaultdict, deque  # defaultdict für automatische Listeninitialisierung, deque für effiziente Warteschlange (Queue)

# JSON-Daten mit Zeitstempel und Pose-Information.
# Jede Kamera (id) beobachtet Marker (detected_id) mit einer Pose bestehend aus rvecs (Rotation) und tvecs (Translation).
data = [
    {
        "id": 1,
        "Others": [
            {
                "detected_id": "6",
                "Position": [
                    { "rvecs": [0.0, 0.0, 0.0] },  # Rotationsvektor: keine Rotation
                    { "tvecs": [10.0, 10.0, 14.0] }  # Translationsvektor in Zentimeter
                ]
            },
            {
                "detected_id": "5",
                "Position": [
                    { "rvecs": [0.0, 0.0, 0.0] },
                    { "tvecs": [40.0, 10.0, 14.0] }
                ]
            },
        ],
        "time": "2025-06-11T12:09:29.146623"
    },
    {
        "id": 2,
        "Others": [
            {
                "detected_id": "6",
                "Position": [
                    { "rvecs": [0.0, 0.0, 0.0] },
                    { "tvecs": [16.0, 50.0, 90.0] }
                ]
            },
            {
                "detected_id": "2",
                "Position": [
                    { "rvecs": [0.0, 0.0, 0.0] },
                    { "tvecs": [20.0, 60.0, 90.0] }
                ]
            },
            {
                "detected_id": "",
                "Position": [ { "rvecs": [] }, { "tvecs": [] } ]  # Leere Erkennung, wird ignoriert
            }
        ],
        "time": "2025-06-11T12:09:29.146623"
    },
    {
        "id": 3,
        "Others": [
            {
                "detected_id": "6",
                "Position": [
                    { "rvecs": [0.0, 0.0, 0.0] },
                    { "tvecs": [50.0, 10.0, 14.0] }
                ]
            }
        ],
        "time": "2025-06-11T12:09:29.146623"
    },
    {
        "id": 4,
        "Others": [
            {
                "detected_id": "",
                "Position": [ { "rvecs": [] }, { "tvecs": [] } ]  # Keine gültige Detektion
            }
        ],
        "time": "2025-06-11T12:09:29.146623"
    },
    {
        "id": 5,
        "Others": [
            {
                "detected_id": "6",
                "Position": [
                    { "rvecs": [0.0, 0.0, 0.0] },
                    { "tvecs": [80.0, 20.0, 14.0] }
                ]
            },
            {
                "detected_id": "0",
                "Position": [
                    { "rvecs": [0.0, 0.0, 0.0] },
                    { "tvecs": [50.0, 20.0, 14.0] }
                ]
            }
        ],
        "time": "2025-06-11T12:09:29.146623"
    },
    {
        "id": 6,
        "Others": [
            {
                "detected_id": "9",
                "Position": [
                    { "rvecs": [0.0, 0.0, 0.0] },
                    { "tvecs": [60.0, 60.0, 14.0] }
                ]
            },
            {
                "detected_id": "6",
                "Position": [
                    { "rvecs": [0.0, 0.0, 0.0] },
                    { "tvecs": [20.0, 20.0, 14.0] }
                ]
            }
        ],
        "time": "2025-06-11T12:09:29.146623"
    }
]

# Funktion zur Berechnung der Kameraposition im Weltkoordinatensystem aus Sicht eines bekannten Markers
def get_camera_position_from_marker(rvec, tvec, marker_global_pos):
    rvec = np.array(rvec, dtype=np.float64)  # Konvertiere Rotationsvektor in NumPy-Array
    tvec = np.array(tvec, dtype=np.float64).reshape((3, 1)) / 100.0  # Umwandlung in Meter und als Spaltenvektor
    R, _ = cv2.Rodrigues(rvec)  # Berechne Rotationsmatrix aus dem Rotationsvektor
    cam_pos = marker_global_pos.reshape((3, 1)) - R @ tvec  # Invertierte Transformation: Marker - (R * t)
    return cam_pos  # Gibt die geschätzte Position der Kamera zurück

# Initialisiere Datenstrukturen zur Speicherung der rekonstruierten Positionen
camera_positions = {}  # Kamera-ID -> 3D-Position (x, y, z)
marker_positions = {}  # Marker-ID -> 3D-Position (x, y, z)
edges = defaultdict(list)  # Verbindungsgraph: jeder Knoten hat eine Liste von (Nachbar, Rotation, Translation)

# Durchlaufe die Daten und baue den Graph der Verbindungen zwischen Kameras und Markern
for entry in data:
    cam_id = entry["id"]
    for other in entry["Others"]:
        rvecs = other["Position"][0]["rvecs"]
        tvecs = other["Position"][1]["tvecs"]
        detected_id = other["detected_id"]
        # Prüfe, ob gültige rvecs/tvecs und eine erkannte ID vorliegen
        if rvecs and tvecs and detected_id != "":
            try:
                rvecs_np = np.array(rvecs, dtype=np.float64)  # Konvertiere Rotationsvektor
                tvecs_np = np.array(tvecs, dtype=np.float64)  # Konvertiere Translationsvektor
                # Füge Verbindung von Kamera zu Marker hinzu
                edges[f"C{cam_id}"].append((f"M{detected_id}", rvecs_np, tvecs_np))
                # Und umgekehrt von Marker zu Kamera
                edges[f"M{detected_id}"].append((f"C{cam_id}", rvecs_np, tvecs_np))
            except:
                continue  # Falls Umwandlung fehlschlägt, ignoriere den Eintrag

# Setze den Ursprung der Weltkoordinaten: Marker 0 liegt bei (0, 0, 0)
marker_positions["M0"] = np.array([0.0, 0.0, 0.0])
queue = deque(["M0"])  # Initialisiere die Warteschlange mit Marker 0 (BFS)

# Breitensuche (Breadth-First Search), um sukzessive Kamera- und Markerpositionen zu berechnen
while queue:
    current = queue.popleft()  # Nächstes Element aus der Warteschlange holen
    if current.startswith("M"):  # Wenn aktueller Knoten ein Marker ist
        marker_id = current
        for neighbor, rvecs, tvecs in edges[marker_id]:
            if neighbor not in camera_positions:  # Falls Kamera noch nicht bekannt ist
                cam_pos = get_camera_position_from_marker(rvecs, tvecs, marker_positions[marker_id])
                camera_positions[neighbor] = cam_pos  # Speichere Kameraposition
                queue.append(neighbor)  # Füge Kamera zur Warteschlange hinzu
    elif current.startswith("C"):  # Wenn aktueller Knoten eine Kamera ist
        cam_id = current
        for neighbor, rvecs, tvecs in edges[cam_id]:
            if neighbor not in marker_positions:  # Falls Marker noch nicht bekannt ist
                cam_pos = camera_positions[cam_id]
                rvec = np.array(rvecs, dtype=np.float64)
                tvec = np.array(tvecs, dtype=np.float64).reshape((3, 1)) / 100.0
                R, _ = cv2.Rodrigues(rvec)
                marker_pos = cam_pos.reshape((3, 1)) + R @ tvec  # Vorwärtstransformation von Kamera zu Marker
                marker_positions[neighbor] = marker_pos.flatten()  # Speichere Markerposition
                queue.append(neighbor)  # Füge Marker zur Warteschlange hinzu

# Plotten der rekonstruierten Kamera- und Markerpositionen in 2D (X-Y-Ebene)
plt.figure(figsize=(8, 8))
for cam_id, pos in camera_positions.items():
    plt.plot(pos[0], pos[1], 'ro')  # Kamera als roter Punkt
    plt.text(pos[0] + 0.01, pos[1] + 0.01, cam_id, color='red')  # Beschriftung

for marker_id, pos in marker_positions.items():
    plt.plot(pos[0], pos[1], 'bs')  # Marker als blauer Punkt
    plt.text(pos[0] + 0.01, pos[1] + 0.01, marker_id, color='blue')  # Beschriftung

plt.grid(True)
plt.xlabel("X [m]")  # Achsenbeschriftung X
plt.ylabel("Y [m]")  # Achsenbeschriftung Y
plt.title("Rekonstruiertes Marker-Kamera-Netzwerk")  # Titel des Plots
plt.axis("equal")  # Gleiche Skalierung beider Achsen
plt.show()  # Plot anzeigen
