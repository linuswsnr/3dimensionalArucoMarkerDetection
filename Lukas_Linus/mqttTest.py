import json
import paho.mqtt.client as mqtt

CAMERA_ID = 5  # gewünschte ID

def on_message(client, userdata, msg):
    print(f"Nachricht empfangen auf Topic: {msg.topic}")
    print(f"Payload:\n{msg.payload.decode()}")

# MQTT-Konfiguration
broker = "test.mosquitto.org"
port = 1883
topic = "EZS/beschtegruppe/5"

# Client erstellen und verbinden
client = mqtt.Client()
client.on_message = on_message
client.connect(broker, port, 60)

# Topic abonnieren
client.subscribe(topic)

print(f"Abonniert: {topic} @ {broker}:{port}")
print("Warte auf Nachrichten... (Strg+C zum Beenden)")

# Endlosschleife starten
client.loop_start()

print("skript läuft... (Strg+C zum Beenden)")

# JSON-Datei laden
with open ('Lukas_Linus/marker_positions_rvecs_tvecs.json', 'r') as file:
    marker_positions = json.load(file)

# Finde die Kamera mit passender ID
entry_id_5 = None
for camera_dict in marker_positions:
    if camera_dict['id'] == CAMERA_ID:
        entry_id_5 = camera_dict
        break  # abbrechen sobald gefunden

# Wenn nichts gefunden wurde, abbrechen
if entry_id_5 is None:
    print("Kein Eintrag mit ID 5 gefunden.")
    exit()

broker = "test.mosquitto.org"
port = 1883
topic = "EZS/beschtegruppe/5"

# MQTT senden
client = mqtt.Client()
client.connect(broker, port, 60)
client.publish(topic, json.dumps(entry_id_5))
print(f"Nachricht an Topic '{topic}' gesendet.")

while True:
    pass  # Endlosschleife, um auf Nachrichten zu warten    

