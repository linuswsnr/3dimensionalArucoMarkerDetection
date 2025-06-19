import json
import paho.mqtt.client as mqtt

CAMERA_ID = 5  # gewünschte ID

def on_message(client, userdata, msg):
    print(f"Nachricht empfangen auf Topic: {msg.topic}")
    print(f"Payload:\n{msg.payload.decode()}")

# MQTT-Konfiguration
broker = "test.mosquitto.org"
port = 1883
topic = "EZS/beschtegruppe/4"

# Client erstellen und verbinden
client = mqtt.Client()
client.on_message = on_message
client.connect(broker, port, 60)

# Topic abonnieren
client.subscribe(topic)

client.loop_start()



while True:
    with open ('src/marker_positions_rvecs_tvecs.json', 'r') as file:
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

    topic_5 = "EZS/beschtegruppe/5"


    client.publish(topic_5, json.dumps(entry_id_5))
    print(f"Nachricht an Topic '{topic_5}' gesendet.")