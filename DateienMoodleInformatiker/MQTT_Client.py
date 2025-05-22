import paho.mqtt.client as pahoclient 
import ssl

# https://www.hivemq.com/blog/implementing-mqtt-in-python/
# https://www.emqx.com/en/blog/how-to-use-mqtt-in-python
class MQTT_Client:
    def __init__(self, clientID:str, username:str = None, password:str = None, tlsInsecure:bool = True, brokerUrl:str = "127.0.0.1", brokerPort:str = 1883, activeTLS:bool = False):

        # when no version of the protocoll is set it takes MQTTv311 as default
        self.client = pahoclient.Client(client_id=clientID, userdata= None, protocol=pahoclient.MQTTv31, reconnect_on_failure=True)

        if activeTLS:
            self.client.tls_set(tls_version=ssl.PROTOCOL_TLSv1_2)
            self.client.tls_insecure_set(tlsInsecure)

        if password and username:
            self.client.username_pw_set(username=username, password=password)

        self.brokerUrl = brokerUrl
        self.brokerPort = brokerPort

    def connect(self) -> pahoclient.MQTTMessage:
        self.client.connect(self.brokerUrl, self.brokerPort)

    def setTopic(self, topic:str):
        self.topic = topic

    def publish(self, msg, topic:str = None) -> pahoclient.MQTTMessageInfo:
        if not self.client.is_connected():
            return

        if topic != None:
            return self.client.publish(topic, msg)
        elif self.topic != None:
            return self.client.publish(self.topic, msg)

    def subscribe(self, topic:str, subscribeFunktion = None):
        # functionOnMessage should be a function 

        if not self.client.is_connected():
            return
        
        if topic != None:
            return self.client.subscribe(topic)
        elif self.topic != None:
            return self.client.subscribe(self.topic)
        
        if subscribeFunktion:
            self.client.on_message = subscribeFunktion

    def disconnect(self):
        self.client.disconnect()

    def startLoop(self):
        self.client.loop_start()

    def stopLoop(self):
        self.client.loop_stop()

    def loopInternaty(self):
        self.client.loop_forever()

    def isConnected(self) -> bool:
        return self.client.is_connected()
