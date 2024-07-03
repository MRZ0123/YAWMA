import eventlet
eventlet.monkey_patch()
import paho.mqtt.client as mqtt
import ssl
from flask import Flask, render_template
from flask_socketio import SocketIO


app = Flask(__name__)
app.config['SECRET_KEY'] = 'somesecretkey!idontunderstand'

MQTT_CLIENT_ID = 'server/listener'
MQTT_USERNAME = 'listener'
MQTT_PASSWORD = 'listener_password'
MQTT_BROKER_URL = 'minty'
MQTT_BROKER_PORT = 8883
MQTT_KEEPALIVE = 60
MQTT_TLS_INSECURE = False
MQTT_TLS_VERSION = ssl.PROTOCOL_TLSv1_2
MQTT_TLS_CA_CERTS = './app/ca_certificates/ca.crt'
MQTT_LISTEN_TOPIC = "esp32/bme280/#"

HTTPS_CERTS = ('./app/certs/YAWMA.weather.station.crt',
               './app/certs/YAWMA.weather.station.key')

read_data = {
    'temperature': 0,
    'humidity': 0,
    'pressure': 0,
    'altitude': 0
}

socketio = SocketIO(app)



# HTTPS

@app.route('/')
def home():
    return render_template('weather.html', temperature=read_data['temperature'], humidity=read_data['humidity'], pressure=read_data['pressure'], altitude=read_data['altitude'])

# SocketIO


@socketio.on('message')
def handle_message(data):
    print('received message: ' + data)


@socketio.on('my event')
def handle_my_custom_event(data):
    print('received my event: ' + str(data))

# MQTT


def handle_connect(client: mqtt.Client, userdata, flags, rc, properties=None):
    if rc == 0:
        print("Connected to MQTT Broker!")
        client.subscribe(MQTT_LISTEN_TOPIC, 0)
    else:
        print("Failed to connect, return code %d\n", rc)


def handle_mqtt_message(client, userdata, msg):
    topic = msg.topic.replace("esp32/bme280/", "")
    payload = msg.payload.decode()
    read_data[topic] = payload
    data = dict(
        topic=topic,
        payload=payload
    )
    socketio.emit('mqtt_message', data=data)


def handle_logging(client, userdata, level, buf):
    print("log: ", buf)


def handle_disconnect():
    print("Disconnected from MQTT Broker!")


def main():
    client = mqtt.Client(client_id=MQTT_CLIENT_ID)
    client.on_connect = handle_connect
    client.on_log = handle_logging
    client.on_message = handle_mqtt_message
    client.on_disconnect = handle_disconnect
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    client.tls_set(ca_certs=MQTT_TLS_CA_CERTS, tls_version=MQTT_TLS_VERSION)
    client.tls_insecure_set(MQTT_TLS_INSECURE)
    client.connect(MQTT_BROKER_URL, MQTT_BROKER_PORT, MQTT_KEEPALIVE)

    client.loop_start()


if __name__ == "__main__":
    main()
    socketio.run(app, host='0.0.0.0', port=443, debug=True,
                 use_reloader=False, certfile=HTTPS_CERTS[0], keyfile=HTTPS_CERTS[1])
