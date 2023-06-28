# Flask
from flask import Flask, render_template, Response
from flask_socketio import SocketIO, emit
# Video stream
import cv2
# Arm control
import json
import robot_arm as ra

# Inits
app = Flask(__name__)
video_capture = cv2.VideoCapture(0)
socketio = SocketIO(app)
lastTimestamp = 0

# Flask
# Static page with an image, stream by sending images from camera
@app.route('/')
def index():
    diagram_img = "completo.png"
    return render_template('index.html', diagram_img=diagram_img)

def generate_frames():
    while True:
        success, frame = video_capture.read()
        if not success:
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# Websocket
# Debug, control arm
@socketio.on('connect')
def handle_connect():
    print('\nClient connected\n')

@socketio.on('disconnect')
def handle_disconnect():
    print('\nClient disconnected\n')

@socketio.on('message')
def handle_message(message):
    jsonMsg = json.loads(message)
    prettyMsg = json.dumps(jsonMsg, indent=4)
    print("\n\n\n" + prettyMsg)

    try:
        timestamp = int(jsonMsg['ts'])
        joint = int(jsonMsg['joint'])
        action = int(jsonMsg['action'])
        if (timestamp > lastTimestamp):
            ra.moveSingleJointRelative(joint, action)
    except AttributeError as ae:
        print("\n\n\nError!\nArm not initialized!")
        print(ae)
    except KeyError as ke:
        print("\n\n\nError!\nJoint or action not found!")
        print(ke)

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0')