# Flask
from flask import Flask, render_template, Response
from flask_socketio import SocketIO, emit, send
# Video stream
import cv2
# Arm control
import json
import robot_arm as ra
from time import sleep

# Inits
app = Flask(__name__)
video_capture = cv2.VideoCapture(0)
socketio = SocketIO(app)
accept = True

DEBUG = False

# Flask
# Static page with an image, stream by sending images from camera
@app.route('/')
def index():
    diagram_img = "completo.png"
    bootstrap_js = "bootstrap.min.js"
    bootstrap_css = "bootstrap.min.css"
    jquery_js = "jquery.min.js"
    jquery_knob_js = "jquery.knob.min.js"
    socket_io_js = "socket.io.js"
    return render_template('index.html', diagram_img=diagram_img, bootstrap_js=bootstrap_js, bootstrap_css=bootstrap_css, jquery_js=jquery_js, jquery_knob_js=jquery_knob_js, socket_io_js=socket_io_js)

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
    global accept

    jsonMsg = json.loads(message)
    prettyMsg = json.dumps(jsonMsg, indent=4)
    print("\n\n" + prettyMsg)

    try:
        if (accept):
            accept = False
            send("wait", broadcast=True)

            if DEBUG:
                sleep(1)
                accept = True
                send("ok", broacast=True)
                return

            if (jsonMsg['joint'] == 'xyz'):             # semicirculo naranja
                x = int(jsonMsg['x'])
                y = int(jsonMsg['y'])
                z = int(jsonMsg['z'])
                #pitch = int(jsonMsg['pitch'])
                #yaw = int(jsonMsg['yaw'])
                #roll = int(jsonMsg['roll'])
                pitch = 0
                yaw = 0
                roll = 0
                jStr = ra.moveAllToPosition(x, y, z, roll, pitch, yaw, "abs")
                accept = True
                send(jStr, broadcast=True)
            elif (jsonMsg['joint'] == 'relxyz'):        # botones naranja arriba y abajo
                x = int(jsonMsg['x'])
                y = int(jsonMsg['y'])
                z = int(jsonMsg['z'])
                #pitch = int(jsonMsg['pitch'])
                #yaw = int(jsonMsg['yaw'])
                #roll = int(jsonMsg['roll'])
                pitch = 0
                yaw = 0
                roll = 0
                jStr = ra.moveAllToRelativePosition(x, y, z, roll, pitch, yaw)
                accept = True
                send(jStr, broadcast=True)
            elif (jsonMsg['joint'] == 'claw'):
                action = jsonMsg['action']
                if (action == 'open'):
                    ra.openClaw()
                elif (action == 'relax'):
                    ra.relaxClaw()
                elif (action == 'close'):
                    ra.closeClaw()
                accept = True
                send("claw", broadcast=True)
            else:
                joint = int(jsonMsg['joint'])
                jStr = None
                if "action" in jsonMsg:
                    action = int(jsonMsg['action'])
                    jStr = ra.moveSingleJointRelative(joint, action)
                if "angle" in jsonMsg:
                    angle = int(jsonMsg['angle'])
                    jStr = ra.moveSingleJointAbsolute(joint, angle)
                accept = True
                send(jStr, broadcast=True)
        
        else:
            print("SLOW DOWN!")
                
    except AttributeError as ae:
        print("\n\nError!\nArm not initialized!")
        print(ae)
    except KeyError as ke:
        print("\n\nError!\nJoint or action not found!")
        print(ke)

if __name__ == '__main__':
    if not DEBUG:
        ra.initArm()
        ra.initWeb()
    socketio.run(app, host='0.0.0.0', port=5000)
