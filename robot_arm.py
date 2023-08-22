from wlkata_mirobot import WlkataMirobot
from wlkata_mirobot import WlkataMirobotTool
from time import sleep
import math
import json

# constants
BASE_ROTATION_JOINT = 1
BASE_ELEVATION_JOINT = 2
ELBOW_JOINT = 3
WRIST_ROTATION_JOINT = 4
WRIST_EXTENSION_JOINT = 5
TOOL_ROTATION_JOINT = 6

# more constants
X = 0
Y = 1
Z = 2
ROLL = 3
PITCH = 4
YAW = 5

storageAngles = [-1,    0,  -30,   35,    0,  -85,    0]   
minAngles =     [-1, -110,  -30,  -90, -180, -180, -180]
maxAngles =     [-1,  160,   70,   60,  145,    0,  180]

currentAngles = [-1, 0, -30, 35, 0, 0, 0]
currentPosition = [0, 0, 0, 0, 0, 0]

arm = None

def initArm():
  global arm
  print("Configuring arm...")
  arm = WlkataMirobot(portname='COM13')
  arm.set_soft_limit(True)
  arm.set_speed(3000)   
  # arm.set_tool_type(WlkataMirobotTool.SUCTION_CUP)
  print("Arm config ready!")

  print("Homing...")
  sleep(1)
  arm.home()
  sleep(1)
  print("Homed successfully!")

  print("Arm is ready!")

def initWeb():
  moveAllJointsArray(currentAngles)

def storeInBox():
  moveAllJointsArray(storageAngles)

def printStatus():
  arm.get_status()
  print("\n--------------------")
  print(f"angles: {arm.status.angle}")
  print(f"position: {arm.cartesian}")
  print("--------------------\n")

def updateArrays():
  global arm, currentAngles, currentPosition
  printStatus()

  currentAngles[BASE_ROTATION_JOINT]   = arm.angle.joint4   # rotacion base
  currentAngles[BASE_ELEVATION_JOINT]  = arm.angle.joint5   # elevacion
  currentAngles[ELBOW_JOINT]           = arm.angle.joint6   # codo
  currentAngles[WRIST_ROTATION_JOINT]  = arm.angle.joint1   # rotacion muneca
  currentAngles[WRIST_EXTENSION_JOINT] = arm.angle.joint2   # elevacion muneca
  currentAngles[TOOL_ROTATION_JOINT]   = arm.angle.joint3   # rotacion herramienta

  currentPosition[X] = arm.pose.x
  currentPosition[Y] = arm.pose.y
  currentPosition[Z] = arm.pose.z
  currentPosition[ROLL]  = arm.pose.roll
  currentPosition[PITCH] = arm.pose.pitch
  currentPosition[YAW]   = arm.pose.yaw

  print("\n--------------------")
  print("angulos: ", currentAngles[1:])
  print("xyz rpy: ", currentPosition)
  print("--------------------\n")

  # FIND ME
  '''
  print("\n\nDEBUG YAW ARM")
  resta = currentAngles[BASE_ROTATION_JOINT] - currentPosition[YAW]
  print("base %i - yaw %i = resta %i" % (currentAngles[BASE_ROTATION_JOINT], currentPosition[YAW], resta))
  print("some modulos %i %i %i" % (resta, (resta+180)%180, (resta+360)%180))
  '''

  j = {
    "status" : "ok",
    "x" : currentPosition[X],
    "y" : currentPosition[Y],
    "z" : currentPosition[Z],
    "roll" : currentPosition[ROLL],
    "pitch" : currentPosition[PITCH],
    "yaw" : currentPosition[YAW],
    "j1" : currentAngles[BASE_ROTATION_JOINT],
    "j2" : currentAngles[BASE_ELEVATION_JOINT],
    "j3" : currentAngles[ELBOW_JOINT],
    "j4" : currentAngles[WRIST_ROTATION_JOINT],
    "j5" : currentAngles[WRIST_EXTENSION_JOINT],
    "j6" : currentAngles[TOOL_ROTATION_JOINT]
  }
  jStr = json.dumps(j)
  return jStr

# ----- por posicion -----
def moveAllToPosition(x, y, z, roll, pitch, yaw, tipo):
  global arm

  # We cannot use the base rotation angle
  # We must use the resulting X and Y coords to get the resulting angle
  currX = currentPosition[X]
  currY = currentPosition[Y]
  if (currX == 0):    # Division by zero
    currX = 1
  currAngle = math.degrees(math.atan(currY/currX))
  if (currX < 0):
    if (currY > 0):
      currAngle = currAngle + 180
    elif (y < 0):
      currAngle = currAngle - 180
  currBase = currAngle

  # This relation must hold true at the end
  currYaw = currentPosition[YAW]
  currRel = currBase - currYaw

  # Use the new X and Y coords to get the resulting angle
  if (x == 0):      # Division by zero
    x = 1
  angle = math.degrees(math.atan(y/x))
  if (x < 0):
    if (y > 0):
      angle = angle + 180
    elif (y < 0):
      angle = angle - 180

  # The relation between angles holds true
  calcYaw = angle - currRel

  # FIND ME
  print("\n--------------------")
  print("DEBUG ANGLE PREDICTION")
  print("BASE ANGLE: ", angle)
  print("CALC YAW:   ", calcYaw)
  print(f'original   {currBase} - {currYaw} = {currRel}')
  print(f'nuevos     {angle} - {calcYaw} = {angle - calcYaw}')
  print("--------------------\n")

  # Don't change these angles             # TO DO: remove these from p2p call
  newRoll  = currentPosition[ROLL]
  newPitch = currentPosition[PITCH]

  if (tipo == "rel"):
    newYaw = currentPosition[YAW]
  else:
    newYaw = calcYaw

  # Use this if the 'chicken mode' is unpopular
  # newYaw = calcYaw

  arm.p2p_interpolation(x, y, z, newRoll, newPitch, newYaw, wait_ok=True)
  return updateArrays()

def moveAllToRelativePosition(x, y, z, roll, pitch, yaw):
  global currentPosition

  newX = currentPosition[X] + (x * 10)
  newY = currentPosition[Y] + (y * 10)
  newZ = currentPosition[Z] + (z * 10)
  newRoll  = currentPosition[ROLL]  + (roll  * 5)
  newPitch = currentPosition[PITCH] + (pitch * 5)
  newYaw   = currentPosition[YAW]   + (yaw   * 5)

  # newRoll, newPitch, newYaw are irrelevant
  return moveAllToPosition(newX, newY, newZ, newRoll, newPitch, newYaw, "rel")
# ----------

# ----- por angulo -----
def moveSingleJoint(joint, angle):
  global arm
  arm.set_joint_angle({joint:angle}, wait_ok=True)
  return updateArrays()

def moveSingleJointRelative(joint, action):
  global currentAngles

  newAngle = currentAngles[joint] + (action * 5)

  if (newAngle < minAngles[joint]):
    newAngle = minAngles[joint]
  if (newAngle > maxAngles[joint]):
    newAngle = maxAngles[joint]

  return moveSingleJoint(joint, newAngle)

def moveSingleJointAbsolute(joint, angle):
  global currentAngles

  if (angle < minAngles[joint]):
    angle = minAngles[joint]
  if (angle > maxAngles[joint]):
    angle = maxAngles[joint]

  return moveSingleJoint(joint, angle)
# ----------

def moveAllJoints(angle1, angle2, angle3, angle4, angle5, angle6):
  global arm
  target_angles = {1:angle1, 2:angle2, 3:angle3, 4:angle4, 5:angle5, 6:angle6}
  arm.set_joint_angle(target_angles, wait_ok=True)
  return updateArrays()

def moveAllJointsArray(arr):
  global arm
  target_angles = {1:arr[1], 2:arr[2], 3:arr[3], 4:arr[4], 5:arr[5], 6:arr[6]}
  arm.set_joint_angle(target_angles, wait_ok=True)
  return updateArrays()

def moveBaseRotation(angle):
  moveSingleJoint(1, angle)

def moveBaseElevation(angle):
  moveSingleJoint(2, angle)

def moveElbow(angle):
  moveSingleJoint(3, angle)

def moveWristRotation(angle):
  moveSingleJoint(4, angle)

def moveWristExtension(angle):
  moveSingleJoint(5, angle)

def moveToolRotation(angle):
  moveSingleJoint(6, angle)

# ----- garra -----
def openClaw():
  global arm
  arm.pump_suction()

def relaxClaw():
  global arm
  arm.pump_off()

def closeClaw():
  global arm
  arm.pump_blowing()
# ----------

# ----- garra servo -----
def openGripper():
  global arm
  arm.gripper_open()

def closeGripper(distance=20):
  global arm
  arm.set_gripper_spacing(distance)
  arm.gripper_close()
# ----------

# ----- ventosa -----
def pickObject():
  global arm
  arm.pump_suction()

def dropObject():
  global arm
  arm.pump_off()
# ----------

# ---------- testing ----------
if __name__ == "__main__":
  initArm()

  while True:
    str = input("x y z: ")
    if (str == "q"):
      break

    try:
      arr = str.split()
      x = int(arr[0])
      y = int(arr[1])
      z = int(arr[2])
      if (len(arr) > 3):
        print("got roll+pitch+yaw")
        roll = int(arr[3])
        pitch = int(arr[4])
        yaw = int(arr[5])
      else:
        roll = 0
        pitch = 0
        yaw = 0
        
      arm.p2p_interpolation(x, y, z, roll, pitch, yaw, wait_ok=True)
      printStatus()
      print("\n\n\n")
    except:
      print("error reading xyz\n\n\n")

  storeInBox()
  sleep(3)

  '''
  Point to Point Interpolation (prefered method)
  x, y, z = 100, -100, 150
  roll, pitch, yaw = 30.0, 0, 45.0
  arm.p2p_interpolation(x, y, z, roll, pitch, yaw)

  arm.linear_interpolation(200, -50, 150)

  ex, ey = (0, -80)     # target coordinates, in mm(relative to current point)
  radius = 100          # radius, unit in mm
  is_cw = False         # Movement direction True: clockwise, False: counterclockwise
  arm.circular_interpolation(ex, ey, radius, is_cw=is_cw)

  arm.set_door_lift_distance(50)
  arm.door_interpolation(200, -40, 150)
  '''