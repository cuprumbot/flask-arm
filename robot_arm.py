from wlkata_mirobot import WlkataMirobot
from wlkata_mirobot import WlkataMirobotTool
from time import sleep

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
PITCH = 3
YAW = 4
ROLL = 5

storageAngles = [-1,    0,  -30,   35,    0,  -85,    0]   
minAngles =     [-1, -110,  -30,  -90, -180, -180, -180]
maxAngles =     [-1,  160,   70,   60,  145,    0,  180]

currentAngles = [-1, 0, -30, 35, 0, -85, 0]
currentPosition = [0, 0, 0, 0, 0, 0]

'''
WlkataMirobotTool defines:
  NO_TOOL
  SUCTION_CUP
  GRIPPER
  FLEXIBLE_CLAW
'''

arm = None

def initArm():
  global arm
  print("Configuring arm...")
  arm = WlkataMirobot(portname='COM13')
  arm.set_soft_limit(True)
  arm.set_speed(3000)   
  #arm.set_tool_type(WlkataMirobotTool.SUCTION_CUP)
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
  print(f"angles: {arm.status.angle}")
  print(f"position: {arm.cartesian}")
  print("\n\n\n")

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
  currentPosition[PITCH] = arm.pose.pitch
  currentPosition[YAW]   = arm.pose.yaw
  currentPosition[ROLL]  = arm.pose.roll

  print(currentAngles[1:])
  print(currentPosition)
  return True

# ----- por posicion -----
def moveAllToPosition(x, y, z, roll, pitch, yaw):
  global arm
  arm.p2p_interpolation(x, y, z, roll, pitch, yaw, wait_ok=True)
  return updateArrays()

def moveAllToRelativePosition(x, y, z, roll, pitch, yaw):
  global currentPosition

  newX = currentPosition[X] + (x * 10)
  newY = currentPosition[Y] + (y * 10)
  newZ = currentPosition[Z] + (z * 10)
  newRoll  = currentPosition[ROLL]  + (roll  * 5)
  newPitch = currentPosition[PITCH] + (pitch * 5)
  newYaw   = currentPosition[YAW]   + (yaw   * 5)

  return moveAllToPosition(newX, newY, newZ, newRoll, newPitch, newYaw)
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
        pitch = int(arr[3])
        yaw = int(arr[4])
        roll = int(arr[5])
      else:
        pitch = 0
        yaw = 0
        roll = 0
      arm.p2p_interpolation(x, y, z, pitch, yaw, roll, wait_ok=True)
      printStatus()
      print("\n\n\n")
    except:
      print("\nerror reading xyz\n\n\n")

  storeInBox()
  sleep(3)

  '''
  Point to Point Interpolation (prefered method)
  x, y, z = 100, -100, 150
  roll, pitch, yaw = 30.0, 0, 45.0
  arm.p2p_interpolation(x, y, z, roll, pitch, yaw)

  arm.linear_interpolation(200, -50, 150)

  ex, ey = (0, -80) 	# target coordinates, in mm(relative to current point)
  radius = 100		    # radius, unit in mm
  is_cw = False		    # Movement direction True: clockwise, False: counterclockwise
  arm.circular_interpolation(ex, ey, radius, is_cw=is_cw)

  arm.set_door_lift_distance(50)
  arm.door_interpolation(200, -40, 150)
  '''