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
  arm = WlkataMirobot(portname='COM12')
  arm.set_soft_limit(True)
  arm.set_speed(2000)   
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
  print(f"status: {arm.status}")
  print(f"tool position: {arm.cartesian}")
  print("\n\n\n")

'''
arm.angle.joint1
arm.angle.joint2
arm.angle.joint3
arm.angle.joint4
arm.angle.joint5
arm.angle.joint6
arm.pose.x
arm.pose.y
arm.pose.z
arm.roll
arm.pitch
arm.yaw
arm.valve_pwm
arm.pump_pwm
arm.gripper_pwm
arm.motion_mode
'''

def updateArrays():
  global arm, currentAngles, currentPosition
  arm.get_status()

  currentAngles[1] = arm.angle.joint1
  currentAngles[2] = arm.angle.joint2
  currentAngles[3] = arm.angle.joint3
  currentAngles[4] = arm.angle.joint4
  currentAngles[5] = arm.angle.joint5
  currentAngles[6] = arm.angle.joint6

  currentPosition[X] = arm.pose.x
  currentPosition[Y] = arm.pose.y
  currentPosition[Z] = arm.pose.z
  currentPosition[PITCH] = arm.pitch
  currentPosition[YAW]   = arm.yaw
  currentPosition[ROLL]  = arm.roll

# ----- por posicion -----
def moveAllToPosition(x, y, z, pitch, yaw, roll):
  global arm
  arm.p2p_interpolation(x, y, z, pitch, yaw, roll)      # TO DO: Que pasa si una coordenada es invalida?
  updateArrays()

def moveAllToRelativePosition(x, y, z, pitch, yaw, roll):
  global currentAngles

  newX = currentPosition[X] + (x * 10)
  newY = currentPosition[Y] + (y * 10)
  newZ = currentPosition[Z] + (z * 10)
  newPitch = currentPosition[PITCH] + (pitch * 5)
  newYaw   = currentPosition[YAW]   + (yaw   * 5)
  newRoll  = currentPosition[ROLL]  + (roll  * 5)

  moveAllToPosition(newX, newY, newZ, newPitch, newYaw, newRoll)
  updateArrays()
# ----------

# ----- por angulo -----
def moveSingleJoint(joint, angle):
  global arm
  arm.set_joint_angle({joint:angle})
  updateArrays()

def moveSingleJointRelative(joint, action):
  global currentAngles

  newAngle = currentAngles[joint] + (action * 5)

  if (newAngle < minAngles[joint]):
    newAngle = minAngles[joint]
  if (newAngle > maxAngles[joint]):
    newAngle = maxAngles[joint]

  moveSingleJoint(joint, newAngle)
  updateArrays()
# ----------

def moveAllJoints(angle1, angle2, angle3, angle4, angle5, angle6):
  global arm
  target_angles = {1:angle1, 2:angle2, 3:angle3, 4:angle4, 5:angle5, 6:angle6}
  arm.set_joint_angle(target_angles)
  updateArrays()

def moveAllJointsArray(arr):
  global arm
  target_angles = {1:arr[1], 2:arr[2], 3:arr[3], 4:arr[4], 5:arr[5], 6:arr[6]}
  arm.set_joint_angle(target_angles)
  updateArrays()

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

  # Move one joint
  #moveSingleJoint(BASE_ROTATION_JOINT, -90)
  #sleep(2)

  # Move all joints
  #moveAllJoints(0, 0, -30, 90, -90, -90)
  #sleep(2)

  # Move some joints
  #arm.set_joint_angle({BASE_ROTATION_JOINT:90, BASE_ELEVATION_JOINT:60})
  #sleep(2)

  # Test the claw
  #closeClaw()
  #sleep(2)
  #dropObject()
  #sleep(2)

  #arm.p2p_interpolation(200, 0, 100, 0, 0, -90)
  #printStatus()
  #sleep(3)

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
      arm.p2p_interpolation(x, y, z, pitch, yaw, roll)
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