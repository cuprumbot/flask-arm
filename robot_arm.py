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

currentAngles = [0, 45, 45, 45, 45, 45, 45]
minAngles = [-1, 0, 0, 0, 0, 0, 0]
maxAngles = [-1, 180, 180, 180, 180, 180, 180]

'''
WlkataMirobotTool defines:
  NO_TOOL
  SUCTION_CUP
  GRIPPER
  FLEXIBLE_CLAW
'''

arm = None
print("\nImported robot_arm successfully!\n")

def initArm():
  global arm
  arm = WlkataMirobot()
  arm.set_soft_limit(True)
  arm.set_speed(2000)                                 # (0, 3000]
  arm.set_tool_type(WlkataMirobotTool.SUCTION_CUP)
  print("\nArm initialized successfully!\n")

def printStatus():
  global arm
  arm.get_status()
  print(f"status: {arm.status}")
  print(f"status state: {arm.status.state}")
  print(f"tool position: {arm.cartesian}")

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

def moveSingleJoint(joint, angle):
  global arm
  arm.set_joint_angle({joint:angle})

def moveSingleJointRelative(joint, action):
  global currentAngles

  newAngle = currentAngles[joint] + (action * 5)
  if (newAngle >= minAngles[joint]) and (newAngle <= maxAngles[joint]):
    currentAngles[joint] = newAngle
    moveSingleJoint(joint, newAngle)


def moveAllJoints(angle1, angle2, angle3, angle4, angle5, angle6):
  global arm
  target_angles = {1:angle1, 2:angle2, 3:angle3, 4:angle4, 5:angle5, 6:angle6}
  arm.set_joint_angle(target_angles)

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

def openClaw():
  global arm
  arm.pump_suction()

def closeClaw():
  global arm
  arm.pump_blowing()

def relaxClaw():
  global arm
  arm.pump_off()

def openGripper():
  global arm
  arm.gripper_open()

def closeGripper(distance=20):
  global arm
  arm.set_gripper_spacing(distance)
  arm.gripper_close()

def pickObject():
  global arm
  arm.pump_suction()

def dropObject():
  global arm
  arm.pump_off()

# ---------- testing ----------
if __name__ == "__main__":
  initArm()

  # Home arm
  arm.home()
  printStatus()
  sleep(2)

  # Go to zero
  arm.go_to_zero()
  printStatus()
  sleep(2)

  # Move one joint
  moveSingleJoint(BASE_ROTATION_JOINT, 60)
  sleep(2)

  # Move all joints
  moveAllJoints(30, 30, 30, 30, 30, 30)
  sleep(2)

  # Move some joints
  arm.set_joint_angle({BASE_ROTATION_JOINT:60, BASE_ELEVATION_JOINT:60})
  sleep(2)

  # Open and close claw
  openClaw()
  sleep(2)
  closeClaw(2)
  sleep(2)

  # Open and close gripper
  openGripper()
  sleep(2)
  closeGripper(20)
  sleep(2)

  '''
  Point to Point Interpolation (prefered method)
  x, y, z = 100, -100, 150
  roll, pitch, yaw = 30.0, 0, 45.0
  arm.p2p_interpolation(x, y, z, roll, pitch, yaw)

  arm.linear_interpolation(200, -50, 150)

  ex, ey = (0, -80) 	# target coordinates, in mm(relative to current point)
  radius = 100		# radius, unit in mm
  is_cw = False		# Movement direction True: clockwise, False: counterclockwise
  arm.circular_interpolation(ex, ey, radius, is_cw=is_cw)

  arm.set_door_lift_distance(50)
  arm.door_interpolation(200, -40, 150)
  '''