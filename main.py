import pybullet as p
import time
import math
import pybullet_data


xpos = 0.99
ypos = 0

tray_x = 1.000539607218904
tray_y = 0.35123134884012464



clid = p.connect(p.GUI)
if (clid < 0):
    p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("./data/plane/plane.urdf", [0, 0, -1])

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

sawyerId = p.loadURDF("./data/sawyer_robot/sawyer_description/urdf/sawyer.urdf", [0, 0, 0], [0, 0, 0, 3], useFixedBase=1)  # load sawyer robot
tableId = p.loadURDF("./data/table/table.urdf", [1.1, 0.000000, -0.3],  p.getQuaternionFromEuler([(math.pi / 2), 0, (math.pi / 2)]), useFixedBase=1, flags=8)
objectId = p.loadURDF('./data/random_urdfs/009/009.urdf', [xpos, ypos, 0.05], p.getQuaternionFromEuler([0, 0, 3.14 * 0.7]))
trayId = p.loadURDF("./data/tray/tray.urdf", [tray_x, tray_y, 0], [0, 0, 0, 3],useFixedBase=1)


p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.resetBasePositionAndOrientation(sawyerId, [0, 0, 0], [0, 0, 0, 1])

def palmP(targetP, orientation):
    jointP = [0] * 65
    jointPoses = p.calculateInverseKinematics(sawyerId, 19, targetP, targetOrientation=orientation, jointDamping=[1.1] * 35)
    j = 0
    for i in [3, 4, 8, 9, 10, 11, 13, 16, 21, 22, 23, 26, 27, 28, 30, 31, 32, 35, 36, 37, 39, 40, 41, 44, 45, 46, 48, 49, 50, 53, 54, 55, 58, 61, 64]:
        jointP[i] = jointPoses[j]
        j = j + 1

    for i in range(p.getNumJoints(sawyerId)):
        p.setJointMotorControl2(bodyIndex=sawyerId,
                                jointIndex=i,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=jointP[i],
                                targetVelocity=0,
                                force=50000,
                                positionGain=0.03,
                                velocityGain=1)
    return jointP


# control the lower joint and middle joint of pinky finger, range both [0.17 - 1.57]
def pinkyF(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[21, 26, 22, 27],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of ring finger, range both [0.17 - 1.57]
def ringF(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[30, 35, 31, 36],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of mid finger, range both [0.17 - 1.57]
def midF(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[39, 44, 40, 45],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of index finger, range both [0.17 - 1.57]
def indexF(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[48, 53, 49, 54],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of thumb, range: low [0.17 - 1.57], mid [0.34, 1.5]
def thumb(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[58, 61, 64],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, middle, middle],
                                targetVelocities=[0, 0, 0],
                                forces=[500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1])

final_palmPosition = [tray_x, tray_y, -0.1]
final_orientation = [math.pi, 0, 0]

# # Approach the object
initial_palmPosition = [0.85, -0.05, 0.1]
initial_orientation = [1.2892775535583496, 2.827588395276342, 1.2237756252288818]


# Step 1: Start from a position directly above the object.
palmP(initial_palmPosition, initial_orientation)
for _ in range(100):
    p.stepSimulation()
    time.sleep(0.01)

# Step 2: Approach the object directly from above.
approach_position = [xpos, ypos,  0.09]  # 10cm above the object
palmP(approach_position, initial_orientation)
for _ in range(100):
    p.stepSimulation()
    time.sleep(0.01)

# Step 3: Retract the Arm
retract_position = [0.85, -0.05, 0.09]
palmP(retract_position, initial_orientation)
for _ in range(100):  # Increase the number of steps
    p.stepSimulation()
    time.sleep(0.01)  # Slow down the simulation

# Step 4: Grasp the Object
thumb(1.57, 1.5)
indexF(1.57, 1.57)
midF(1.57, 1.57)
ringF(1.57, 1.57)
pinkyF(1.57, 1.57)
for _ in range(100):  # Increase the number of steps
    p.stepSimulation()
    time.sleep(0.01)  # Slow down the simulation


# Step 5: Lift the Object
lift_position = [xpos, ypos,  0.3]  # Lift it 10cm above the current position
palmP(lift_position, final_orientation)
for _ in range(300):  # Increase the number of steps
    p.stepSimulation()
    time.sleep(0.02)  # Slow down the simulation


# Step 6: Move the Object over the Tray
move_to_tray_position = [tray_x + 0.05, tray_y, -0.03 + 0.1]  # Directly over the tray but 10cm above
palmP(move_to_tray_position, final_orientation)
for _ in range(200):  # Increase the number of steps
    p.stepSimulation()
    time.sleep(0.01)  # Slow down the simulation


# Step 7: Release the Object
thumb(0.17, 0.34)
indexF(0.17, 0.17)
midF(0.17, 0.17)
ringF(0.17, 0.17)
pinkyF(0.17, 0.17)
for _ in range(200):  # Increase the number of steps
    p.stepSimulation()
    time.sleep(0.01)  # Slow down the simulation


p.disconnect()
print("disconnected")