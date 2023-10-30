import pybullet as p
import time
import math
import random
import pybullet_data
from datetime import datetime
import numpy as np

######################################################### Simulation Setup ############################################################################

clid = p.connect(p.GUI)
if (clid < 0):
    p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("plane.urdf", [0, 0, -1])

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
sawyerId = p.loadURDF("./sawyer_robot/sawyer_description/urdf/sawyer.urdf", [0, 0, 0], [0, 0, 0, 3],
                      useFixedBase=1)  # load sawyer robot


tableId = p.loadURDF("./table/table.urdf", [1.1, 0.000000, -0.3],
                     p.getQuaternionFromEuler([(math.pi / 2), 0, (math.pi / 2)]), useFixedBase=1, flags=8)


######################################################### Load Object Here!!!!#############################################################################

# load object, change file name to load different objects
# p.loadURDF(finlename, position([X,Y,Z]), orientation([a,b,c,d]))
# Example:
# objectId = p.loadURDF("random_urdfs/001/001.urdf", [1.25 ,0.25,-0.1], p.getQuaternionFromEuler([0,0,1.56])) # pi*0.5

# xpos = 0.95
# ypos = 0
# ang = 3.14 * 0.5
# orn = p.getQuaternionFromEuler([0, 0, ang])

# object_path ='random_urdfs/009/009.urdf'
# objectId = p.loadURDF(object_path, xpos, ypos, -0.03, orn[0], orn[1], orn[2], orn[3])
xpos = 0.95
ypos = 0
ang = 3.14 * 0.5
orn = p.getQuaternionFromEuler([0, 0, ang])
object_path ='random_urdfs/009/009.urdf'
objectId = p.loadURDF(object_path, xpos, ypos, -0.03, orn[0], orn[1], orn[2], orn[3])


tray_x = 1.000539607218904
tray_y = 0.35123134884012464
trayId = p.loadURDF("./tray/tray.urdf", [tray_x, tray_y, 0], [0, 0, 0, 3],useFixedBase=1)


###########################################################################################################################################################


p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.resetBasePositionAndOrientation(sawyerId, [0, 0, 0], [0, 0, 0, 1])

sawyerEndEffectorIndex = 16
numJoints = p.getNumJoints(sawyerId)  # 65 with ar10 hand

# useRealTimeSimulation = 0
# p.setRealTimeSimulation(useRealTimeSimulation)
# p.stepSimulation()
# all R joints in robot
js = [3, 4, 8, 9, 10, 11, 13, 16, 21, 22, 23, 26, 27, 28, 30, 31, 32, 35, 36, 37, 39, 40, 41, 44, 45, 46, 48, 49, 50,
      53, 54, 55, 58, 61, 64]
# lower limits for null space
ll = [-3.0503, -5.1477, -3.8183, -3.0514, -3.0514, -2.9842, -2.9842, -4.7104, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17,
      0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.85, 0.34,
      0.17]
# upper limits for null space
ul = [3.0503, 0.9559, 2.2824, 3.0514, 3.0514, 2.9842, 2.9842, 4.7104, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57,
      0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 2.15, 1.5, 1.5]
# joint ranges for null space
jr = [0, 0, 0, 0, 0, 0, 0, 0, 1.4, 1.4, 1.4, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4,
      0, 1.4, 1.4, 0, 1.3, 1.16, 1.33]
# restposes for null space
rp = [0] * 35
# joint damping coefficents
jd = [1.1] * 35


######################################################### Inverse Kinematics Function ##########################################################################

# Finger tip ID: index:51, mid:42, ring: 33, pinky:24, thumb 62
# Palm ID: 20
# move palm (center point) to reach the target postion and orientation
# input: targetP --> target postion
#        orientation --> target orientation of the palm
# output: joint positons of all joints in the robot
#         control joint to correspond joint position
def palmP(targetP, orientation):
    jointP = [0] * 65
    jointPoses = p.calculateInverseKinematics(sawyerId, 19, targetP, targetOrientation=orientation, jointDamping=jd)
    j = 0
    for i in js:
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


######################################################### Hand Direct Control Functions ##########################################################################

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


######################################################### detected information ##########################################################################
'''
19 b'palm'
20 b'rb1'
21 b'finger1.1a'
22 b'finger1.1b'
23 b'finger1.1c'
24 b'fingertip1'
25 b'rb2'
26 b'finger1.2a'
27 b'finger1.2b'
28 b'finger1.2c'
29 b'rb3'
30 b'finger2.1a'
31 b'finger2.1b'
32 b'finger2.1c'
33 b'fingertip2'
34 b'rb4'
35 b'finger2.2a'
36 b'finger2.2b'
37 b'finger2.2c'
38 b'rb5'
39 b'finger3.1a'
40 b'finger3.1b'
41 b'finger3.1c'
42 b'fingertip3'
43 b'rb6'
44 b'finger3.2a'
45 b'finger3.2b'
46 b'finger3.2c'
47 b'rb7'
48 b'finger4.1a'
49 b'finger4.1b'
50 b'finger4.1c'
51 b'fingertip4'
52 b'rb8'
53 b'finger4.2a'
54 b'finger4.2b'
55 b'finger4.2c'
56 b'rb9'
57 b'rb10'
58 b'thumb1'
59 b'thumb2'
60 b'rb11'
61 b'thumb3.1'
62 b'thumbtip'
63 b'rb12'
64 b'thumb3.2'
hand = [21, 22, 23, 26, 27, 28, 30, 31, 32, 35, 36 ,37, 39, 40, 41, 44, 45, 46, 48, 49, 50, 53, 54, 55, 58, 61, 64]
[0.2196998776260993, 0.9841056922424084, 0.16991782178342238, 0.21967883521345558, 0.9846229478397389, 0.1699958046620013, 0.5711534611694058, 0.5914229523765463, 0.16999954970542672, 0.573730600144428, 0.5902151809391006, 0.17000660753266578, 0.9359158730554522, 0.265116872922352, 0.170003190706592, 0.9361250259528252, 0.2652466938834658, 0.17003347470289248, 0.9068051254489781, 0.2490975329073341, 0.17008149880963058, 0.9066050389575453, 0.2502858674912193, 0.16999999999999976, 1.5698468053021237, 0.34006621802344955, 0.3400508342876441]

'''


def info():
    palmContact = []
    thumbContact = []
    indexContact = []
    midContact = []
    ringContact = []
    pinkyContact = []
    palmLinks = [19, 20, 25, 29, 34, 38, 43, 47, 52, 56, 57]
    thumbLinks = [58, 59, 60, 61, 62, 63, 64]
    indexLinks = [48, 49, 50, 51, 53, 54, 55]
    middleLinks = [39, 40, 41, 42, 44, 45, 46]
    ringLinks = [30, 31, 32, 33, 35, 36, 37]
    pinkyLinks = [21, 22, 23, 24, 26, 27, 28]

    contact = p.getContactPoints(sawyerId, objectId)  # pubullet quick guide
    nums = len(contact)
    if (nums == 0):
        print("There are no contact points")
        return [], [], [], [], [], []

    for i in range(nums):
        temp = []
        if (contact[i][3] in palmLinks):
            temp.append(contact[i][3])
            temp.append(contact[i][6])
            temp.append(contact[i][9])
            temp.append(contact[i][10])
            temp.append(contact[i][11])
            temp.append(contact[i][12])
            temp.append(contact[i][13])
            palmContact.append(temp)

        if (contact[i][3] in thumbLinks):
            temp.append(contact[i][3])
            temp.append(contact[i][6])
            temp.append(contact[i][9])
            temp.append(contact[i][10])
            temp.append(contact[i][11])
            temp.append(contact[i][12])
            temp.append(contact[i][13])
            thumbContact.append(temp)

        if (contact[i][3] in indexLinks):
            temp.append(contact[i][3])
            temp.append(contact[i][6])
            temp.append(contact[i][9])
            temp.append(contact[i][10])
            temp.append(contact[i][11])
            temp.append(contact[i][12])
            temp.append(contact[i][13])
            indexContact.append(temp)

        if (contact[i][3] in middleLinks):
            temp.append(contact[i][3])
            temp.append(contact[i][6])
            temp.append(contact[i][9])
            temp.append(contact[i][10])
            temp.append(contact[i][11])
            temp.append(contact[i][12])
            temp.append(contact[i][13])
            midContact.append(temp)

        if (contact[i][3] in ringLinks):
            temp.append(contact[i][3])
            temp.append(contact[i][6])
            temp.append(contact[i][9])
            temp.append(contact[i][10])
            temp.append(contact[i][11])
            temp.append(contact[i][12])
            temp.append(contact[i][13])
            ringContact.append(temp)

        if (contact[i][3] in pinkyLinks):
            temp.append(contact[i][3])
            temp.append(contact[i][6])
            temp.append(contact[i][9])
            temp.append(contact[i][10])
            temp.append(contact[i][11])
            temp.append(contact[i][12])
            temp.append(contact[i][13])
            pinkyContact.append(temp)

    return palmContact, thumbContact, indexContact, midContact, ringContact, pinkyContact


######################################################### Simulation ##########################################################################
currentP = [0] * 65
k = 0

handInitial = [0.1745631085544039, 0.1782199842342839, 0.17000049054390223, 0.17574630988469556, 0.16997785827638068, 0.2097479613791115, 0.32065248846823124, 0.2139570666281873, 0.1699361852141938, 0.31638080766518206, 0.20761372512381612, 0.16999919477480716, 0.5718429771568684, 1.4576079422113783, 0.18251364922936328, 0.5778739348713193, 1.4014264228848239, 0.1699980975645333, 0.7752309113564865, 1.5285909347630509, 0.17018867728755527, 0.7757628503042919, 1.5308398766139475, 0.17013692296726954, 1.2036168509325875, 0.883009649447393, 0.8835241500002111]
orientation =  [1.2892775535583496, 2.827588395276342, 1.2237756252288818]
palmPosition =  [0.9225473534199409, -0.12, -0.10]


# final_palmPosition =
# final_orientation =
# import pdb; pdb.set_trace()
final_palmPosition = [tray_x, tray_y, -0.1]
final_orientation = [math.pi, 0, 0]
# Approach the object

# ... [rest of the code remains unchanged]

# # Approach the object
initial_palmPosition = [0.85, -0.05, 0.1]
# initial_orientation = [1.2892775535583496, 2.827588395276342, 1.2237756252288818]

# palmPosition = [palmPosition[0]-0.1,palmPosition[1]-0.05,palmPosition[2]]

# # Step 1: Approach the Object
# approach_position = [xpos, ypos, -0.03 + 0.10]  # This should be closer to the object's Z-coordinate, but slightly above it.
# palmP(approach_position, final_orientation)
# for _ in range(200):  # Increase the number of steps
#     p.stepSimulation()
#     time.sleep(0.01)  # Slow down the simulation
######################################################### Simulation ##########################################################################

# ...

# Approach the object directly from above with wrist orientation
# Approach the object directly from above with wrist orientation
# Approach the object directly from above with wrist orientation
# Adjust the xpos to move the arm to the left

# initial_palmPosition = [xpos, ypos, 0.3]  # Directly above the object, 30cm higher
initial_orientation = p.getQuaternionFromEuler([math.pi, 0, 0])  # Orientation facing downwards

# Step 1: Start from a position directly above the object.
palmP(initial_palmPosition, initial_orientation)
for _ in range(200):
    p.stepSimulation()
    time.sleep(0.01)
# xpos = 1.2892775535583496

# Step 2: Approach the object directly from above.
approach_position = [xpos, ypos,  0.10]  # 10cm above the object
palmP(approach_position, initial_orientation)
for _ in range(200):
    p.stepSimulation()
    time.sleep(0.01)


# Step 6: Retract the Arm
retract_position = [0.85, -0.05, 0.1]
palmP(retract_position, initial_orientation)
for _ in range(200):  # Increase the number of steps
    p.stepSimulation()
    time.sleep(0.01)  # Slow down the simulation

# Step 2: Grasp the Object
thumb(1.57, 1.5)
indexF(1.57, 1.57)
midF(1.57, 1.57)
ringF(1.57, 1.57)
pinkyF(1.57, 1.57)
for _ in range(200):  # Increase the number of steps
    p.stepSimulation()
    time.sleep(0.01)  # Slow down the simulation


# Step 3: Lift the Object
lift_position = [xpos, ypos, -0.03 + 0.1]  # Lift it 10cm above the current position
palmP(lift_position, final_orientation)
for _ in range(200):  # Increase the number of steps
    p.stepSimulation()
    time.sleep(0.01)  # Slow down the simulation


# Step 4: Move the Object over the Tray
move_to_tray_position = [tray_x, tray_y, -0.03 + 0.1]  # Directly over the tray but 10cm above
palmP(move_to_tray_position, final_orientation)
for _ in range(200):  # Increase the number of steps
    p.stepSimulation()
    time.sleep(0.01)  # Slow down the simulation


# Step 5: Release the Object
thumb(0.17, 0.34)
indexF(0.17, 0.17)
midF(0.17, 0.17)
ringF(0.17, 0.17)
pinkyF(0.17, 0.17)
for _ in range(200):  # Increase the number of steps
    p.stepSimulation()
    time.sleep(0.01)  # Slow down the simulation



time.sleep(1)
p.disconnect()
print("disconnected")