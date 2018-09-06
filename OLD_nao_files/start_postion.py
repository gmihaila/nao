# -*- encoding: UTF-8 -*-
from naoqi import ALProxy

from nao_class_wrapper import NaoRobot
import time

IP = "10.125.227.170"
# IP = "localhost"
dataFile = "my_data2"


# bobby = NaoRobot(IP, dataFile)
# bobby.FallManager(False)
#
# bobby.NewInitStart()
# # bobby.CameraFunction()
# # bobby.startAwareness()
# bobby.Rest()

# bobby.EpisodeRestart()

# bobby.ExecutePickle(dataFile)

# print bobby.memory.getData("Device/SubDeviceList/LFoot/FSR/TotalWeight/Sensor/Value")
# print bobby.memory.getData("Device/SubDeviceList/RFoot/FSR/TotalWeight/Sensor/Value")

# print bobby.ForceSensingResistorFeet()


# bobby.Control()

# print bobby.memory.getData("Diagnosis/Temperature/JointName")

# print bobby.body_parts

# print bobby.ReadJoints()

# bobby.Move(bobby.ReadJoints()[0])
# bobby.MoveRShoulder(0.5, 0.2)
# time.sleep(2)
# bobby.StiffRArm(0.0)

# print bobby.ReadJoints()[0][0]

# bobby.Configuration()
# bobby.CenterOfPresure()
# bobby.ForceSensingResistorFeet()
memory = ALProxy("ALMemory", IP, 9559)

head_front = memory.getData("Device/SubDeviceList/Head/Touch/Front/Sensor/Value")
while True:
    if memory.getData("Device/SubDeviceList/Head/Touch/Front/Sensor/Value") != head_front:
        head_front = memory.getData("Device/SubDeviceList/Head/Touch/Front/Sensor/Value")
        print(head_front)
    # print(memory.getData("Device/SubDeviceList/Head/Touch/Front/Sensor/Value"))
    # print(memory.getData("Device/SubDeviceList/LHand/Position/Sensor/Value"))
    # print(memory.getData("Device/SubDeviceList/LHand/ElectricCurrent/Sensor/Value"))
