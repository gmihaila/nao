# -*- encoding: UTF-8 -*-
import sys
import almath
from naoqi import ALProxy
import numpy as np
import time
import cPickle as pickle
from PIL import Image


class NaoRobot(object):
    def __init__(self, ip, dataFile):
        self.dataFile = dataFile
        self.recordedActions = []
        try:
            """ ALL PROXY """
            self.motionProxy =  ALProxy("ALMotion", ip, 9559)
            self.postureProxy = ALProxy("ALRobotPosture", ip, 9559)
            # self.awareness = ALProxy('ALBasicAwareness', ip, 9559)
            self.leds =         ALProxy("ALLeds",ip,9559)
            self.memory =       ALProxy("ALMemory", ip, 9559)
            self.camProxy =     ALProxy("ALVideoDevice", ip, 9559)
        except Exception,e:
            print "Could not create proxy to ALMotion"
            print "Error was: ",e
            sys.exit(1)

        """ BODY PARTS """
        self.body_parts = self.motionProxy.getBodyNames("Body")
        self.body_parts = self.body_parts[2:8] + [self.body_parts[10]] + [self.body_parts[16]] + self.body_parts[20:26]

        """ JOINTS LIMITS """
        limits = self.motionProxy.getLimits("Body")
        limits = limits[2:8] + [limits[10]] + [limits[16]] + limits[20:26]
        self.joints_limits = []
        [self.joints_limits.append(limit[:2]) for limit in limits]
        del self.joints_limits[7]

        '''Initialize Force sensing value'''
        self.feet_pressure = self.ForceSensingResistorFeet()

    '''
    START-------------------POSTURES FUNCTIONS-----------------------------START
    '''

    ''' Start Robot in initial posture '''
    def PostureStandInit(self, speed):
        self.postureProxy.goToPosture("StandInit", speed)
        return

    ''' Start Robot in initial posture '''
    def PostureStand(self, speed):
        self.postureProxy.goToPosture("Stand", speed)
        return

    ''' Start Robot in initial posture '''
    def PostureStandZero(self, speed):
        self.postureProxy.goToPosture("StandZero", speed)
        return

    ''' Set Nao to Rest '''
    def Rest(self):
        self.motionProxy.rest()
        return

    ''' Start Position for Domino play - Stretch knees '''
    def NewInitStart(self):
        #move arms down and stiffness 0
        self.PostureStandZero(0.2)
        self.motionProxy.setAngles("LKneePitch", -0.103083, 0.05)
        self.motionProxy.setAngles("RKneePitch", -0.103083, 0.05)

        self.motionProxy.setAngles("LShoulderPitch",    0.95, 0.05)
        self.motionProxy.setAngles("RShoulderPitch",    0.95, 0.05)

        self.motionProxy.setStiffnesses(["LArm", "RArm"], 0.2)
        return

    def EpisodeRestart(self):
        '''SET ARM STIFFNESS TO 0'''
        self.motionProxy.setStiffnesses("RArm", 0.1)
        self.motionProxy.setStiffnesses("LArm", 0.1)
        '''SET HIP STIFFNESS TO 1 AND MOVE IT BACK'''
        self.motionProxy.setStiffnesses(self.body_parts[6], 1.0)    #left hip
        self.motionProxy.setStiffnesses(self.body_parts[7], 1.0)    #right hip
        self.motionProxy.angleInterpolationWithSpeed([self.body_parts[6], self.body_parts[7]], [0, 0], 0.1)
        '''RESTART ARM POSITION'''
        self.motionProxy.setStiffnesses("RArm", 1)
        self.motionProxy.setStiffnesses("LArm", 1)
        self.motionProxy.angleInterpolationWithSpeed([self.body_parts[0], self.body_parts[8]], [0.8, 0.8], 0.1)
        self.feet_pressure = self.ForceSensingResistorFeet()
        print '\nPosition Restarted!\n'


    '''
    END---------------------POSTURES FUNCTIONS-------------------------------END
    '''

    '''
    START-------------------READ/Map VALUES--------------------------------START
    '''

    ''' Map 1 interval bounds to a different interval bound '''
    def Map(self, value, istart, istop, ostart, ostop):
        value = float(value)
        istart = float(istart)
        istop = float(istop)
        ostart = float(ostart)
        ostop = float(ostop)
        output = ostart + (ostop - ostart) * ((value - istart) / (istop - istart))
        return (ostop if (output > ostop) else ostart if (output < ostart) else output)

    ''' Inputs vector action of values 0-1 and outputs vector aciton with radians '''
    def ToRadians(self, action_vector):
        #create empty vector of 6 length
        radiansVector = np.zeros(13).astype(float)

        if len(action_vector) < 13:
            print "Invalid action vector!"
            return None
        else:
            for i in range(0, 13):
                radiansVector[i] = self.Map(action_vector[i], 0, 1, self.joints_limits[i][0],    self.joints_limits[i][1])
        return radiansVector

    ''' Inputs Radians Vector and outputs Action Vector of 0-1 values '''
    def ToAction(self, input_radians):
        action_vector = np.zeros(13).astype(float)
        if len(input_radians) < 13:
            print "Invalid Input Radians Vector"
            return None
        else:
            for i in range(0, 13):
                action_vector[i] = self.Map(input_radians[i], self.joints_limits[i][0],    self.joints_limits[i][1], 0, 1)
        return action_vector

    ''' Outputs 3 vectore in 0-1 format: command readings, sensed readings and error. '''
    def ReadJoints(self):
        #Commands angels
        command = self.motionProxy.getAngles(self.body_parts, False)
        #get average of right and left hip
        command[6] = (command[6] + command[7]) / 2
        #leave only 1 hip coordinate
        del command[7]
        command_val = np.around(self.ToAction(command), 6)

        #Sensed angels
        sensed = self.motionProxy.getAngles(self.body_parts, True)
        sensed[6] = (sensed[6] + sensed[7]) / 2
        #leave only 1 hip coordinate
        del sensed[7]
        sensor_val = np.around(self.ToAction(sensed), 6)

        errors = np.around([(command - sensed) for command, sensed in zip(command_val, sensor_val)], 3)

        return command_val, sensor_val, errors

    ''' Filters any outliers or noise. Standard deviation by default is 1'''
    def FilterNoise(self, data, std=1):
        return data[abs(data - np.mean(data)) < std * np.std(data)]

    '''
    END-----------------------READ/Map VALUES----------------------------------END
    '''

    '''
    START---------------------MOVEMENT FUNCTIONS-----------------------------START
    '''
    ''' Move all upper body: Larm, Rarm, Hips len(action_vector) = 13 '''
    def Move(self, action_vector, blocking):
        if len(action_vector) < 13:
            print "Invalid Movements! Flag Detected!"
            return
        else:
            '''Set stiffness to 1 before move or read values'''
            self.motionProxy.setStiffnesses("RArm", 1.0)
            self.motionProxy.setStiffnesses("LArm", 1.0)
            self.motionProxy.setStiffnesses(self.body_parts[6], 1.0)    #left hip
            self.motionProxy.setStiffnesses(self.body_parts[7], 1.0)    #right hip

            vector_radian = list(self.ToRadians(action_vector))
            ''' Duplicate value for Hips'''
            vector_radian.insert(6, vector_radian[6])
            reading_radian = list(self.ToRadians(self.ReadJoints()[1]))
            ''' Duplicate value for Hips'''
            reading_radian.insert(6, reading_radian[6])

            '''MOVE ARM/S AND/OR HIPS'''
            action_radians, reading_radians, joint_names   = [[], [], []]
            '''Avoid moving on noise data-compaire reading with command rounded'''
            for joint in range(14):
                if joint in [5, 13]:
                    #avoid hands
                    continue
                if round(vector_radian[joint],1) != round(reading_radian[joint], 1):
                    print vector_radian[joint], reading_radian[joint]
                    action_radians.append(vector_radian[joint])
                    reading_radians.append(reading_radian[joint])
                    joint_names.append(self.body_parts[joint])
            if action_radians != []:
                #move all joints
                if blocking == False:   # Allow NON-BLOCKING move
                    #NON-BLOCKING CALL: task is created in a parallel thread. This enables you to do other work at the same time
                    self.motionProxy.setAngles(joint_names, action_radians, 0.06)
                    #start listening
                    while (round(sum(np.abs(action_radians)), 2) - round(sum(np.abs(self.motionProxy.getAngles(joint_names, True))), 2)) != 0:
                        '''CHECK UTILITY VALUES TO RETURN'''
                        if self.UtilityFunction(action_vector) < 0:
                            print 'RESET POSITION -------> END EPISODE!'
                            # self.motionProxy.angleInterpolationWithSpeed(joint_names, reading_radians, 0.06)
                            break
                        self.EpisodeRestart()
                else:
                    # BLOKING CALL: The next instruction will be executed after the end of the previous call
                    # All calls can raise an exception and should be encapsulated in a try-catch block
                    self.motionProxy.angleInterpolationWithSpeed(joint_names, action_radians, 0.06)

            '''MOVE LEFT HAND'''
            if (action_vector[5] >= 0.7) :
                #move hand to that position and set stiffness to 0
                self.motionProxy.angleInterpolationWithSpeed(self.body_parts[5], 0.7, 0.1)
                self.motionProxy.setStiffnesses(self.body_parts[5], 0)
                action_vector[5] = self.ReadJoints()[1][5]
            else:
                #just move hand to that position
                self.motionProxy.angleInterpolationWithSpeed(self.body_parts[5], action_vector[5], 0.1)
            '''MOVE RIGHT HAND'''
            if (action_vector[12] >= 0.7):
                #move hand to that position and set stiffness to 0
                self.motionProxy.angleInterpolationWithSpeed(self.body_parts[13], 0.7, 0.1)
                self.motionProxy.setStiffnesses(self.body_parts[13], 0)
                action_vector[12] = self.ReadJoints()[1][12]
            else:
                #just move hand to that position
                self.motionProxy.angleInterpolationWithSpeed(self.body_parts[13], action_vector[12], 0.1)

            '''CHECK UTILITY VALUES TO RETURN'''
            if self.UtilityFunction(action_vector) < 0:
                print 'RESET POSITION -------> END EPISODE!'
                self.EpisodeRestart()
                return


        return

    '''  Move right and left hip '''
    def MoveHips(self, action_value, speed):
        radians_value = self.Map(action_value, 0, 1, self.joints_limits[6][0], self.joints_limits[6][1])
        self.motionProxy.setStiffnesses([self.body_parts[6], self.body_parts[7]], 1.0)
        self.motionProxy.angleInterpolationWithSpeed([self.body_parts[6], self.body_parts[7]], radians_value, speed)
        return

    '''  Move left shoulder '''
    def MoveLShoulder(self, action_value, speed):
        radians_value = self.Map(action_value, 0, 1, self.joints_limits[0][0], self.joints_limits[0][1])
        self.motionProxy.setStiffnesses(self.body_parts[0], 1.0)
        self.motionProxy.angleInterpolationWithSpeed(self.body_parts[0], radians_value, speed)
        return

    '''  Move right shoulder '''
    def MoveRShoulder(self, action_value, speed):
        radians_value = self.Map(action_value, 0, 1, self.joints_limits[8][0], self.joints_limits[8][1])
        self.motionProxy.setStiffnesses(self.body_parts[8], 1.0)
        self.motionProxy.angleInterpolationWithSpeed(self.body_parts[8], radians_value, speed)
        return
    '''
    END-----------------------MOVEMENT FUNCTIONS-----------------------------END
    '''

    '''
    START---------------------STIFFNESS FUNCTIONS--------------------------START
    '''

    ''' Set Stiffness left Arm '''
    def StiffLArm(self, stiffness):
        self.motionProxy.setStiffnesses(self.body_parts[0:5], 0.01)
        time.sleep(1)
        self.motionProxy.setStiffnesses(self.body_parts[0:5], stiffness)
        return

    ''' Set Stiffness right Arm '''
    def StiffRArm(self, stiffness):
        self.motionProxy.setStiffnesses(self.body_parts[8:13], 0.01)
        time.sleep(1)
        self.motionProxy.setStiffnesses(self.body_parts[8:13], stiffness)
        return

    ''' Set Stiffness Hips '''
    def StiffHips(self, stiffness):
        self.motionProxy.setStiffnesses([self.body_parts[6], self.body_parts[7]], 0.01)
        time.sleep(1)
        self.motionProxy.setStiffnesses([self.body_parts[6], self.body_parts[7]], stiffness)
        return
        '''
        END-----------------------STIFFNESS FUNCTIONS------------------------------END
        '''

    '''
    START---------------------MENU CONTORL FUNCTION-------------------------START
    '''
    def Control(self):
        #initialize list of actions vectors
        actions = []
        #get aciton vector
        action_vector = self.ReadJoints()[1]
        #starting joint
        joint = 0
        #initialize input
        increment = action_vector[joint]
        #initialize message
        joints = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'LHand', 'Hips', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw', 'RHand']
        info = "\n-----> Type r to show current action_vector.\
        \n-----> Type d to dump temp vector to pickle.\
        \n-----> Type a adjust movement. \
        \n-----> Type e execute temp action list. \
        \n-----> Type s save to temp vector. \
        \n-----> Type r rest selected joint and erase temp vector. \
        \n-----> Type p to print joint readings. \
        \n-----> Type snap to show images. \
        \n-----> Current joint: %s\n-----> Type j[joint #] to change joint \
        \n-----> Type i for info. \
        \n-----> Press Enter to exit!\n"%joints[joint]
        print info

        while True:
                user_input = raw_input('\nMove %s: '%(joints[joint]))
                if str(user_input) == '+':
                    if (increment + 0.1) < 1:
                        increment += 0.1
                    else:
                        increment = 1
                    print increment

                elif str(user_input) == '-':
                    if (increment - 0.1) > 0:
                        increment -= 0.1
                    else:
                        increment = 0
                    print increment

                elif 'j' in str(user_input):
                    print str(user_input)[1:]
                    if (len(str(user_input)) == 1):
                        print "Invalid joint. Selecte form 0 to 12!"
                        continue
                    elif (len(str(user_input)) > 3) or (int(str(user_input)[1:]) > 12 ):
                        print "Invalid joint. Selecte form 0 to 12!"
                        continue
                    else:
                        joint = int(str(user_input)[1:])
                        increment = action_vector[joint]
                        print "Sensed Value: ",self.ReadJoints()[1][joint]
                        print "Joint %s selected!"%(joints[joint])

                elif str(user_input) == 'p':
                    print "Command Values: ",self.ReadJoints()[0]
                    print "Sensed Values: ",self.ReadJoints()[1]
                    print "errors: ",self.ReadJoints()[2]

                elif str(user_input) == 'd':
                    [self.DumpToPickle(action) for action in actions]
                    self.DumpToPickle([0])
                    del actions[:]
                    print "Temp action list cleared!"

                elif str(user_input) == 'a':
                        record = self.ReadJoints()[0]
                        print record
                        self.motionProxy.setStiffnesses(self.body_parts[6:14], 0)
                        print "-------------------> Start moving arm! 6 seconds <-----------"
                        for i in range(6):
                            if i <= 3:
                                time.sleep(1)
                                print "-----> SECONDS: ",i+1
                            else:
                                time.sleep(1)
                                print "-----!!!!!!!!!!!!!!!!!----> SECONDS: ",i+1
                        increment = self.ReadJoints()[1][joint]

                elif str(user_input) == 's':
                    #save to action list
                    reading = self.ReadJoints()[1]
                    print "Temporary action added: ",reading
                    actions.append(reading)

                elif str(user_input) == 'e':
                    counter = 0
                    for action in actions:
                        print "\nTemp Action %s\n%s" %(counter,action)
                        self.Move(action, True)
                        counter += 1

                elif str(user_input) == 'r':
                    #rest arm
                    self.motionProxy.setStiffnesses(joints[joint], 0.01)
                    print "Joint %s rested!"%joints[joint]
                    print "Temp action list cleared!"
                    del actions[:]
                    continue

                elif str(user_input) == 'i':
                    print(self.ForceSensingResistorFeet())
                    continue

                elif str(user_input) == 'snap':
                    top_image = self.SnapPicture(camera_code=0, resoution=2, color_space=11)
                    bottom_image = self.SnapPicture(camera_code=1, resoution=2, color_space=11)
                    continue

                elif str(user_input) == "q":
                    print "\nCommand closed!"
                    break

                else:
                    try:
                        float(user_input)
                        increment = float(user_input)
                        print increment
                    except ValueError:
                        print "\nTry again!"
                        continue
                    # continue
                #update reading before sending command
                action_vector = self.ReadJoints()[1]
                action_vector[joint] = increment
                self.Move(action_vector, True)

        return
    '''
    END-----------------------MENU CONTORL FUNCTIONS---------------------------END
    '''

    '''
    START-----------------------PICKLE FUNCTIONS-----------------------------START
    '''

    ''' Dump action vector to pickle file '''
    def DumpToPickle(self, sensed_values):
        with open('%s.pickle'%(self.dataFile), 'a+b') as handle:
       	 pickle.dump(sensed_values, handle, -1)
         print "Dumped: ", sensed_values
         self.recordedActions.append(sensed_values)
         return

    ''' Execute pickle file in actions '''
    def ExecutePickle(self, dataFile):
        counter = 0
        print "Executing file", dataFile
        self.dataFile = dataFile + "_flagged"
        with (open(dataFile+".pickle", "rb")) as openfile:
            while True:
                try:
                    action = pickle.load(openfile)
                    print "\nAction %s\n%s" %(counter,action)
                    self.Move(action, True)
                    counter += 1
                    user_input = raw_input('\nWhat to do? : ')
                    if str(user_input) == 'n':
                        self.DumpToPickle(action)
                        continue
                    elif str(user_input) == 'f':
                        self.DumpToPickle(action)
                        self.DumpToPickle([0])
                        continue
                except EOFError:
                    break
        return
    '''
    END-------------------------PICKLE FUNCTIONS-------------------------------END
    '''

    '''
    START-----------------------FALL MANAGER---------------------------------START
    '''
    #deactivate/activate fall manager
    def FallManager(self, activ):
        self.motionProxy.setFallManagerEnabled(activ)
        return
    '''
    END-------------------------FALL MANAGER-----------------------------------END
    '''

    '''
    START-----------------------ROBOT CONFIGURATION--------------------------START
    '''
    def Configuration(self):
        robotConfig = self.motionProxy.getRobotConfig()
        print "\nRobot Configuration:"
        for i in range(len(robotConfig[0])):
            print robotConfig[0][i], ": ", robotConfig[1][i]
        print "End Robot Configuration.\n"
        return
    '''
    END-------------------------ROBOT CONFIGURATION----------------------------END
    '''

    '''
    START-----------------------GYRO ACCELLEROMETER--------------------------START
    '''
    def Gyrometer(self):
        print "Gyrometer values"
        return

    def Accelerometer(self):
        AngleX = self.memory.getData("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value")
        AngleY = self.memory.getData("Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value")
        print AngleX
        print AngleY
        return

    def CenterOfPresure(self):
        print self.memory.getData("Device/SubDeviceList/LFoot/FSR/CenterOfPressure/X/Sensor/Value")
        print self.memory.getData("Device/SubDeviceList/LFoot/FSR/CenterOfPressure/Y/Sensor/Value")
        print self.memory.getData("Device/SubDeviceList/RFoot/FSR/CenterOfPressure/X/Sensor/Value")
        print self.memory.getData("Device/SubDeviceList/RFoot/FSR/CenterOfPressure/Y/Sensor/Value")
        return

    def ForceSensingResistorFeet(self):
        LFsrTW, RFsrTW = ([], [])
        for i in range(20):
            if i < 10:
                #Avoids inertial noise
                continue
            #Left Foot
            LFsrTW.append(self.memory.getData("Device/SubDeviceList/LFoot/FSR/TotalWeight/Sensor/Value"))
            #Right Foot
            RFsrTW.append(self.memory.getData("Device/SubDeviceList/RFoot/FSR/TotalWeight/Sensor/Value"))

        LFsrTW = np.mean(self.FilterNoise(np.array(LFsrTW), 0.75))
        RFsrTW = np.mean(self.FilterNoise(np.array(RFsrTW), 0.75))

        return round((LFsrTW + RFsrTW)/2, 4)

    '''
    END-------------------------GYRO ACCELLEROMETER----------------------------END
    '''

    '''
    START-----------------------UTILITY FUNCTION-----------------------------START
    '''
    def UtilityFunction(self, action_vector):
        print '\nFunction called\n'
        updated_feet_sens = self.ForceSensingResistorFeet()
        utility_value = None
        if (abs(updated_feet_sens - self.feet_pressure) > 0.18):
            print "TOUCHED TABLE------------------>FEET SENSORS -"
            utility_value = -1
        else:
            #Update position - NOTHING HAPPEN
            self.feet_pressure = updated_feet_sens
            utility_value = 0

        if (abs(self.ReadJoints()[1][5] - action_vector[5]) >= 0.05 and self.ReadJoints()[1][5] < 0.5) or (abs(self.ReadJoints()[1][12] - action_vector[12]) >= 0.05 and self.ReadJoints()[1][12] < 0.5):
            print "------------------------------->OBJECT GRABBED +"
            utility_value = 1

        if (self.ReadJoints()[1][5] > 0.709) or (self.ReadJoints()[1][5] > 0.709):
            print "TOUCHED TABLE------------------>HAND -"
            utility_value = -1
        return utility_value
    '''
    END-------------------------UTILITY FUNCTION-------------------------------END
    '''

    '''
    START----------------------CAMERA FUNCTION-------------------------------START
    '''

    def SnapPicture(self, camera_code, resoution, color_space):

        self.camProxy.setActiveCamera("ALVideoDevice",camera_code)

        client = self.camProxy.subscribeCamera("python_client",camera_code, resoution, color_space, 5)
        raw_image = self.camProxy.getImageRemote(client)
        self.camProxy.unsubscribe(client)

        image_width = raw_image[0]
        image_height = raw_image[1]
        image_array = raw_image[6]

        image = Image.frombytes("RGB", (image_width, image_height), image_array)

        image.show()
        image = np.array(image)

        return image

    def CameraFunction(self):

        resolution = 2    # VGA
        color_space = 11   # RGB

        top_image = self.SnapPicture(camera_code=0, resoution=resolution, color_space=color_space)

        bottom_image = self.SnapPicture(camera_code=1, resoution=resolution, color_space=color_space)


        return top_image, bottom_image

    '''
    END------------------------CAMERA FUNCTION--------------------------------END
    '''

    '''
    START-----------------------RANDOM FUNCTIONS-----------------------------START
    '''

    # Start Awareness
    def startAwareness(self):
        self.awareness.startAwareness()
        return

    # Stop Awareness
    def stopAwareness(self):
        self.awareness.stopAwareness()
        return

    # Rasta Eyes Leds
    def randomEyes(self, duration):
        self.leds.rasta(duration)
        return

    # Cosine similarity
    def Cosim(a, b):
        dot_product = np.dot(a, b)
        norm_a = np.linalg.norm(a)
        norm_b = np.linalg.norm(b)
        return dot_product / (norm_a * norm_b)

    '''
    END-----------------------RANDOM FUNCTIONS-----------------------------END
    '''



if __name__ == "nao_class_wrapper":
    print ("\n-------Thank you for using my library!------- by George M.\n")
