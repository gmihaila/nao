import sys
import time
import math
import random

import csv
import numpy as np
from PIL import Image
from naoqi import ALProxy
# from naoqi import ALBroker
import scipy.misc

from naoqi import ALModule
class NaoWrapper(object):
    """
        LINKS:  http://doc.aldebaran.com/2-1/dev/naoqi/index.html#naoqi-process
                http://doc.aldebaran.com/1-14/glossary.html#term-dcm
    """
    def __init__(self, ip, path_csv):
        try:
            """ ALL PROXY """
            self.motionProxy =  ALProxy("ALMotion", ip, 9559)
            self.postureProxy = ALProxy("ALRobotPosture", ip, 9559)
            # self.awareness = ALProxy('ALBasicAwareness', ip, 9559)
            self.leds =         ALProxy("ALLeds",ip,9559)
            self.memory =       ALProxy("ALMemory", ip, 9559)
            self.camProxy =     ALProxy("ALVideoDevice", ip, 9559)
	    self.diagnosisProxy= ALProxy("ALDiagnosis", ip,9559)
	    self.sonarProxy = ALProxy("ALSonar", ip, 9559)
      	    self.sonarProxy.subscribe("myApplication")
        except Exception,e:
            print "Could not create proxy to ALMotion"
            print "Error was: ",e
            sys.exit(1)

        """
            SUBSCRIBE TO EVENTS
            LINK: http://doc.aldebaran.com/2-1/naoqi-eventindex.html
        """

        """
            VARIABLES
        """
        self.path_csv = path_csv
        # MOTORS SPEED
        self.speed = 0.1
        # BODY PARTS
        self.body_parts =   self.motionProxy.getBodyNames("Body")
	print(self.body_parts)
	# AVOID RIGHT HIP self.body_parts[16] -> COMMAND BOTH THE SAME
        self.body_parts =   self.body_parts[2:8] + [self.body_parts[10]] + self.body_parts[20:26]
	#print(self.body_parts)
        # JOINTS LIMITS
        self.joints_limits = self.motionProxy.getLimits("Body")[2:8]    \
                            + [self.motionProxy.getLimits("Body")[10]]  \
                            + self.motionProxy.getLimits("Body")[20:26]
        self.joints_limits = [limit[:2] for limit in self.joints_limits]

        return

    """
        -------------------POSTURES FUNCTIONS-----------------------------
        LINK: http://doc.aldebaran.com/2-1/family/robots/postures_robot.html

    """

    """ Start Robot in initial posture """
    def PostureStandInit(self, speed):
        self.postureProxy.goToPosture("StandInit", speed)
        return

    """ Start Robot in initial posture """
    def PostureStand(self, speed):
        self.postureProxy.goToPosture("Stand", speed)
        return

    """ Start Robot in initial posture """
    def PostureStandZero(self, speed):
        self.postureProxy.goToPosture("StandZero", speed)
        return

    """ Set Nao to Rest """
    def Rest(self):
        self.motionProxy.rest()
        return

    ''' Start Position for Domino play - Stretch knees '''
    def NewInitStart(self):
        #move arms down and stiffness 0
        self.PostureStandZero(0.2)
        # self.motionProxy.setAngles("LHipPitch", -0.3, 0.05)
        # self.motionProxy.setAngles("RHipPitch", -0.3, 0.05)

        self.motionProxy.setAngles("LKneePitch", -0.103083, 0.05)
        self.motionProxy.setAngles("RKneePitch", -0.103083, 0.05)

        self.motionProxy.setAngles("LShoulderPitch",  0.3, 0.05)
        self.motionProxy.setAngles("RShoulderPitch",  0.3, 0.05)
        self.motionProxy.setStiffnesses(["RArm"], 0.1)
        return


    """
        -------------------MAP FUNCTIONS-----------------------------
    """

    """
        Random number in interval
    """
    def RandFloat(self, high):
        return random.random()*(high)


    """
        Truncate numbers
    """
    def Truncate(self,f, n):
        return math.floor(f * 10 ** n) / 10 ** n

    """
        Map interval bounds to a different interval bound
    """
    def Map(self, value, istart, istop, ostart, ostop):
        value = float(value)
        istart = float(istart)
        istop = float(istop)
        ostart = float(ostart)
        ostop = float(ostop)
        output = ostart + (ostop - ostart) * ((value - istart) / (istop - istart))
        return (ostop if (output > ostop) else ostart if (output < ostart) else output)

    """
        Inputs vector action of values 0-1 and outputs vector aciton with radians
    """
    def ToRadians(self, action_vector):
        #create empty vector of 6 length
        radiansVector = np.zeros(13).astype(float)

        if len(action_vector) != 13:
            print("Invalid action vector!")
            return None
        else:
            for i in range(0, 13):
                radiansVector[i] = self.Map(action_vector[i], 0, 1, self.joints_limits[i][0],    self.joints_limits[i][1])
        return radiansVector

    """
        Inputs Radians Vector and outputs Action Vector of 0-1 values
    """
    def ToAction(self, input_radians):
        action_vector = np.zeros(13).astype(float)
        if len(input_radians) != 13:
            print("Invalid Input Radians Vector")
            return None
        else:
            for i in range(0, 13):
                action_vector[i] = self.Map(input_radians[i], self.joints_limits[i][0],    self.joints_limits[i][1], 0, 1)
        return action_vector

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


    """
        -------------------READ FUNCTIONS-----------------------------
    """

    """
        Outputs 3 vectore in 0-1 format: command readings, sensed readings and error.
        LINK: http://doc.aldebaran.com/1-14/naoqi/motion/control-joint-api.html
    """
    def ReadMemoryJoints(self):
        # JOINTS POSITION ACTUATORS
        pos_act = []
        # JOINTS POSITION SENSORS
        pos_sens = []
        for body_part in self.body_parts:
            # JOINTS POSITION ACTUATORS
            pos_act.append(
                self.memory.getData("Device/SubDeviceList/%s/Position/Actuator/Value"%body_part))
            # JOINTS POSITION SENSORS
            pos_sens.append(
                self.memory.getData("Device/SubDeviceList/%s/Position/Sensor/Value"%body_part))
            # RADIANS TO ACTION
        pos_act[6] = (pos_act[6] + self.memory.getData("Device/SubDeviceList/"\
                                    "RHipPitch/Position/Sensor/Value")) / 2
        pos_act = np.around(self.ToAction(pos_act), 5)
        pos_sens = np.around(self.ToAction(pos_sens), 5)

        return pos_act, pos_sens


    """
        Read Memory values
        LINK: http://doc.aldebaran.com/2-1/family/nao_dcm/actuator_sensor_names.html#lhippitch
    """
    def PrintNaoData(self):
        print(  "\n0: means regular temperature\n"  \
                "1: means temperature has reach the max limit, start reducing stiffness.\n"   \
                "2: means the joint is very hot, stiffness reduced over 30%.\n"               \
                "3: means the joint is critically hot, stiffness value is set to 0\n")

        print("CPU: %sC"%self.memory.getData("Device/SubDeviceList/Head/Temperature/Sensor/Value"))
        print("Battery: %2.2fA %2.2fcharge %2.2fC"%(self.memory.getData("Device/SubDeviceList/Battery/Current/Sensor/Value"),
                                        self.memory.getData("Device/SubDeviceList/Battery/Charge/Sensor/Value"),
                                        self.memory.getData("Device/SubDeviceList/Battery/Temperature/Sensor/Value")))
        for i, body_part in enumerate(self.body_parts):
            actuator = self.memory.getData("Device/SubDeviceList/%s/Position/Actuator/Value"%body_part)
            actuator = round(self.Map(actuator, self.joints_limits[i][0],    self.joints_limits[i][1], 0, 1),4)
            print("ACT: %s"%actuator),
            sensor = self.memory.getData("Device/SubDeviceList/%s/Position/Sensor/Value"%body_part)
            sensor = round(self.Map(sensor, self.joints_limits[i][0],    self.joints_limits[i][1], 0, 1),4)
            print("\tSEN: %s"%sensor),
            print("\tCUR: %2.4fA"%self.memory.getData("Device/SubDeviceList/%s/ElectricCurrent/Sensor/Value"%body_part)),
            print("\tTEM: %2.4fC"%self.memory.getData("Device/SubDeviceList/%s/Temperature/Sensor/Value"%body_part)),
            print("\tTEM: %2.4flvl"%self.memory.getData("Device/SubDeviceList/%s/Temperature/Sensor/Status"%body_part)),
            print("\tSTI: %2.4f"%self.memory.getData("Device/SubDeviceList/%s/Hardness/Actuator/Value"%body_part)),
            print("\t%s"%body_part)
        return



    def ReadMemory(self, body_parts):
        # JOINTS POSITION ACTUATORS
        pos_act = []
        # JOINTS POSITION SENSORS
        pos_sens = []
        # JOINTS CURRENTS
        current = []
        # JOINTS TEMPERATURE VALUES
        temp_val = []
        # JOINTS TEMPERATURE STATUS
        temp_sts = []
        # JOINTS STIFFNESS
        stif_val = []
        for body_part in body_parts:
            # JOINTS POSITION ACTUATORS
            pos_act.append(
                self.memory.getData("Device/SubDeviceList/%s/Position/Actuator/Value"%body_part))
            # JOINTS POSITION SENSORS
            pos_sens.append(
                self.memory.getData("Device/SubDeviceList/%s/Position/Sensor/Value"%body_part))
            # JOINTS CURRENTS
            current.append(
                self.memory.getData("Device/SubDeviceList/%s/ElectricCurrent/Sensor/Value"%body_part))
            # JOINTS TEMPERATURE VALUES
            temp_val.append(
                self.memory.getData("Device/SubDeviceList/%s/Temperature/Sensor/Value"%body_part))
            # JOINTS TEMPERATURE STATUS
            temp_sts.append(
                self.memory.getData("Device/SubDeviceList/%s/Temperature/Sensor/Status"%body_part))
            # JOINTS STIFFNESS
            stif_val.append(
                self.memory.getData("Device/SubDeviceList/%s/Hardness/Actuator/Value"%body_part))
        # RADIANS TO ACTION
        pos_act = np.around(self.ToAction(pos_act), 5)
        pos_sens = np.around(self.ToAction(pos_sens), 5)
        # ERRORS BETWEEN SENSORS AND ACTUATORS
        errors = np.around([(command - sensed) for command, sensed in zip(pos_act, pos_sens)], 3)

        return pos_act, pos_sens, errors, current, temp_val, temp_sts, stif_val

    def ReadTopHead(self):
        return self.memory.getData("Device/SubDeviceList/Head/Touch/Middle/Sensor/Value")
    def ReadFrontHead(self):
        return self.memory.getData("Device/SubDeviceList/Head/Touch/Front/Sensor/Value")
    def ReadRearHead(self):
        return self.memory.getData("Device/SubDeviceList/Head/Touch/Rear/Sensor/Value")


    """
        -------------------MOVE FUNCTIONS-----------------------------
    """

    """
        USE ACTION VECTOR TO MOVE
        LINK: http://doc.aldebaran.com/1-14/naoqi/motion/control-joint.html
    """
    def Move(self, action_vector):
        # READ VALUES JOINTS
        pos_act, pos_sens = self.ReadMemoryJoints()
        # GET RADIANS TO COMMAND
        rad_vector, pos_sens = list(self.ToRadians(action_vector)), list(self.ToRadians(pos_sens))
        rad_vector, pos_sens = np.around(rad_vector, 4), np.around(pos_sens, 4)

        to_move =    [i for i, (sen,act) in enumerate(zip(pos_sens, rad_vector)) \
                        if self.Truncate(sen,2) != self.Truncate(act,2)]

        if to_move:
            # WE HAVE THINGS TO MOVE
            parts_move = [self.body_parts[move] for move in to_move]
            print("Moving: %s"%(parts_move))
            rad_move = [round(rad_vector[move],3) for move in to_move]
            if 6 in to_move:
                parts_move.append("RHipPitch")
                rad_move.append(rad_move[to_move.index(6)])
            # SET STIFFNESS
            [self.motionProxy.setStiffnesses(body_part, 1.0) for body_part in parts_move]
            # NON-BLOCKING CALL:
            # task is created in a parallel thread. This enables you to do other work at the same time
            # LINK: http://doc.aldebaran.com/1-14/dev/naoqi/index.html
            self.motionProxy.setAngles(parts_move, rad_move, self.speed)
            # TRUE IF THERE IS MOVEMENT
            return True

        else:
            # NOTHING TO MOVE
            print("No move!")
            # FALSE IF THERE IS NO MOVEMENT
            return False

    """
        -------------------STIFFNESS FUNCTIONS-----------------------------
    """
    def StiffRArm(self, stiffness):
        self.motionProxy.setStiffnesses(self.body_parts[8:13], stiffness)
        print("Stiffness RArm set to ",stiffness)
        return

    """
        ---------------------SNAP DATA FUNCTION-------------------------STARTiNRETIAL_gYROSCOPE
    """
    def ToCsv(self,tocsv):
	body_joints=self.motionProxy.getBodyNames("Body")
	body_joints= body_joints[0:14]+body_joints[15:]
      	joints_limits_csv = self.motionProxy.getLimits("Body")[0:14]+self.motionProxy.getLimits("Body")[15:]
	body_info=[[] for i in range(34)]
	i=0;
	#JOINTS DATA
	for body_part in body_joints:
		body_info[i].append(self.memory.getData("Device/SubDeviceList/%s/Position/Actuator/Value"%body_part))
		body_info[i].append(self.memory.getData("Device/SubDeviceList/%s/Position/Sensor/Value"%body_part))
		body_info[i].append(self.memory.getData("Device/SubDeviceList/%s/ElectricCurrent/Sensor/Value"%body_part))
 		body_info[i].append(self.memory.getData("Device/SubDeviceList/%s/Temperature/Sensor/Value"%body_part))
		body_info[i].append(self.memory.getData("Device/SubDeviceList/%s/Hardness/Actuator/Value"%body_part))
 		body_info[i].append(self.memory.getData("Device/SubDeviceList/%s/Temperature/Sensor/Status"%body_part))
		body_info[i].append(body_info[i][0]-body_info[i][1])
		body_info[i].append(self.memory.getData("Diagnosis/Active/%s/Error"%body_part))
		body_info[i].append(self.memory.getData("Diagnosis/Passive/%s/Error"%body_part))
		body_info[i].append(self.memory.getData("Diagnosis/Temperature/%s/Error"%body_part))
		i+=1
	#CPU DATA		
	body_info[i].append(self.memory.getData("Device/SubDeviceList/Head/Temperature/Sensor/Value"))		
	i+=1	
	#BATTERY DATA
	Battery=["Current","Charge","Temperature"]
	for battery_value in Battery:
		body_info[i].append(self.memory.getData("Device/SubDeviceList/Battery/%s/Sensor/Value"%battery_value))
	i+=1	
	#INERTIAL DATA
	inertial=["Gyroscope","Angle","Accelerometer"]
	for j in range(0,len(inertial)):
		body_info[i].append(self.memory.getData("Device/SubDeviceList/InertialSensor/%sX/Sensor/Value"%inertial[j]))
		body_info[i].append(self.memory.getData("Device/SubDeviceList/InertialSensor/%sY/Sensor/Value"%inertial[j]))
		body_info[i].append(self.memory.getData("Device/SubDeviceList/InertialSensor/%sZ/Sensor/Value"%inertial[j]))
		i+=1	
	#FSR DATA
	#FSR=["L","R"]
        FSR=["FrontLeft", "FrontRight", "RearLeft", "RearRight", "TotalWeight", "CenterOfPressure/X", "CenterOfPressure/Y"]
        for FSR_feature in FSR:
		body_info[i].append(self.memory.getData("Device/SubDeviceList/LFoot/FSR/%s/Sensor/Value"%FSR_feature))
		body_info[i+1].append(self.memory.getData("Device/SubDeviceList/RFoot/FSR/%s/Sensor/Value"%FSR_feature))
        i+=2
        #HANDS DATA
	if(abs(body_info[24][0]-body_info[24][1])>0.01):
		body_info[i].append(1)	
	else:
		body_info[i].append(0)
	i+=1
	if(abs(body_info[7][0]-body_info[7][1])>0.01):
		body_info[i].append(1)		
	else:
		body_info[i].append(0)
	i+=1
	#SNAP
	top_image,bottom_image=self.CameraFunction()
	scipy.misc.imsave("Snaps/%s_top.jpg"%tocsv, top_image)
	scipy.misc.imsave("Snaps/%s_bottom.jpg"%tocsv, bottom_image)	
        #MAP VALUES
	'''for i in range(0,len(body_info)-9):
		for j in range(0,len(body_info[i])-1):
			body_info[i][j] = self.Map(body_info[i][j], joints_limits_csv[i][0],joints_limits_csv[i][1], 0, 1)
	body_info[0:25]=np.around(body_info[0:25], 2)
	'''
	#DUMP INTO CSV
	body_info2=list(body_info)	
	body_info2.append(tocsv)    
	with open(self.path_csv, "a+b") as f:
       	    	writer = csv.writer(f)
            	writer.writerows([body_info2])
	f.close()
        return

    def sonar(self):
	a=[]
	b=[]
	for i in range(9):
		if(i==0):
			a.append(self.memory.getData("Device/SubDeviceList/US/Left/Sensor/Value"))
			b.append(self.memory.getData("Device/SubDeviceList/US/Right/Sensor/Value"))
		else:
			a.append(self.memory.getData("Device/SubDeviceList/US/Left/Sensor/Value%s"%i))
			b.append(self.memory.getData("Device/SubDeviceList/US/Right/Sensor/Value%s"%i))
	print()	
	print(a)
	print(b)
	print(self.memory.getData("Device/SubDeviceList/US/Actuator/Value"))
	print(self.memory.getData("Device/SubDeviceList/US/Sensor/Value"))
	
    """
        ---------------------MENU CONTORL FUNCTION-------------------------START
    """
    def Control(self):
        status = sum(self.Fsr())
        #initialize list of actions vectors
        actions = []
        #get aciton vector
        action_vector = self.ReadMemoryJoints()[1]
        print("Menu ",action_vector)
        #starting joint
        joint = 0
        #initialize input
        increment = action_vector[joint]
        #initialize message
        joints = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'LHand', 'Hips', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw', 'RHand']
        info = "\n-----> Type r to show current action_vector.\
        \n-----> Type d[movement or action performed] to dump temp vector to pickle and take snap.\
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
                    print "Sensed Value: ",self.ReadMemoryJoints()[1][joint]
                    print "Joint %s selected!"%(joints[joint])

            elif str(user_input) == 'p':
                actuator = self.ReadMemoryJoints()[0]
                sensor = self.ReadMemoryJoints()[1]
                errors = np.around([abs(command - sensed) for command, sensed in zip(actuator, sensor)], 3)
                for body_part, act, sen, err in zip(self.body_parts, actuator, sensor, errors):
                    print("{:>15s}\tACT: {:>4.4f}\tSEN: {:>2.4f}\tERR: {:>2.4f}".format(body_part, act, sen, err))

            elif str(user_input[0]) == 'd':
                # DUMP ACTION VECTOR TO CSV
		tocsv=user_input[1:]
                self.ToCsv(tocsv)
                print "Action dumped to %s"%self.path_csv

            elif str(user_input) == 'a':
                    record = self.ReadMemoryJoints()[0]
                    print record
                    self.motionProxy.setStiffnesses(self.body_parts[0:5], 0)
                    print "-------------------> Start moving arm! 6 seconds <-----------"
                    for i in range(6):
                        if i <= 3:
                            time.sleep(1)
                            print "-----> SECONDS: ",i+1
                        else:
                            time.sleep(1)
                            print "-----!!!!!!!!!!!!!!!!!----> SECONDS: ",i+1
                    increment = self.ReadMemoryJoints()[1][joint]

            elif str(user_input) == 's':
                #save to action list
                reading = self.ReadMemoryJoints()[1]
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
                self.PrintNaoData()
                continue

            elif str(user_input) == 'i':
                print(self.ReadMemoryJoints()[0]) 
		print(self.ReadMemoryJoints()[1])              
		continue

            elif str(user_input) == 'snap':
                top_image = self.SnapPicture(camera_code=0, resoution=2, color_space=11)
                bottom_image = self.SnapPicture(camera_code=1, resoution=2, color_space=11)
                continue

            elif str(user_input) == "q":
                print "\nCommand closed!"
                break
	
	    elif str(user_input) == "o":
		self.sonar()
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
            action_vector = self.ReadMemoryJoints()[1]
            action_vector[joint] = increment
            self.Move(action_vector)

        return


    """
    ---------------------BALANCE FUNCTION-------------------------
    """
    def CopLF(self):
        return round(self.memory.getData("Device/SubDeviceList/LFoot/FSR/CenterOfPressure/X/Sensor/Value"),2),\
               round(self.memory.getData("Device/SubDeviceList/RFoot/FSR/CenterOfPressure/X/Sensor/Value"),2)

    def Fsr(self):
        return round(self.memory.getData("Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value"),2),\
               round(self.memory.getData("Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value"),2),\
               round(self.memory.getData("Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value"),2),\
               round(self.memory.getData("Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value"),2)

    def ActionSim(self, target, predicted):
        error = np.around([abs(command - sensed) for command, sensed in zip(target, predicted)], 2)
        if (error[0] > 0.02) or (error[1] > 0.02):
            return -1
        elif (error[2] > 0.1) or (error[3] > 0.1):
            return -1
        elif (error[4] > 0.15) or (error[5] > 0.15):
            return -1
        else:
            return 1

    def SimMov(self, action):
        # MAKE A COPY TO AVOID VARIABLE CHANGING
        new_action = action[:]
        new_action[0] += random.choice([self.RandFloat(0.05), - self.RandFloat(0.05)])
        new_action[1] += random.choice([self.RandFloat(0.05), - self.RandFloat(0.05)])
        new_action[2] += random.choice([self.RandFloat(0.1), - self.RandFloat(0.1)])
        new_action[3] += random.choice([self.RandFloat(0.1), - self.RandFloat(0.1)])
        new_action[4] += random.choice([self.RandFloat(0.15), - self.RandFloat(0.15)])
        new_action[5] += random.choice([self.RandFloat(0.15), - self.RandFloat(0.15)])
        new_action = [0 if a <0 else a for a in new_action]
        return new_action


    '''
    START----------------------CAMERA FUNCTION-------------------------------START
        LINK:   http://doc.aldebaran.com/2-1/family/robots/video_robot.html
                http://doc.aldebaran.com/1-14/dev/python/examples/vision/get_image.html
    '''

    def SnapPicture(self, camera_code, resoution, color_space):

        self.camProxy.setActiveCamera("ALVideoDevice",camera_code)

        client = self.camProxy.subscribeCamera("python_client",camera_code, resoution, color_space, 5)
        raw_image = self.camProxy.getImageRemote(client)
        self.camProxy.unsubscribe(client)
        # self.camProxy.releaseImage(client)
        image_width = raw_image[0]
        image_height = raw_image[1]
        image_array = raw_image[6]

        image = Image.frombytes("RGB", (image_width, image_height), image_array)

        # image.show()
        image = np.array(image)

        return image

    def CameraFunction(self):
        """
            Resolution size:
                0 (120x160)
                1 (320x240)     QVGA
                2 (640x480)     VGA
                3 (1280x960)    4VGA
                4 (320x240)
                5 (1280x720)
                6 (320x240)
                7 (60x80)
                8 (30x40)
        """
        resolution = 13    # VGA
        color_space = 11   # RGB

        top_image = self.SnapPicture(camera_code=0, resoution=resolution, color_space=color_space)

        bottom_image = self.SnapPicture(camera_code=1, resoution=resolution, color_space=color_space)


        return top_image, bottom_image

    '''
    END------------------------CAMERA FUNCTION--------------------------------END
    '''




if __name__ == "nao_class":
    print("\n-------Thank you for using my library!------- by George M.\n")
