import sys
import time

import numpy as np
from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule

class NaoWrapper(ALModule):
    def __init__(self, name):
        ALModule.__init__(self, name)
        # No need for IP and port here because
        # we have our Python broker connected to NAOqi broker

        # Create a proxy
        try:
            """ ALL PROXY """
            self.motionProxy =  ALProxy("ALMotion")
            self.postureProxy = ALProxy("ALRobotPosture")
            # self.awareness = ALProxy('ALBasicAwareness', ip, 9559)
            self.leds =         ALProxy("ALLeds")
            self.memory =       ALProxy("ALMemory")
            self.camProxy =     ALProxy("ALVideoDevice")
        except Exception,error:
            print "Could not create proxy to ALMotion"
            print "Error was: ",error
            sys.exit(1)

        """
            SUBSCRIBE TO EVENTS
        """
        self.memory.subscribeToEvent(
            "TouchChanged",
            "NaoWrapper",
            "onTouched")

        """
            BODY PARTS
        """
        self.body_parts =   self.motionProxy.getBodyNames("Body")
        self.body_parts =   self.body_parts[2:8] + [self.body_parts[10]] + [self.body_parts[16]] + self.body_parts[20:26]

        """ JOINTS LIMITS """
        limits = self.motionProxy.getLimits("Body")
        limits = limits[2:8] + [limits[10]] + [limits[16]] + limits[20:26]
        self.joints_limits = []
        [self.joints_limits.append(limit[:2]) for limit in limits]
        del self.joints_limits[7]


    '''
    -------------------SUBSCRIBE FUNCTIONS-----------------------------
    '''
    def onTouched(self, key, value, message):
        """ This will be called each time a touch
        is detected.
        """
        # Unsubscribe to the event when talking,
        # to avoid repetitions
        self.memory.unsubscribeToEvent(
            "TouchChanged",
            "NaoWrapper",
            )

        print("var1: %s\n var2: %s\n var3: %s"%(key, value, message ))

        # Subscribe again to the event
        self.memory.subscribeToEvent(
            "TouchChanged",
            "NaoWrapper",
            "onTouched")



    '''
    -------------------POSTURES FUNCTIONS-----------------------------
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


    '''
    -------------------MAP FUNCTIONS-----------------------------
    '''

    ''' Map interval bounds to a different interval bound '''
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

    '''
    -------------------READ FUNCTIONS-----------------------------
    '''

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










if __name__ == "nao_wrapper":
    print ("\n-------Thank you for using my library!------- by George M.\n")
