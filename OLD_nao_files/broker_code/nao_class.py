import sys
import time

import numpy as np
from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule

class NaoWrapper(ALModule):
    """
        LINKS:  http://doc.aldebaran.com/2-1/dev/naoqi/index.html#naoqi-process
                http://doc.aldebaran.com/1-14/glossary.html#term-dcm
    """
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
            print("Could not create proxy to ALMotion")
            print("Error was: ",error)
            sys.exit(1)

        """
            SUBSCRIBE TO EVENTS
            LINK: http://doc.aldebaran.com/2-1/naoqi-eventindex.html
        """
        self.memory.subscribeToEvent(
            "TouchChanged",
            "NaoWrapper",
            "onTouched")

        """
            VARIABLES
        """
        # MOTORS SPEED
        self.speed = 0.1
        # BODY PARTS
        self.body_parts =   self.motionProxy.getBodyNames("Body")
        # AVOID RIGHT HIP self.body_parts[16] -> COMMAND BOTH THE SAME
        self.body_parts =   self.body_parts[2:8] + [self.body_parts[10]] + self.body_parts[20:26]

        # JOINTS LIMITS
        self.joints_limits = self.motionProxy.getLimits("Body")[2:8]    \
                            + [self.motionProxy.getLimits("Body")[10]]  \
                            + self.motionProxy.getLimits("Body")[20:26]
        self.joints_limits = [limit[:2] for limit in self.joints_limits]

    """
        -------------------SUBSCRIBE FUNCTIONS-----------------------------
    """
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

        # print("var1: %s\n var2: %s\n var3: %s"%(key, value, message ))
        # DO ANY FEEDBACK
        for val in value:
            if val[0] == "Head/Touch/Front" and val[1] == False:
                memory = self.ReadMemory()
                print("Actuator: %s\nSensor: %s\nError: %s\nCurrent: %s\nTemperature: %s\nTemp status:%s\nStiffness: %s"
                        %(memory[0], memory[1], memory[2], memory[3], memory[4], memory[5], memory[6]))
            elif val[0] == "Head/Touch/Top" and val[1] == False:
                self.StiffRArm(0)


        # Subscribe again to the event
        self.memory.subscribeToEvent(
            "TouchChanged",
            "NaoWrapper",
            "onTouched")



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


    """
        -------------------MAP FUNCTIONS-----------------------------
    """

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

    """
        -------------------READ FUNCTIONS-----------------------------
    """

    """
        Outputs 3 vectore in 0-1 format: command readings, sensed readings and error.
        LINK: http://doc.aldebaran.com/1-14/naoqi/motion/control-joint-api.html
    """
    def ReadJoints(self):
        # VALUES FROM ACTUATORS -> THE DESIRED POSITION
        actuator = self.motionProxy.getAngles(self.body_parts, False)
        # GET AVERAGE OF RIGHT AND LEFT HIP
        actuator[6] =   (self.motionProxy.getAngles("LHipPitch", False)[0]  \
                        + self.motionProxy.getAngles("RHipPitch", False)[0])/ 2
        print("Len before: ", len(actuator))
        actuator = np.around(self.ToAction(actuator), 5)
        print("Len after: ", len(actuator))
        # VALUES FROM SENSORS -> THE ACTUAL POSITION
        sensor = self.motionProxy.getAngles(self.body_parts, True)
        # GET AVERAGE OF RIGHT AND LEFT HIP
        sensor[6] = (self.motionProxy.getAngles("LHipPitch", True)[0]   \
                    + self.motionProxy.getAngles("RHipPitch", True)[0]) / 2
        sensor = np.around(self.ToAction(sensor), 5)
        # ERROR BETWEEN THE 2 VALUES
        errors = np.around([(command - sensed) for command, sensed in zip(actuator, sensor)], 5)

        return actuator, sensor, errors


    """
        Read Memory values
        LINK: http://doc.aldebaran.com/2-1/family/nao_dcm/actuator_sensor_names.html#lhippitch
    """
    def ReadMemory(self):
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
        for body_part in self.body_parts:
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

    """
        -------------------MOVE FUNCTIONS-----------------------------
    """

    """
        USE ACTION VECTOR TO MOVE
        LINK: http://doc.aldebaran.com/1-14/naoqi/motion/control-joint.html
    """
    def Move(self, action_vector):
        rad_vector = action_vector
        # READ VALUES JOINTS
        pos_act, pos_sens, _, _, _, _, _ = self.ReadMemory()
        # GET RADIANS TO COMMAND
        # rad_vector, pos_sens = list(self.ToRadians(action_vector)), list(self.ToRadians(pos_sens))
        rad_vector, pos_sens = np.around(rad_vector, 4), np.around(pos_sens, 4)

        to_move =    [i for i, (sen,act) in enumerate(zip(pos_sens, rad_vector)) \
                        if round(sen,2) != round(act,2)]
        if to_move:
            # WE HAVE THINGS TO MOVE
            parts_move = [self.body_parts[move] for move in to_move]
            print("Parts to move: ",parts_move)
            rad_move = [round(rad_vector[move],3) for move in to_move]
            # SET STIFFNESS
            [self.motionProxy.setStiffnesses(body_part, 1.0) for body_part in parts_move]
            # NON-BLOCKING CALL:
            # task is created in a parallel thread. This enables you to do other work at the same time
            # LINK: http://doc.aldebaran.com/1-14/dev/naoqi/index.html
            # self.motionProxy.setAngles(parts_move, rad_move, self.speed)
            self.motionProxy.angleInterpolationWithSpeed(parts_move, rad_move, self.speed)
            # print(rad_vector, action_vector, pos_sens)
            # print(parts_move, rad_move)

            return True

        else:
            # NOTHING TO MOVE
            print("No move!")

            return False

    """
        -------------------STIFFNESS FUNCTIONS-----------------------------
    """
    def StiffRArm(self, stiffness):
        self.motionProxy.setStiffnesses(self.body_parts[8:13], stiffness)
        print("Stiffness RArm set to ",stiffness)
        return

if __name__ == "nao_class":
    print("\n-------Thank you for using my library!------- by George M.\n")
