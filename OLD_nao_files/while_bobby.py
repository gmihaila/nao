from nao_class_wrapper_V6 import NaoRobot
import numpy as np
import time

IP = "10.125.190.173"
dataFile = "actual_data_points_final.pickle"

bobby = NaoRobot(IP, dataFile)

while(1):
    if (bobby.TemperatureSensor() < 0):
        break

    movement = np.random.random(13)
    # movement[6] = 0.76

    bobby.Move(movement, False)
    if(bobby.UtilityFunction(bobby.ReadJoints()[1]) < 0):
        print 'RESET POSITION ---------> END EPISODE!'
        bobby.EpisodeRestart()
