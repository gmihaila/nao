import sys
import time
import argparse
from PIL import Image

# from naoqi import ALProxy
# from naoqi import ALBroker
from naoqi import ALModule
from nao_class import NaoWrapper


IP = "10.125.198.69"
dataFile = "bobby_data"

# NaoWrapper = None

def main(ip, port):
    """ Main entry point
    """
    my_nao = NaoWrapper(ip, "nao_data.csv")
    my_nao.FallManager(False)






    # my_nao.NewInitStart()
    status = sum(my_nao.Fsr())
    state = my_nao.ReadMemoryJoints()[1]

    target = [0.58089,0.18772,0.49815,0.99368,0.50882,0.042,0.56143,0.57908,0.80849,0.5022,0.00434,0.49116,0.7784]
    # target = [0.63642,0.28029,0.31465,0.79656,0.52901,0.0148,0.76193,0.72985,0.801,0.50551,0.00637,0.48948,0.0148]

    # print(target, n_target)
    my_nao.PostureStandInit(0.2)
    # time.sleep(1)
    my_nao.Rest()


    # joints = my_nao.ReadMemoryJoints()[1]
    # joints[0] -= 0.1
    # my_nao.speed = 0.3

    my_nao.Control()
    # my_nao.PostureStand(0.1)

    # GET STATUS OF SENSORS
    front   = my_nao.ReadFrontHead()
    rear    = my_nao.ReadRearHead()
    top     = my_nao.ReadTopHead()
    lf, rf = my_nao.CopLF()

    lffl, lffr, rffl, rffr = my_nao.Fsr()
    tcam, bcam = my_nao.CameraFunction()
    # print(tcam.shape)
    # print(bcam.shape)
    # image = Image.fromarray(tcam)
    # image.show()
    # image = Image.fromarray(bcam)
    # image.show()

    try:
        """
            INFINITE LOOP
        """
        while True:

            """GET PREDICTION FROM MODEL"""
            action = target

            """CHECK SIMILARITY"""
            # sim = my_nao.ActionSim()

            """ EXECUTE ACTION """
            # my_nao.Move(action)

            """GET IMAGES"""
            tcam, bcam = my_nao.CameraFunction()

            """GET POSITION"""
            position = my_nao.ReadMemoryJoints()[1]

            """CHECK IF HAND TOUCHED THE TABLE"""
            if sum(my_nao.Fsr()) < .01:
                print("Table Touched at diff 0.3")
                # print(round(sum(abs(state - my_nao.ReadMemoryJoints()[1]))))

            """GET SCORE USING SIMILARITY AND TOUCH_TABLE"""

            """TRAIN TO MODEL"""
            n_target = my_nao.SimMov(target)


            """UPDATE STATUS SENSORS"""
            n_front = my_nao.ReadFrontHead()
            n_top = my_nao.ReadTopHead()
            n_status = sum(my_nao.Fsr())
            variance = abs(status - n_status)

            """DETECT ANY TOUCH"""
            if n_top != top:
                top = n_top
                if top:
                    my_nao.PrintNaoData()

            if n_front != front:
                front = n_front
                if front:
                    print('Front pressed')
                else:
                    print("Relseased")



            # CHECK IF ACITON IS SIMILAR
            # target = n_target
            # EXECUTE PREDICTED ACTION
            # print("Target")
            # my_nao.Move(target)
            # time.sleep(3)
            # print("Similar")
            # my_nao.Move(n_target)
            # time.sleep(3)
            #
            # print(my_nao.ActionSim(target, n_target))

    except KeyboardInterrupt:
        print
        print "Interrupted by user, shutting down"
        # my_nao.Rest()
        sys.exit(0)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default=IP,
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number")
    args = parser.parse_args()
main(args.ip, args.port)
