import sys
import time
import argparse

# from naoqi import ALProxy
# from naoqi import ALBroker
from naoqi import ALModule
from nao_class import NaoWrapper


IP = "10.125.187.254"
dataFile = "bobby_data"

# NaoWrapper = None

def main(ip, port):
    """ Main entry point
    """
    my_nao = NaoWrapper(ip, "nao_data.csv")
    # my_nao.NewInitStart()
    # time.sleep(5)
    status = sum(my_nao.Fsr())


    target = [0.63642,0.28029,0.31465,0.79656,0.52901,0.0148,0.40954,0.83282,0.7234,0.6835,0.65057,0.46929,0.0148]

    # print(target, n_target)
    # NaoWrapper.PostureStandInit(0.2)
    # time.sleep(1)
    # my_nao.Rest()

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
    try:
        """
            INFINITE LOOP
        """
        while True:
            # UPDATE STATUS SENSORS
            n_front = my_nao.ReadFrontHead()
            n_top = my_nao.ReadTopHead()
            n_status = sum(my_nao.Fsr())
            variance = abs(status - n_status)

            if variance > 0.3:
                # RESTART EPISODE
                print("Table Touched at diff 0.3")
            elif variance < 0.1:
                status = n_status


            # n_lf, n_rf = my_nao.CopLF()
            n_lffl, n_lffr, n_rffl, n_rffr = my_nao.Fsr()
            # print("Avg: %s"%(sum([n_lffl, n_lffr, n_rffl, n_rffr])/4))
            # print("Sum %s"%sum([n_lffl, n_lffr, n_rffl, n_rffr]))
            # print(sum(my_nao.Fsr()))
            # print(n_lffl, n_lffr, n_rffl, n_rffr)

            # if (n_lf != lf) or (n_rf != rf):
            #     lf = n_lf; rf = n_rf
            #     print("Right Foot: %s\tLeft Foot: %s"%(rf, lf))

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
            # PREDICT ACTIONS - CALL MODEL
            n_target = my_nao.SimMov(target)
            # CHECK IF ACITON IS SIMILAR
            # target = n_target
            # EXECUTE PREDICTED ACTION
            print("Target")
            my_nao.Move(target)
            time.sleep(3)
            print("Similar")
            my_nao.Move(n_target)
            time.sleep(3)

            print(my_nao.ActionSim(target, n_target))

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
