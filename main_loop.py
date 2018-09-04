import sys
import time
import argparse

from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule
import nao_class


IP = "10.125.204.245"
dataFile = "bobby_data"

NaoWrapper = None

def main(ip, port):
    """ Main entry point
    """
    # We need this broker to be able to construct
    # NAOqi modules and subscribe to other modules
    # The broker must stay alive until the program exists
    myBroker = ALBroker("bobby_broker",
       "0.0.0.0",   # listen to anyone
       0,           # find a free port and use it
       ip,          # parent broker IP
       port)        # parent broker port
    # myBroker.shutdown()
    # sys.exit()
    global NaoWrapper
    NaoWrapper = nao_class.NaoWrapper("NaoWrapper")

    joints = NaoWrapper.ReadJoints()
    print(NaoWrapper.body_parts)
    print("command vald: %s\nSensor value: %s\nerror: %s"%(joints[0], joints[1], joints[2]))


    try:
        """
            INFINITE LOOP
        """
        while True:

            time.sleep(1)

    except KeyboardInterrupt:
        print
        print "Interrupted by user, shutting down"
        myBroker.shutdown()
        sys.exit(0)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default=IP,
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number")
    args = parser.parse_args()
main(args.ip, args.port)
