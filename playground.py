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

