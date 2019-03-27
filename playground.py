import sys
import time
import argparse
from PIL import Image

# from naoqi import ALProxy
# from naoqi import ALBroker
from naoqi import ALModule
from nao_class import NaoWrapper


IP = "10.125.198.69"
data = "nao_data.csv"

# create nao object
my_nao = NaoWrapper(IP, data)

# deactivate fall manager
my_nao.FallManager(False)


