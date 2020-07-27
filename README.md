# Nao Python Wrapper for Machine Learning Purposes

[![GitHub license](https://img.shields.io/github/license/Naereen/StrapDown.js.svg)](https://github.com/Naereen/StrapDown.js/blob/master/LICENSE)
[![Ask Me Anything !](https://img.shields.io/badge/Updated-July,2020-1abc9c.svg)](https://GitHub.com/Naereen/ama)

See documentation **[here[(http://doc.aldebaran.com/2-1/dev/naoqi/index.html#naoqi-process)**.

This is a wrapper around the Nao Python Wrapper to encode joint positions into a vector of 0-1 values.

The purpose of this encoding si to make the Nao robot easy to train using Machine Learning algorithms.

Main class is coded into **[nao_class.py](https://github.com/gmihaila/nao/blob/master/nao_class.py)**

Each function contains a description like this:

```python
"""
        -------------------POSTURES FUNCTIONS-----------------------------
        LINK: http://doc.aldebaran.com/2-1/family/robots/postures_robot.html
"""
```
There are link to lead to the official documentaiton code used for that particlar function. It is useful to understand some fixed values coded that belong to the robots parameters.


The **[playground.py](https://github.com/gmihaila/nao/blob/master/playground.py)** contains sample code to test the **nao_class.py**:

```python
import sys
import time
import argparse
from PIL import Image
from naoqi import ALModule
from nao_class import NaoWrapper

# Nao robot IP address
IP = "10.125.200.124"
# file to dump all recorded values
data = "nao_data.csv"

# create nao object
my_nao = NaoWrapper(IP, data)

# deactivate fall manager
my_nao.FallManager(False)

# initialize position
my_nao.PostureStandInit(0.2)

# use the `Control` function which allows keyboard input to control Nao joints
my_nao.Control()

# wait for 3 seconds
time.sleep(3)

# put Nao to rest position
my_nao.Rest()
```

For the Nao robot I had available it only worked with [nao/pynaoqi/pynaoqi-python2.7-2.1.4.13-linux64.tar.gz](https://github.com/gmihaila/nao/blob/master/pynaoqi/pynaoqi-python2.7-2.1.4.13-linux64.tar.gz). To use Naoqi `from naoqi import ALModule`.

I worked on this project with the intent of train Nao robot for object grasping. After some testing I found out that the Nao robot is not apropriate to be used with Machine Learning - it is VERY unprecise! it overheats quickly and it puts a lot of wear and tear when trying to gatter movement data.

Feel free to use the code as you please.

