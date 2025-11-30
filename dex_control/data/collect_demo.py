# we follow droid setup, it can be saved as a droid dataset format
# we also provide script to convert the data to lerobot dataset format
# This should be run on the client computer side, and the data should be saved to the client computer side as well.
# all data should include state, action, camera, and other sensor data.
# also include the timestamp of the data.

import os
import numpy as np
import json
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import droid_utils


def collect_demo():
    pass