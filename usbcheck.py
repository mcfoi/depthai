from pathlib import Path
from time import sleep
from time import perf_counter_ns
import time
import depthai as dai
import cv2

pipeline = dai.Pipeline()

with dai.Device(pipeline) as device:
    # Print MxID, USB speed, and available cameras on the device
    print('MxId:', device.getDeviceInfo().getMxId())
    print('USB speed:', device.getUsbSpeed())
    print('Connected cameras:', device.getConnectedCameras())