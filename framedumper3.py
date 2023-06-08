#!/usr/bin/env python3

from pathlib import Path
import time
import cv2
import numpy as np
import depthai as dai

# #################################
# CONSTANTS
# Weights to use when blending depth/rgb image (should equal 1.0)
rgbWeight = 0.4
depthWeight = 0.6
autoExposureSetsAfterFrameCount = 40
# #################################


def updateBlendWeights(percent_rgb):
    """
    Update the rgb and depth weights used to blend depth/rgb image
    @param[in] percent_rgb The rgb weight expressed as a percentage (0..100)
    """
    global depthWeight
    global rgbWeight
    rgbWeight = float(percent_rgb)/100.0
    depthWeight = 1.0 - rgbWeight



def shouldSave(frameCounter: int):
    return frameCounter > autoExposureSetsAfterFrameCount

def cvSaveFile(fName: str, cvImage: object):
    cv2.imwrite(fName, cvImage)    

# Make sure the destination path is present before starting to store the frames
dirName = "output"
Path(dirName).mkdir(parents=True, exist_ok=True)

# Optional. If set (True), the ColorCamera is downscaled from 1080p to 720p.
# Otherwise (False), the aligned depth is automatically upscaled to 1080p
downscaleColor = True
fps = 10
# The disparity is computed at this resolution, then upscaled to RGB resolution
monoResolution = dai.MonoCameraProperties.SensorResolution.THE_800_P

# Create pipeline
pipeline = dai.Pipeline()
device = dai.Device()

# Define sources and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
left = pipeline.create(dai.node.MonoCamera)
right = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)

rgbOut = pipeline.create(dai.node.XLinkOut)
depthOut = pipeline.create(dai.node.XLinkOut)
disparityOut = pipeline.create(dai.node.XLinkOut)

rgbOut.setStreamName("rgb")
depthOut.setStreamName("depth")
disparityOut.setStreamName("disparity")

#Properties
camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_12_MP) # 4056x3040

# CALIBRATION : NOT REQUIRED IN FIXED-FOCUS CAMERAS
try:
    calibData = device.readCalibration2()
    lensPosition = calibData.getLensPosition(dai.CameraBoardSocket.RGB)
    if lensPosition:
        camRgb.initialControl.setManualFocus(lensPosition)
except:
    raise

left.setResolution(monoResolution)
left.setBoardSocket(dai.CameraBoardSocket.LEFT)
left.setFps(fps)
right.setResolution(monoResolution)
right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
right.setFps(fps)

stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
# LR-check is required for depth alignment
stereo.setLeftRightCheck(True)
# ExtendedDisparity(True) = Closer-in minimum depth, disparity range is doubled:
stereo.setExtendedDisparity(True)
# Subpixel(True) = Better accuracy for longer distance, fractional disparity 32-levels:
stereo.setSubpixel(True)
# Alignment
stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
# # 4056x3040
stereo.setOutputSize(1248, 936)

# Linking
camRgb.isp.link(rgbOut.input)
left.out.link(stereo.left)
right.out.link(stereo.right)
stereo.depth.link(depthOut.input)
stereo.disparity.link(disparityOut.input)


counter4AutoExposure = 1
start = time.time()
# Connect to device and start pipeline
with device:
    device.startPipeline(pipeline)
    # Print MxID, USB speed, and available cameras on the device
    print('MxId:', device.getDeviceInfo().getMxId())
    print('USB speed:', device.getUsbSpeed())
    print('Connected cameras:', device.getConnectedCameras())

    rgbFrame = None
    depthFrame = None
    disparityFrame = None

    # Configure windows; trackbar adjusts blending ratio of rgb/depth
    rgbWindowName = "rgb"
    depthWindowName = "depth"
    disparityWindowName = "disparity"
    blendedWindowName = "rgb-depth"
    # cv2.namedWindow(rgbWindowName)
    # cv2.namedWindow(depthWindowName)
    cv2.namedWindow(blendedWindowName)
    cv2.createTrackbar('RGB Weight %', blendedWindowName, int(rgbWeight*100), 100, updateBlendWeights)

    rgbFrameCount = 0
    depthFrameCount = 0
    disparityFrameCount = 0
    
    while True:
        
        osTimes = int(time.time_ns() / 1_000_000) # Milliseconds
        
        latestPacket = {}
        latestPacket["rgb"] = None
        latestPacket["depth"] = None
        latestPacket["disparity"] = None

        queueEvents = device.getQueueEvents(("rgb", "depth", "disparity"))
        for queueName in queueEvents:
            packets = device.getOutputQueue(queueName).tryGetAll()
            if len(packets) > 0:
                latestPacket[queueName] = packets[-1]            
            counter4AutoExposure += 1

        if latestPacket["rgb"] is not None:
            rgbFrame = latestPacket["rgb"].getCvFrame()
            rgbFrame = cv2.resize(rgbFrame, (1248, 936), interpolation=cv2.INTER_NEAREST)
            # cv2.imshow(rgbWindowName, rgbFrame)
            rgbFrameCount += 1
            print(f"Got rgbFrame # {rgbFrameCount}")

        if latestPacket["depth"] is not None:
            depthFrame = latestPacket["depth"].getFrame()
            depthFrameCount += 1
            print(f"Got depthFrame # {depthFrameCount}")

        if latestPacket["disparity"] is not None:
            disparityFrame = latestPacket["disparity"].getFrame()
            disparityFrame = cv2.normalize(disparityFrame, None, 255, 0, cv2.NORM_MINMAX, cv2.CV_8UC1)
            # depthFrame = cv2.equalizeHist(depthFrame)
            disparityFrame = cv2.applyColorMap(disparityFrame, cv2.COLORMAP_JET)
            # cv2.imshow(depthWindowName, depthFrame)
            disparityFrameCount += 1
            print(f"Got disparityFrame # {disparityFrameCount}")            
            fName = f"{dirName}/{osTimes}_disparity_scaled_colored.png"
            if (shouldSave(counter4AutoExposure)):
                # cv2.imwrite(fName, disparityCvImageScaledColored)
                Thread(target=cvSaveFile, args=(fName, disparityCvImageScaledColored)).start()

        # Blend when both received
        # if rgbFrame is not None and disparityFrame is not None:
        #     # Need to have both frames in BGR format before blending
        #     if len(disparityFrame.shape) < 3:
        #         disparityFrame = cv2.cvtColor(depthFrame, cv2.COLOR_GRAY2BGR)
        #     blended = cv2.addWeighted(rgbFrame, rgbWeight, disparityFrame, depthWeight, 0)
        #     cv2.imshow(blendedWindowName, blended)
        #     rgbFrame = None
        #     disparityFrame = None

        if cv2.waitKey(1) == ord('q'):
            break