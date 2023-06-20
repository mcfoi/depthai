#!/usr/bin/env python3

from datetime import timedelta
import os
from pathlib import Path
from threading import Thread
import time
import cv2
import numpy as np
import depthai as dai

# #################################
# CONSTANTS
# Weights to use when blending depth/rgb image (should equal 1.0)
rgbWeight = 0.4
depthWeight = 0.6
fps = 2
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

'''Computes FPS using an average over 5 loops'''
def printFPS(start, loopCounter, fpsCounter):
    fpsCounter += 1
    if (fpsCounter % 5 == 0):                
        stop = time.time()
        elapsed = stop - start
        _fps =  5.0 / elapsed
        #print(f'   ### {loopCounter} ###   =>  {_fps:.1f} fps')
        print(f'   ### {loopCounter} ###')
        # reset 
        fpsCounter = 0
        start = time.time()
        return (start, fpsCounter)
    else:
        print(f'   ### {loopCounter} ###')
        return (start, fpsCounter)

def getPrintTime(imgFrame: dai.ImgFrame) -> str:
    # imgFrame: dai.ImgFrame = latestPacket["rgb"]
    imgFrameTimesdelta = imgFrame.getTimestamp()
    printTime = f"{imgFrameTimesdelta.seconds}_{imgFrameTimesdelta.microseconds}"
    return printTime

def renameDir(dirName):
    i = 1
    while(Path.exists(Path(dirName+str(i)))):
        i += 1
    os.rename(dirName, dirName+str(i))

# Make sure the destination path is present before starting to store the frames
dirName = "output"
Path(dirName).mkdir(parents=True, exist_ok=True)

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

camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_12_MP) # 4056x3040
camRgb.setFps(fps)
# Initial configration to reduce timings
camRgb.initialControl.setAntiBandingMode(dai.RawCameraControl.AntiBandingMode.OFF)
camRgb.initialControl.setAutoWhiteBalanceMode(dai.RawCameraControl.AutoWhiteBalanceMode.DAYLIGHT)
# 1/250 = 4000 Us; 1/360 = 2777 Us; 1/500 = 2000 Us; 1/1000 = 1000 Us
exposureMicrosec = 2000
exposureIso = 3200
#camRgb.initialControl.setManualExposure(exposureMicrosec, exposureIso)

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
    WINDOW_NORMAL = 0x00000000 #cv2.WINDOW_NORMAL
    WINDOW_AUTOSIZE = 0x00000001 #cv2.WINDOW_AUTOSIZE
    WINDOW_KEEPRATIO = 0x00000000 #cv2.WINDOW_KEEPRATIO
    WINDOW_GUI_EXPANDED = 0x00000000 # cv2.WINDOW_GUI_EXPANDED
    flags = WINDOW_NORMAL & WINDOW_KEEPRATIO & WINDOW_GUI_EXPANDED
    cv2.namedWindow(blendedWindowName, flags)
    cv2.createTrackbar('RGB Weight %', blendedWindowName, int(rgbWeight*100), 100, updateBlendWeights)

    loopCounter = 1
    fpsCounter = 0
    rgbFrameCount = 0
    depthFrameCount = 0
    disparityFrameCount = 0
    start = time.time()
    
    while True:
        # Check if the windows is still displayed or was closed using UI buttons
        try:
            # getWindowProperty will fail in no windows exists!
            autosize = cv2.getWindowProperty(blendedWindowName, 1)                
        except:
            # rename ouptut dir
            renameDir(dirName)
            # exit the loop
            break
        
        printTime = int(time.time_ns() / 1_000_000) # Milliseconds
        frameTimesdelta: timedelta = None
        latestPacket = {}
        latestPacket["rgb"]: dai.ImgFrame = None
        latestPacket["depth"]: dai.ImgFrame = None
        latestPacket["disparity"]: dai.ImgFrame = None

        queueEvents = device.getQueueEvents(("rgb", "depth", "disparity"))
        print(f"Events # {queueEvents}")
        for queueName in queueEvents:
            packets = device.getOutputQueue(queueName).getAll()
            if len(packets) > 0:
                latestPacket[queueName]: dai.ImgFrame = packets[-1]
                # f: dai.ImgFrame = latestPacket[queueName] 
                # frameTimesdelta = f.getTimestamp()
                # printTime = f"{frameTimesdelta.seconds}_{frameTimesdelta.microseconds}"
                

        # COLOR
        if latestPacket["rgb"] is not None:
            imgFrame: dai.ImgFrame = latestPacket["rgb"]
            printTime = getPrintTime(imgFrame)
            rgbFrame = imgFrame.getCvFrame()
            rgbFrame = cv2.resize(rgbFrame, (1248, 936), interpolation=cv2.INTER_NEAREST).astype(np.uint8)
            exposure_millisec = latestPacket["rgb"].getExposureTime().total_seconds()
            print(f'Exposure: 1/{1.0/exposure_millisec:.0f} s')
            # rgbFrame = cv2.resize(rgbFrame, (1248, 936), interpolation=cv2.INTER_NEAREST)
            # cv2.imshow(rgbWindowName, rgbFrame)            
            rgbFrameCount += 1
            fName = f"{dirName}/{printTime}_rgb_color.jpg"            
            if (shouldSave(loopCounter)):
                cv2.imwrite(fName, rgbFrame)
                # Thread(target=cvSaveFile, args=(fName, rgbFrame)).start()
                # print(f"Got rgbFrame # {rgbFrameCount}")
                pass

        # DEPTH
        if latestPacket["depth"] is not None:
            imgFrame: dai.ImgFrame = latestPacket["depth"]
            printTime = getPrintTime(imgFrame)
            depthFrame = imgFrame.getFrame()
            depthFrame = cv2.resize(depthFrame, (1248, 936), interpolation=cv2.INTER_NEAREST)
            depthFrameCount += 1
            fName = f"{dirName}/{printTime}_depth_gray.tiff"
            if (shouldSave(loopCounter)):
                cv2.imwrite(fName, depthFrame)
                # Thread(target=cvSaveFile, args=(fName, depthFrame)).start()
                # print(f"Got depthFrame # {depthFrameCount}")
                pass

        # DISPARITY
        if latestPacket["disparity"] is not None:
            imgFrame: dai.ImgFrame = latestPacket["disparity"]
            printTime = getPrintTime(imgFrame)
            disparityFrame = imgFrame.getFrame()
            maxDisparity = stereo.initialConfig.getMaxDisparity()
            disparityFrame = (disparityFrame * (255 / maxDisparity)).astype(np.uint8) # MY addition
            disparityFrame = cv2.resize(disparityFrame, (1248, 936), interpolation=cv2.INTER_LINEAR) # MY addition
            # disparityFrame = cv2.normalize(disparityFrame, None, 255, 0, cv2.NORM_MINMAX, cv2.CV_8UC1)
            # depthFrame = cv2.equalizeHist(depthFrame)
            disparityFrame = cv2.applyColorMap(disparityFrame, cv2.COLORMAP_JET)
            # cv2.imshow(depthWindowName, depthFrame)
            disparityFrameCount += 1
            # print(f"Got disparityFrame # {disparityFrameCount}")            
            fName = f"{dirName}/{printTime}_disparity_scaled_colored.jpg"
            if (shouldSave(loopCounter)):
                cv2.imwrite(fName, disparityFrame)
                # Thread(target=cvSaveFile, args=(fName, disparityFrame)).start()
                # print(f"Got disparityFrame # {disparityFrameCount}")
                pass

        # Blend when both received
        if rgbFrame is not None and disparityFrame is not None:
            # Need to have both frames in BGR format before blending
            if len(disparityFrame.shape) < 3:
                disparityFrame = cv2.cvtColor(depthFrame, cv2.COLOR_GRAY2BGR)
            blended = cv2.addWeighted(rgbFrame, rgbWeight, disparityFrame, depthWeight, 0)
            cv2.imshow(blendedWindowName, blended)
            rgbFrame = None
            disparityFrame = None
                
        # FPS
        (start, fpsCounter) = printFPS(start, loopCounter, fpsCounter)
        
        if cv2.waitKey(1) == ord('q'):
            break
        
        loopCounter += 1