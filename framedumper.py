from pathlib import Path
from time import sleep
from time import perf_counter_ns
import time
import depthai as dai
import cv2
import numpy as np

globalFPS = 2.0 # float(1.0/10.0)
monoCameraResolution = dai.MonoCameraProperties.SensorResolution.THE_800_P
colorCameraResolution = dai.ColorCameraProperties.SensorResolution.THE_800_P


def captureStillRGB(colorCameraControl_In):
    ctrl = dai.CameraControl()
    ctrl.setCaptureStill(True)
    colorCameraControl_In.send(ctrl)
    print("Sent 'still' event to the camera!")


# Make sure the destination path is present before starting to store the frames
dirName = "output"
Path(dirName).mkdir(parents=True, exist_ok=True)

pipeline = dai.Pipeline()

# #############################
# DEPTH

# Create MonoCamera nodes
monoRight = pipeline.create(dai.node.MonoCamera)
monoRight.setResolution(monoCameraResolution)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
monoRight.setFps(globalFPS)
# 3A algorithms (auto exposure, auto white balance and auto focus) won’t be run every frame, but every 15 frames
#monoRight.setIsp3aFps(15)

monoLeft = pipeline.create(dai.node.MonoCamera)
monoLeft.setResolution(monoCameraResolution)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoLeft.setFps(globalFPS)
# 3A algorithms (auto exposure, auto white balance and auto focus) won’t be run every frame, but every 15 frames
#monoLeft.setIsp3aFps(15)

# Create StereoDepth node
stereoDepth = pipeline.create(dai.node.StereoDepth)
stereoDepth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)

# https://docs.luxonis.com/projects/api/en/latest/components/nodes/stereo_depth/#currently-configurable-blocks

# To prioritize fill-rate, sets Confidence threshold to 245
# To prioritize accuracy, sets Confidence threshold to 200 .. ..or use: stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
threshold = 200 # Can range between 0:max confidence - 255:min confidence
stereoDepth.initialConfig.setConfidenceThreshold(threshold)
#stereoDepth.initialConfig.setDisparityShift(25) # This will reduce MazZ to ~140 cm
# LeftRightCheck(True) = Better handling for occlusions but has a performance cost
stereoDepth.setLeftRightCheck(True)
# ExtendedDisparity(True) = Closer-in minimum depth, disparity range is doubled:
stereoDepth.setExtendedDisparity(True)
# Subpixel(True) = Better accuracy for longer distance, fractional disparity 32-levels:
stereoDepth.setSubpixel(False)

stereoDepth.setRectifyEdgeFillColor(0)  # Black, to better see the cutout from rectification (black stripe on the edges)

# Link MonoCameras to StereoCamera node without creating explicitly a XLinkOut for each of the mono cameras
monoRight.out.link(stereoDepth.right)
monoLeft.out.link(stereoDepth.left)

# Create a XLinkOut and link it to StereoCamera output, both for DEPTH and DISPARITY
stereoDepth_Out_Depth = pipeline.createXLinkOut()
stereoDepth_Out_Depth.setStreamName("stereoDepth_Out_Depth")
stereoDepth.depth.link(stereoDepth_Out_Depth.input)
stereoDepth_Out_Disparity = pipeline.createXLinkOut()
stereoDepth_Out_Disparity.setStreamName("stereoDepth_Out_Disparity")
stereoDepth.disparity.link(stereoDepth_Out_Disparity.input)


# #############################
# COLOR

# Create ColorCamera node
colorCamera = pipeline.create(dai.node.ColorCamera)
colorCamera.setResolution(colorCameraResolution)
colorCamera.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
colorCamera.setFps(globalFPS)

# Create CameraColor Control Node
colorCameraControl_In = pipeline.create(dai.node.XLinkIn)
colorCameraControl_In.setStreamName("colorCameraControl_In")
colorCameraControl_In.out.link(colorCamera.inputControl)

# Create encoder node for RGB frame encoding
colorCameraEncoder = pipeline.create(dai.node.VideoEncoder)
colorCameraEncoder.setDefaultProfilePreset(1, dai.VideoEncoderProperties.Profile.MJPEG)
colorCamera.still.link(colorCameraEncoder.input)

colorCameraEncoder_Out = pipeline.createXLinkOut()
colorCameraEncoder_Out.setStreamName("colorCameraEncoder_Out")
colorCameraEncoder.bitstream.link(colorCameraEncoder_Out.input)

'''
# Create explicit 'XLinkOut' for each Mono camera node and attach it
monoRightXlinkOut = pipeline.create(dai.node.XLinkOut) # The out of a XLinkOut is a Queue!
monoRightXlinkOut.setStreamName("monoRightXlinkOut")
monoRight.out.link(monoRightXlinkOut.input)
monoLeftXlinkOut = pipeline.create(dai.node.XLinkOut) # The out of a XLinkOut is a Queue!
monoLeftXlinkOut.setStreamName("monoLeftXlinkOut")
monoRight.out.link(monoLeftXlinkOut.input)

# Create ImageManip node
manip = pipeline.create(dai.node.ImageManip)
manip.initialConfig.setFrameType(dai.ImgFrame.Type.RGB888p)

# Create ColorCamera node
colorCamera = pipeline.create(dai.node.ColorCamera)
colorCamera.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
#colorCamera.setInterleaved(False)
colorCamera.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
#colorCamera.setIsp3aFps(2)

# Define XLinkIn 'control' node
controlXlinkIn = pipeline.create(dai.node.XLinkIn)
controlXlinkIn.setStreamName('controlXlinkIn')
# Link output of 'control' node to inputControl of ColorCamera node
controlXlinkIn.out.link(colorCamera.inputControl)

# Create XLinkOut node
colorXlinkOut = pipeline.create(dai.node.XLinkOut)
colorXlinkOut.setStreamName('colorXlinkOut')

colorCamera.still.link(colorXlinkOut.input) # The out of a XLinkOut is a Queue!
'''

with dai.Device(pipeline) as device:
    # Print MxID, USB speed, and available cameras on the device
    print('MxId:', device.getDeviceInfo().getMxId())
    print('USB speed:', device.getUsbSpeed())
    print('Connected cameras:', device.getConnectedCameras())
    
    _colorCameraControl_In = device.getInputQueue(name="colorCameraControl_In")
    _stereoDepth_Out_Depth = device.getOutputQueue(name="stereoDepth_Out_Depth", maxSize=100, blocking=False)
    _stereoDepth_Out_Disparity = device.getOutputQueue(name="stereoDepth_Out_Disparity", maxSize=1, blocking=True)
    _colorCameraEncoder_Out = device.getOutputQueue(name="colorCameraEncoder_Out", maxSize=30, blocking=True)
    
    i=1
    while (True):
        start = time.time()
        depth = _stereoDepth_Out_Depth.tryGet()
        if depth is not None:
            # This code will run at 5 FPS
            depthCVimage = depth.getCvFrame()
            disparity = _stereoDepth_Out_Disparity.get() # blocking call, will wait until a new data has arrived
            disparityFrame = disparity.getFrame()
            # Normalization for better visualization
            maxDisparity = stereoDepth.initialConfig.getMaxDisparity()
            disparityFrameScaled = (disparityFrame * (255 / maxDisparity)).astype(np.uint8)
            # cv2.imshow("disparity", disparityFrameScaled)
            # Available color maps: https://docs.opencv.org/3.4/d3/d50/group__imgproc__colormap.html
            disparityFrameScaledColored = cv2.applyColorMap(disparityFrameScaled, cv2.COLORMAP_JET)
            cv2.imshow("disparity_color", disparityFrameScaledColored)
            
            stop = time.time()
            elapsed = stop - start
            if elapsed > 0:
                print(f"{i} Dequeued DEPTH frame: {(1_000_000_000/elapsed):.1f} FPS")
            i+=1
            
            # Save RGB still frame
            if _colorCameraEncoder_Out.has():
                fName = f"{dirName}/{int(time.time() * 1000)}.jpeg"
                with open(fName, "wb") as f:
                    f.write(_colorCameraEncoder_Out.get().getData())
                    print('Image saved to', fName)
            else:
                captureStillRGB(_colorCameraControl_In)
    # cv2.imshow("Depth", depthCVimage)


'''
configInNode = pipeline.create(dai.node.XLinkIn)
ispOutNode = pipeline.create(dai.node.XLinkOut)
videoOutNode = pipeline.create(dai.node.XLinkOut)
stillMjpegOutNode = pipeline.create(dai.node.XLinkOut)
stillJpegOutNode = pipeline.create(dai.node.XLinkOut)

stillEncoder = pipeline.create(dai.node.VideoEncoder)
stillEncoder.setDefaultProfilePreset(1, dai.VideoEncoderProperties.Profile.MJPEG)

controlInNode.setStreamName('control')
configInNode.setStreamName('config')
ispOutNode.setStreamName('isp')
videoOutNode.setStreamName('video')
stillMjpegOutNode.setStreamName('still')
'''

'''
# Upload the pipeline to the device
with dai.Device(pipeline) as device:
    # Print MxID, USB speed, and available cameras on the device
    print('MxId:', device.getDeviceInfo().getMxId())
    print('USB speed:', device.getUsbSpeed())
    print('Connected cameras:', device.getConnectedCameras())

    device.startPipeline()
    controlInQueue = device.getInputQueue("controlInNode")

    # Create a CameraControl object
    cameraControl = dai.CameraControl()
    cameraControl.setCaptureStill(True)
    # Send a message to controlInQueue
    controlInQueue.send(cameraControl)
'''
