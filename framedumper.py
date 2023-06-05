from datetime import timedelta
from pathlib import Path
from time import sleep
from time import perf_counter_ns
import time
import depthai as dai
import cv2
import numpy as np

globalFPS = 5.0 # float(1.0/10.0)
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
threshold = 255 # Can range between 0:max confidence - 255:min confidence
stereoDepth.initialConfig.setConfidenceThreshold(threshold)
# Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
stereoDepth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
#stereoDepth.initialConfig.setDisparityShift(25) # This will reduce MaxZ to ~140 cm
# LeftRightCheck(True) = Better handling for occlusions but has a performance cost
stereoDepth.setLeftRightCheck(True)
# ExtendedDisparity(True) = Closer-in minimum depth, disparity range is doubled:
stereoDepth.setExtendedDisparity(True)
# Subpixel(True) = Better accuracy for longer distance, fractional disparity 32-levels:
stereoDepth.setSubpixel(True)

# https://docs.luxonis.com/projects/api/en/latest/samples/StereoDepth/rgb_depth_aligned/
stereoDepth.setDepthAlign(dai.CameraBoardSocket.RGB)

# stereoDepth.setRectifyEdgeFillColor(0)  # Black, to better see the cutout from rectification (black stripe on the edges)

# Link MonoCameras to StereoCamera node without creating explicitly a XLinkOut for each of the mono cameras
monoRight.out.link(stereoDepth.right)
monoLeft.out.link(stereoDepth.left)

# Create a XLinkOut and link it to StereoCamera output, both for DEPTH and DISPARITY
stereoDepth_Depth_Out = pipeline.createXLinkOut()
stereoDepth_Depth_Out.setStreamName("stereoDepth_Depth_Out")
stereoDepth.depth.link(stereoDepth_Depth_Out.input)
stereoDepth_Disparity_Out = pipeline.createXLinkOut()
stereoDepth_Disparity_Out.setStreamName("stereoDepth_Disparity_Out")
stereoDepth.disparity.link(stereoDepth_Disparity_Out.input)

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
    
    colorCameraControl_InQueue: dai.DataInputQueue = device.getInputQueue(name="colorCameraControl_In")
    stereoDepth_Depth_OutQueue: dai.DataOutputQueue = device.getOutputQueue(name="stereoDepth_Depth_Out", maxSize=1, blocking=False)
    stereoDepth_Disparity_OutQueue: dai.DataOutputQueue = device.getOutputQueue(name="stereoDepth_Disparity_Out", maxSize=1, blocking=True)
    colorCameraEncoder_OutQueue: dai.DataOutputQueue = device.getOutputQueue(name="colorCameraEncoder_Out", maxSize=1, blocking=True)
    
    # cv2.namedWindow("Preview")
    
    i=1
    while (True):
        start = time.time()
        depthImg: dai.ImgFrame = stereoDepth_Depth_OutQueue.tryGet()
        if depthImg is not None:
            # This code will run at 5 FPS
            captureStillRGB(colorCameraControl_InQueue)  
            
            osTimes = int(time.time_ns() / 1_000_000) # Milliseconds
            disparityImg: dai.ImgFrame = stereoDepth_Disparity_OutQueue.get() # blocking call, will wait until a new data has arrived
            # Extracted just to test if they are synchronized: (they are!)
            # depthImgTimedelta:timedelta = depthImg.getTimestampDevice()
            # disparityImgTimedelta:timedelta = disparityImg.getTimestampDevice()
            
            # DEPTH
            # Get the uint16 array
            depthCVimage = depthImg.getCvFrame() # A CVImage is actually just an array of row-arrays, each row being a single array of column-values.
            fName = f"{dirName}/{osTimes}_depth_gray.tiff"
            cv2.imwrite(fName, depthCVimage)
            
            # Depth as numpy.ndarray
            # maxDepth = np.max(depthCVimage)
            # maxFigures = len(str(maxDepth))
            # fName = f"{dirName}/{osTimes}_depth_array.gz"
            # np.savetxt(fName, depthCVimage, delimiter=',', fmt=f'%{maxFigures}.0u')
            
            # Unique depth values
            # fName = f"{dirName}/{osTimes}_depth_unique_values.txt"
            # unique_depth_values_mm = np.unique(depthCVimage, return_counts=True)
            # unique_depth_values_mm = np.transpose(unique_depth_values_mm)
            # np.savetxt(fName, unique_depth_values_mm, delimiter=',', fmt=f'%{maxFigures}.0u')
            
            # maxDepth = np.max(depthCVimage[depthCVimage != np.max(depthCVimage)])
            maxDepth = np.max(depthCVimage)
            minDepth = np.min(depthCVimage[np.nonzero(depthCVimage)])
            print(f"{osTimes} => Max DEPTH : {maxDepth/10:.0f}cm | Min DEPTH : {minDepth/10:.0f}cm")
            # depthyCvImageScaled = (depthCVimage * (255 / maxDepth)).astype(np.uint8)
            # fName = f"{dirName}/{osTimes}_depth_scaled_gray.png"
            # cv2.imwrite(fName, depthyCvImageScaled)
            
            # DISPARITY
            disparityCvImage = disparityImg.getCvFrame() # A Frame is an 'object' with data in uint8
            fName = f"{dirName}/{osTimes}_disparity_gray.png"
            # cv2.imwrite(fName, disparityCvImage)
            # disparityFrame = disparityImg.getFrame() # A Frame is actually just an 'numpy.ndarray'
            # Normalize for better visualization
            maxDisparity = stereoDepth.initialConfig.getMaxDisparity()
            disparityCvImageScaled = (disparityCvImage * (255 / maxDisparity)).astype(np.uint8)
            # fName = f"{dirName}/{osTimes}_disparity_scaled_gray.png"
            # cv2.imwrite(fName, disparityCvImageScaled)            
            # Available color maps: https://docs.opencv.org/3.4/d3/d50/group__imgproc__colormap.html
            disparityCvImageScaledColored = cv2.applyColorMap(disparityCvImageScaled, cv2.COLORMAP_JET)
            fName = f"{dirName}/{osTimes}_disparity_scaled_colored.png"
            cv2.imwrite(fName, disparityCvImageScaledColored)
            
            # try:
            #     if (disparityCvImageScaledColored is not None):
            #         cv2.imshow("Preview", disparityCvImageScaledColored)
            #         pass
            # except Exception as e:
            #     cv2.destroyAllWindows()
            
            stop = time.time()
            elapsed = stop - start
            # if elapsed > 0:
            #     print(f"{i} Dequeued DEPTH frame: {(1_000_000_000/elapsed):.1f} FPS")
            #     pass
            # i+=1
            
            # Save RGB still frame
            if colorCameraEncoder_OutQueue.has():
                fName = f"{dirName}/{osTimes}_rgb_color.jpg"
                colorImg: dai.ImgFrame = colorCameraEncoder_OutQueue.get()
                colorCvFrame = colorImg.getCvFrame()
                cv2.imwrite(fName, colorCvFrame)
                with open(f"{dirName}/{osTimes}_rgb_color.png", "wb") as f:
                    f.write(colorImg.getData())
                    print('RGB Image saved to', fName)       

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
