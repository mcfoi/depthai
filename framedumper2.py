from datetime import timedelta
from pathlib import Path
from threading import Thread
from time import sleep
from time import perf_counter_ns
import time
import depthai as dai
import cv2
import numpy as np

# #################################
# CONSTANTS
globalFPS = 5.0 # float(1.0/10.0)
monoCameraResolution = dai.MonoCameraProperties.SensorResolution.THE_800_P # MAX for MonoCamera is THE_800_P
colorCameraResolution = dai.ColorCameraProperties.SensorResolution.THE_800_P
width:int = 1280
height:int = 800
autoExposureSetsAfterFrameCount = 40
# #################################

captureColorStill = dai.CameraControl().setCaptureStill(True)

def request_StillImage_to_ColorCamera(colorCameraControl_In: dai.DataInputQueue):
    colorCameraControl_In.send(captureColorStill)
    # print("Sent 'still' event to the camera!")

def shouldSave(frameCounter: int):
    return frameCounter > autoExposureSetsAfterFrameCount

def cvSaveFile(fName: str, cvImage: object):
    cv2.imwrite(fName, cvImage)

# Make sure the destination path is present before starting to store the frames
dirName = "output"
Path(dirName).mkdir(parents=True, exist_ok=True)

pipeline = dai.Pipeline()


# #############################
# COLOR

# Create ColorCamera node
colorCamera = pipeline.create(dai.node.ColorCamera)
# colorCamera.setResolution(colorCameraResolution)
colorCamera.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
colorCamera.setFps(globalFPS)
#colorCamera.setStillSize(1280, 800)

# Create CameraColor Control Node
colorCameraControl_In = pipeline.create(dai.node.XLinkIn)
colorCameraControl_In.setStreamName("colorCameraControl_In")
colorCameraControl_In.out.link(colorCamera.inputControl)

# Create XLinkOut node for Color Camera Still output
colorCameraStill_Out = pipeline.create(dai.node.XLinkOut)
colorCameraStill_Out.setStreamName('colorCameraStill_Out')
colorCamera.still.link(colorCameraStill_Out.input)

# Create encoder node for RGB frame encoding
# colorCameraEncoder = pipeline.create(dai.node.VideoEncoder)
# colorCameraEncoder.setDefaultProfilePreset(1, dai.VideoEncoderProperties.Profile.MJPEG)
# colorCamera.still.link(colorCameraEncoder.input)
# # Create XLinkOut node for Color Camera Encoder output
# colorCameraEncoder_Out = pipeline.createXLinkOut()
# colorCameraEncoder_Out.setStreamName("colorCameraEncoder_Out")
# colorCameraEncoder.bitstream.link(colorCameraEncoder_Out.input)


# #############################
# DEPTH

# Create MonoCamera nodes
monoRight = pipeline.create(dai.node.MonoCamera)
monoRight.setResolution(monoCameraResolution)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
monoRight.setFps(globalFPS)
# 3A algorithms (auto exposure, auto white balance and auto focus) won’t be run every frame, but every 15 frames
# monoRight.setIsp3aFps(15)

monoLeft = pipeline.create(dai.node.MonoCamera)
monoLeft.setResolution(monoCameraResolution)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoLeft.setFps(globalFPS)
# 3A algorithms (auto exposure, auto white balance and auto focus) won’t be run every frame, but every 15 frames
# monoLeft.setIsp3aFps(15)

# Create StereoDepth node
stereoDepth = pipeline.create(dai.node.StereoDepth)
stereoDepth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)

# https://docs.luxonis.com/projects/api/en/latest/components/nodes/stereo_depth/#currently-configurable-blocks

# To prioritize fill-rate, sets Confidence threshold to 245
# To prioritize accuracy, sets Confidence threshold to 200 .. ..or use: stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
threshold = 255 # Can range between 0:max confidence - 255:min confidence
stereoDepth.initialConfig.setConfidenceThreshold(threshold)
# stereoDepth.setOutputSize(width, height) # Required to prevent depth scaling
# Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
# stereoDepth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
# stereoDepth.initialConfig.setDisparityShift(25) # This will reduce MaxZ to ~140 cm
# LeftRightCheck(True) = Better handling for occlusions but has a performance cost
# LR-check is required for depth alignment
# stereoDepth.setLeftRightCheck(True)
# ExtendedDisparity(True) = Closer-in minimum depth, disparity range is doubled:
# stereoDepth.setExtendedDisparity(True)
# Subpixel(True) = Better accuracy for longer distance, fractional disparity 32-levels:
# stereoDepth.setSubpixel(True)

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
# Create ImageManip node
manip = pipeline.create(dai.node.ImageManip)
manip.initialConfig.setFrameType(dai.ImgFrame.Type.RGB888p)
'''

with dai.Device(pipeline) as device:
    # Print MxID, USB speed, and available cameras on the device
    print('MxId:', device.getDeviceInfo().getMxId())
    print('USB speed:', device.getUsbSpeed())
    print('Connected cameras:', device.getConnectedCameras())
    
    colorCameraControl_InQueue: dai.DataInputQueue = device.getInputQueue(name="colorCameraControl_In")
    stereoDepth_Depth_OutQueue: dai.DataOutputQueue = device.getOutputQueue(name="stereoDepth_Depth_Out", maxSize=1, blocking=False)
    stereoDepth_Disparity_OutQueue: dai.DataOutputQueue = device.getOutputQueue(name="stereoDepth_Disparity_Out", maxSize=1, blocking=True)
    # colorCameraEncoder_OutQueue: dai.DataOutputQueue = device.getOutputQueue(name="colorCameraEncoder_Out", maxSize=1, blocking=True)
    colorCameraStill_OutQueue: dai.DataOutputQueue = device.getOutputQueue(name="colorCameraStill_Out", maxSize=1, blocking=True)
    
    # cv2.namedWindow("Preview")
    
    i = 1
    counter4FPS = 0
    counter4AutoExposure = 1
    start = time.time()
    while (True):
        depthImg: dai.ImgFrame = stereoDepth_Depth_OutQueue.tryGet()
        if depthImg is not None:
            # This code will run at 5 FPS
            request_StillImage_to_ColorCamera(colorCameraControl_InQueue)  
            
            osTimes = int(time.time_ns() / 1_000_000) # Milliseconds
            disparityImg: dai.ImgFrame = stereoDepth_Disparity_OutQueue.get() # blocking call, will wait until a new data has arrived
            # Extracted just to test if they are synchronized: (they are!)
            # depthImgTimedelta:timedelta = depthImg.getTimestampDevice()
            # disparityImgTimedelta:timedelta = disparityImg.getTimestampDevice()
            
            # DEPTH
            # Get the uint16 array
            depthCVimage = depthImg.getCvFrame() # A CVImage is actually just an array of row-arrays, each row being a single array of column-values.
            fName = f"{dirName}/{osTimes}_depth_gray.tiff"
            if (shouldSave(counter4AutoExposure)):
                # cv2.imwrite(fName, depthCVimage)
                Thread(target=cvSaveFile, args=(fName, depthCVimage)).start()
            
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
            
            # maxDepth = np.max(depthCVimage)
            # maxDepth = np.max(depthCVimage[depthCVimage != np.max(depthCVimage)]) # escludo il massimo valore di profindità.. ..cosa non particolarmente utile in verità
            # minDepth = np.min(depthCVimage[np.nonzero(depthCVimage)])
            # print(f"{osTimes} => Max DEPTH : {maxDepth/10:.0f}cm | Min DEPTH : {minDepth/10:.0f}cm")
            # depthyCvImageScaled = (depthCVimage * (255 / maxDepth)).astype(np.uint8)
            # fName = f"{dirName}/{osTimes}_depth_scaled_gray.png"
            # cv2.imwrite(fName, depthyCvImageScaled)
            
            # DISPARITY
            disparityCvImage = disparityImg.getCvFrame() # A Frame is an 'object' with data in uint8
            fName = f"{dirName}/{osTimes}_disparity_gray.png"
            # cv2.imwrite(fName, disparityCvImage)
            # disparityFrame = disparityImg.getFrame() # A Frame is actually just an 'numpy.ndarray'
            
            # DISPARITY - COLOURED
            # Normalize for better visualization
            maxDisparity = stereoDepth.initialConfig.getMaxDisparity()
            disparityCvImageScaled = (disparityCvImage * (255 / maxDisparity)).astype(np.uint8)
            # fName = f"{dirName}/{osTimes}_disparity_scaled_gray.png"
            # cv2.imwrite(fName, disparityCvImageScaled)            
            # Available color maps: https://docs.opencv.org/3.4/d3/d50/group__imgproc__colormap.html
            disparityCvImageScaledColored = cv2.applyColorMap(disparityCvImageScaled, cv2.COLORMAP_JET)
            fName = f"{dirName}/{osTimes}_disparity_scaled_colored.png"
            if (shouldSave(counter4AutoExposure)):
                # cv2.imwrite(fName, disparityCvImageScaledColored)
                Thread(target=cvSaveFile, args=(fName, disparityCvImageScaledColored)).start()
            
            # SHOW COLOR DISPARITY IMAGE
            # try:
            #     if (disparityCvImageScaledColored is not None):
            #         cv2.imshow("Preview", disparityCvImageScaledColored)
            #         pass
            # except Exception as e:
            #     cv2.destroyAllWindows()
            
            # DEQUEUE 'STILL' COLOR CAMERA IMAGES
            stop = time.time()
            elapsed = stop - start            
            # Save RGB still frame
            # if colorCameraEncoder_OutQueue.has():
            if colorCameraStill_OutQueue.has():
                fName = f"{dirName}/{osTimes}_rgb_color.jpg"
                # colorImg: dai.ImgFrame = colorCameraEncoder_OutQueue.get()
                colorImg: dai.ImgFrame = colorCameraStill_OutQueue.get()
                colorCvFrame = colorImg.getCvFrame()
                if (shouldSave(counter4AutoExposure)):
                    # cv2.imwrite(fName, colorCvFrame)
                    Thread(target=cvSaveFile, args=(fName, colorCvFrame)).start()
                # with open(f"{dirName}/{osTimes}_rgb_color.png", "wb") as f:
                #     f.write(colorImg.getData())
                #     print('RGB Image saved to', fName)            
            counter4FPS += 1
            if (counter4FPS % 5 == 0):                
                stop = time.time()
                elapsed = stop - start
                FPS =  5.0 / elapsed
                print(f'   ### {counter4AutoExposure} ###   =>  {FPS:.1f} FPS')
                # reset 
                counter4FPS = 0
                start = time.time()
            else:
                print(f'   ### {counter4AutoExposure} ###')
            counter4AutoExposure += 1
            
            # SET ESCAPE-KEY
            key = cv2.waitKey(1)
            if key == ord(' ') or key == ord('q'):
                break

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