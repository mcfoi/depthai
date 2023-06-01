from pathlib import Path
from time import sleep
from time import perf_counter_ns
import time
import depthai as dai
import cv2

globalFPS = 5.0
monoCameraResolution = dai.MonoCameraProperties.SensorResolution.THE_800_P
colorCameraResolution = dai.ColorCameraProperties.SensorResolution.THE_800_P


def captureStillRGB(rgbControlQueue):
    ctrl = dai.CameraControl()
    ctrl.setCaptureStill(True)
    rgbControlQueue.send(ctrl)
    print("Sent 'still' event to the camera!")


# Make sure the destination path is present before starting to store the examples
dirName = "output"
Path(dirName).mkdir(parents=True, exist_ok=True)

pipeline = dai.Pipeline()

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

# https://docs.luxonis.com/projects/api/en/latest/components/nodes/stereo_depth/#currently-configurable-blocks

# To prioritize fill-rate, sets Confidence threshold to 245
# To prioritize accuracy, sets Confidence threshold to 200 .. ..or use: stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
threshold = 200 # Can range between 0:max confidence - 255:min confidence
stereoDepth.initialConfig.setConfidenceThreshold(threshold)
# 
#stereoDepth.initialConfig.setDisparityShift(25) # This will reduce MazZ to ~140 cm
# LeftRightCheck(True) = Better handling for occlusions but has a performance cost
stereoDepth.setLeftRightCheck(True)
# ExtendedDisparity(True) = Closer-in minimum depth, disparity range is doubled:
stereoDepth.setExtendedDisparity(True)
# Subpixel(True) = Better accuracy for longer distance, fractional disparity 32-levels:
stereoDepth.setSubpixel(True)

stereoDepth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
stereoDepth.setRectifyEdgeFillColor(0)  # Black, to better see the cutout from rectification (black stripe on the edges)

# Link MonoCameras to StereoCamera node without creating explicitly a XLinkOut for each of the mono cameras
monoRight.out.link(stereoDepth.right)
monoLeft.out.link(stereoDepth.left)

# Create a XLinkOut and link it to StereoCamera output
stereoXlinkOut_Depth = pipeline.createXLinkOut()
stereoXlinkOut_Depth.setStreamName("stereoXlinkOut_Depth")
stereoDepth.depth.link(stereoXlinkOut_Depth.input)

# Create encoder node for frame encoding
videoEncoder = pipeline.create(dai.node.VideoEncoder)
videoEncoder.setDefaultProfilePreset(1, dai.VideoEncoderProperties.Profile.MJPEG)
stereoDepth.depth.link(videoEncoder.input)

videoEncoderXlinkOut = pipeline.createXLinkOut()
videoEncoderXlinkOut.setStreamName("videoEncoderXlinkOut")
videoEncoder.bitstream.link(videoEncoderXlinkOut.input)

# Create ColorCamera node
colorCamera = pipeline.create(dai.node.ColorCamera)
colorCamera.setResolution(colorCameraResolution)
colorCamera.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
colorCamera.setFps(globalFPS)

# Create CameraColor Control Node
colorCameraControlXlinkOut = pipeline.create(dai.node.XLinkIn)
colorCameraControlXlinkOut.setStreamName("colorCameraControlXlinkOut")
colorCameraControlXlinkOut.out.link(colorCamera.inputControl)

# Create encoder node for RGB frame encoding
videoEncoderRGB = pipeline.create(dai.node.VideoEncoder)
videoEncoderRGB.setDefaultProfilePreset(1, dai.VideoEncoderProperties.Profile.MJPEG)
colorCamera.still.link(videoEncoderRGB.input)

videoEncoderRGBXlinkOut = pipeline.createXLinkOut()
videoEncoderRGBXlinkOut.setStreamName("videoEncoderRGBXlinkOut")
videoEncoderRGB.bitstream.link(videoEncoderRGBXlinkOut.input)

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
    
    depthQueue = device.getOutputQueue(name="stereoXlinkOut_Depth", maxSize=100, blocking=False)
    rgbStillQueue = device.getOutputQueue(name="videoEncoderRGBXlinkOut", maxSize=30, blocking=True)
    rgbControlQueue = device.getInputQueue(name="colorCameraControlXlinkOut")
    
    i=1
    while (True):
        start = time.time()
        depthFrame = depthQueue.tryGet()
        if depthFrame is not None:
            # This code will run at 5 FPS
            depthCVimage = depthFrame.getCvFrame()
            
            stop = time.time()
            elapsed = stop - start
            if elapsed > 0:
                print(f"{i} Dequeued DEPTH frame: {(1_000_000_000/elapsed):.1f} FPS")
            i+=1
            
            # Save RGB still frame
            if rgbStillQueue.has():
                fName = f"{dirName}/{int(time.time() * 1000)}.jpeg"
                with open(fName, "wb") as f:
                    f.write(rgbStillQueue.get().getData())
                    print('Image saved to', fName)
            else:
                captureStillRGB(rgbControlQueue)
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
