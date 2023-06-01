import depthai as dai

pipeline = dai.Pipeline()

# Create MonoCamera nodes
monoRight = pipeline.create(dai.node.MonoCamera)
# monoRight = pipeline.createMonoCamera() # Same as above using factory method
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
monoLeft = pipeline.create(dai.node.MonoCamera)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)

# Create StereoDepth node
stereo = pipeline.create(dai.node.StereoDepth)

# https://docs.luxonis.com/projects/api/en/latest/components/nodes/stereo_depth/#currently-configurable-blocks

# To prioritize fill-rate, sets Confidence threshold to 245
# To prioritize accuracy, sets Confidence threshold to 200 .. ..or use: stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
threshold = 200 # Can range between 0:max confidence - 255:min confidence
stereo.initialConfig.setConfidenceThreshold(threshold)
# 
stereo.initialConfig.setDisparityShift(50) # This will reduce MazZ to ~140 cm
# LeftRightCheck(True) = Better handling for occlusions but has a performance cost
stereo.setLeftRightCheck(True)
# ExtendedDisparity(True) = Closer-in minimum depth, disparity range is doubled:
stereo.setExtendedDisparity(True)
# Subpixel(True) = Better accuracy for longer distance, fractional disparity 32-levels:
stereo.setSubpixel(True)

stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
stereo.setRectifyEdgeFillColor(0)  # Black, to better see the cutout from rectification (black stripe on the edges)


# Link MonoCameras to StereoCamera node without creating explicitly a XLinkOut for each of the mono cameras
monoRight.out.link(stereo.right)
monoLeft.out.link(stereo.left)

# Create a XLinkOut and link it to StereoCamera output
stereoXlinkOut = pipeline.createXLinkOut()
stereoXlinkOut.setStreamName("stereoXlinkOut")

# Create explicit 'XLinkOut' for each Mono camera node and attach it
monoRightXlinkOut = pipeline.create(dai.node.XLinkOut) # The out of a XLinkOut is a Queue!
monoRightXlinkOut.setStreamName("monoRightOut")
monoRight.out.link(monoRightXlinkOut.input)
monoLeftXlinkOut = pipeline.create(dai.node.XLinkOut) # The out of a XLinkOut is a Queue!
monoLeftXlinkOut.setStreamName("monoLeftOut")
monoRight.out.link(monoLeftXlinkOut.input)

# Create ImageManip node
manip = pipeline.create(dai.node.ImageManip)
manip.initialConfig.setFrameType(dai.ImgFrame.Type.RGB888p)

# Create ColorCamera node
colorCamera = pipeline.create(dai.node.ColorCamera)
colorCamera.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1200_P)
colorCamera.setInterleaved(False)
colorCamera.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
colorCamera.setIsp3aFps(2)

# Define XLinkIn 'control' node
controlInNode = pipeline.create(dai.node.XLinkIn)
controlInNode.setStreamName('controlInNode')
# Link output of 'control' node to inputControl of ColorCamera node
controlInNode.out.link(colorCamera.inputControl)

# Create XLinkOut node
colorXlinkOut = pipeline.create(dai.node.XLinkOut)
colorXlinkOut.setStreamName('cameraOut')

colorCamera.still.link(colorXlinkOut.input) # The out of a XLinkOut is a Queue!

with dai.Device(pipeline) as device:
    # Print MxID, USB speed, and available cameras on the device
    print('MxId:', device.getDeviceInfo().getMxId())
    print('USB speed:', device.getUsbSpeed())
    print('Connected cameras:', device.getConnectedCameras())
    #device.startPipeline()

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

# colorCamera.setPreviewSize(300, 300)
# colorCamera.setInterleaved(False)
