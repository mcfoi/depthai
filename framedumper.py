import depthai as dai

pipeline = dai.Pipeline()

# Define XLinkIn 'control' node
controlInNode = pipeline.create(dai.node.XLinkIn)
controlInNode.setStreamName('controlInNode')

# Create MonoCamera nodes
monoRight = pipeline.create(dai.node.MonoCamera)
monoLeft = pipeline.create(dai.node.MonoCamera)

# Create StereoDepth node
stereo = pipeline.create(dai.node.StereoDepth)
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
stereo.setRectifyEdgeFillColor(0)  # Black, to better see the cutout from rectification (black stripe on the edges)


# Link MonoCameras to StereoCamera node
monoRight.out.link(stereo.right)
monoLeft.out.link(stereo.left)

# Create ImageManip node
manip = pipeline.create(dai.node.ImageManip)
manip.initialConfig.setFrameType(dai.ImgFrame.Type.RGB888p)

# Create ColorCamera node
colorCamera = pipeline.create(dai.node.ColorCamera)
colorCamera.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1200_P)
colorCamera.setInterleaved(False)
colorCamera.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
colorCamera.setIsp3aFps(2)

# Link output of 'control' node to inputControl of ColorCamera node
controlInNode.out.link(colorCamera.inputControl)

# Create XLinkOut node
xlinkOut = pipeline.create(dai.node.XLinkOut)
xlinkOut.setStreamName('cameraOutput')

colorCamera.isp.link(xlinkOut.input)


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

# colorCamera.setPreviewSize(300, 300)
# colorCamera.setInterleaved(False)
