#!/usr/bin/env python3
# Import the camera server
from cscore import CameraServer
from enum import Enum
# Import OpenCV and NumPy
import cv2
import numpy as np
import imutils
from collections import deque
from networktables import NetworkTablesInstance
from networktables import NetworkTables
import ntcore
from cscore import CameraServer
from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
import ntcore
import json,sys
from GripRedBall import GripRedBall
from GripBlueBall import GripBlueBall


def parseError(str):
    """Report parse error."""
    print("config error in '" + configFile + "': " + str, file=sys.stderr)

def readCameraConfig(config):
    """Read single camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read camera name")
        return False

    # path
    try:
        cam.path = config["path"]
    except KeyError:
        parseError("camera '{}': could not read path".format(cam.name))
        return False

    # stream properties
    cam.streamConfig = config.get("stream")

    cam.config = config

    cameraConfigs.append(cam)
    return True

def readSwitchedCameraConfig(config):
    """Read single switched camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read switched camera name")
        return False

    # path
    try:
        cam.key = config["key"]
    except KeyError:
        parseError("switched camera '{}': could not read key".format(cam.name))
        return False

    switchedCameraConfigs.append(cam)
    return True

def readConfig():
    """Read configuration file."""
    global team
    global server

    # parse file
    try:
        with open(configFile, "rt", encoding="utf-8") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # ntmode (optional)
    if "ntmode" in j:
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))

    # cameras
    try:
        cameras = j["cameras"]
    except KeyError:
        parseError("could not read cameras")
        return False
    for camera in cameras:
        if not readCameraConfig(camera):
            return False

    # switched cameras
    if "switched cameras" in j:
        for camera in j["switched cameras"]:
            if not readSwitchedCameraConfig(camera):
                return False

    return True

def startCamera(config):
    """Start running the camera."""
    print("Starting camera '{}' on {}".format(config.name, config.path))
    inst = CameraServer.getInstance()
    camera = UsbCamera(config.name, config.path)
    server = inst.startAutomaticCapture(camera=camera, return_server=True)
    camera.setConfigJson(json.dumps(config.config))
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen)

    if config.streamConfig is not None:
        server.setConfigJson(json.dumps(config.streamConfig))

    return camera,inst

def startSwitchedCamera(config):
    """Start running the switched camera."""
    print("Starting switched camera '{}' on {}".format(config.name, config.key))
    server = CameraServer.getInstance().addSwitchedCamera(config.name)

    def listener(fromobj, key, value, isNew):
        if isinstance(value, float):
            i = int(value)
            if i >= 0 and i < len(cameras):
              server.setSource(cameras[i])
        elif isinstance(value, str):
            for i in range(len(cameraConfigs)):
                if value == cameraConfigs[i].name:
                    server.setSource(cameras[i])
                    break

    NetworkTablesInstance.getDefault().getEntry(config.key).addListener(
        listener,
        ntcore.constants.NT_NOTIFY_IMMEDIATE |
        ntcore.constants.NT_NOTIFY_NEW |
        ntcore.constants.NT_NOTIFY_UPDATE)

    return server

configFile = "/boot/frc.json"
class CameraConfig: pass
team = None # Get from config
server = False
cameraConfigs = []
switchedCameraConfigs = []
cameras = []
roboRioIP = '10.58.30.2'

if __name__ == "__main__":
    if len(sys.argv) >= 2:
        configFile = sys.argv[1]

    # read configuration
    if not readConfig():
        sys.exit(1)

    # start NetworkTables
    ntinst = NetworkTablesInstance.getDefault()
    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClientTeam(team)
    first_config = None
    # start cameras
    for config in cameraConfigs:
        camera,cs = startCamera(config)
        cameras.append(camera)
        if first_config is None:
            first_config = config
    # start switched cameras
    for config in switchedCameraConfigs:
        startSwitchedCamera(config)
    cvSink = cs.getVideo()
    # First Camera for Vision Target

    gpipeline = GripRedBall()

    #cmpMask = cv2.imread('mask_1.jpg',0)
    #out = cv2.cvtColor(cmpMask, cv2.COLOR_BGR2HSV)
    
    #hue = gpipeline.__hsv_threshold_hue
    #sat = gpipeline.__hsv_threshold_saturation
    #val = gpipeline.__hsv_threshold_value
    #cntMask = cv2.inRange(out, (hue[0], sat[0], val[0]),  (hue[1], sat[1], val[1]))
    
    vCam = 0
    #import pdb; pdb.set_trace()
    outputStream = cs.putVideo("Camera %d"%vCam, 320, 240)
    #CvSource imageSource = new CvSource("CV Image Source", VideoMode.PixelFormat.kMJPEG, 640, 480, 30);
    #MjpegServer cvStream = new MjpegServer("CV Image Stream", 1186);
    #cvStream.setSource(imageSource)
    img = np.zeros(shape=(320,240, 3), dtype=np.uint8)
    NetworkTables.initialize(server=roboRioIP)
    sd = NetworkTables.getTable("SmartDashboard")
    while True:
        # Tell the CvSink to grab a frame from the camera and put it
        # in the source image.  If there is an error notify the output.
        time, img = cvSink.grabFrame(img)
        if time == 0:
            # Send the output the error.
            outputStream.notifyError(cvSink.getError());
            # skip the rest of the current iteration
            continue
        
        gpipeline.process(img)
        
        cnts = gpipeline.filter_contours_output
        circles = cv.HoughCircles(gpipeline.hsv_threshold_output,cv.HOUGH_GRADIENT,1,20,
                            param1=50,param2=30,minRadius=20,maxRadius=120)
        #cnts = gpipeline.find_blobs_output
        # (x, y) center of the target
        imgWidth = float(first_config.config['width'])
        imgHeight = float(first_config.config['height'])
        # only proceed if at least one contour was found
        #if len(cnts) > 0:
        #    cnts = gpipeline.filter_contours_output
        center = None
        # only proceed if at least one contour was found
        score = []
        if len(cnts) > 0:
            #import pdb; pdb.set_trace()
            for c in cnts:
                #score.append(cv2.matchShapes(cntMask[0],cnt,1,0.0))
                #c = cnts[score.index(min(score))]
                #if len(score) > 0 and min(score) < 10:
                # Find furthest left, right points of largest contour
                leftmost = tuple(c[c[:,:,0].argmin()][0])
                rightmost = tuple(c[c[:,:,0].argmax()][0])
                x = (leftmost[0] + rightmost[0])/2 
                y = (leftmost[1] + rightmost[1])/2
                length = rightmost[0] -leftmost[0]
                cv2.circle(img, (int(x), int(y)), int(10),(0, 255, 255), 2) 
                print('X: %0.2f Y: %0.2f Length %0.2f '%(x,y,length))
        else:
            print('Nothing found')
            """
            score = []
            for cnt in cnts:
                 biggest = 0
                 if ~isinstance(cnt,list) and  cnt.size > biggest:
                      out = cnt 
                      biggest = cnt.size
                      x = cnt.pt[0]
                      y = cnt.pt[1]
             
            cv2.circle(img, (int(x), int(y)), int(10),(0, 255, 255), 2)  
            sd.putNumber("xPosition",int(x))
            sd.putNumber("yPosition",int(y))
            sd.putNumber("length",biggest)
            #biggest
            print('%f, %f, %d'%(float(float(x-(biggest/2))/(biggest/2)),float(-float(y-(biggest/2))/(biggest/2)),int(biggest)))
            """   
        outputStream.putFrame(img)
