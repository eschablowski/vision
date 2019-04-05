import cv2
from grip import Vision
from picamera import PiCamera
from picamera.array import PiRGBArray
from frc import server, team
from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
from networktables import NetworkTablesInstance
import ntcore

vision_processor = Vision()

expected_area = 50 * 100

def process(img):
    vision_processor.process(img) # use grip processing
    contours = vision_processor.filter_contours_output # get grip output
    boxes = [[9999], [0]] # store corners in array
    for contour in contours: # loop through countour pts
        x,y,w,h = cv2.boundingRect(contour) # establish contour pts
        if(x > boxes[1][0]):
            boxes[1] = [x, w, h]
        elif(x < boxes[0][0]):
            boxes[1] = [x, w, h]
    for i in range(0, len(boxes)):
        boxes[i] = [boxes[i][1], boxes[i][2]] # remove x coordinate

    ratios = []
    for box in boxes: # loop through corner pts
        area = boxes[0] * boxes[1]
        ratios.append((expected_area - area) / expected_area)

    return ratios

ntinst = NetworkTablesInstance.getDefault()
if server:
    print("Setting up NetworkTables server")
    ntinst.startServer()
else:
    print("Setting up NetworkTables client for team {}".format(team))
    ntinst.startClientTeam(team)
sd = ntinst.getTable("VisionData")
with PiCamera() as camera:
    # initialize the camera and grab a reference to the raw camera capture
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))
    while True:
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            # grab the raw NumPy array representing the image, then initialize the timestamp
            # and occupied/unoccupied text
            image = frame.array
        
            # process the frame
            ratios = process(image)

            sd.putNumber("left", ratios[0])
            sd.putNumber("right", ratios[1])


            # clear the stream in preparation for the next frame
            rawCapture.truncate(0)
