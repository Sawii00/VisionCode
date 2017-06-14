import cv2
import numpy as np
from networktables import NetworkTables
#import argparse
import math
########################################################

#IP address of the RoboRio
NetworkTables.setIPAddress("10.6.14.2")

#NetworkTables initialization
NetworkTables.setClientMode()
NetworkTables.initialize()
#retrieval of the correct table. The Roborio needs to use the same table
table = NetworkTables.getTable("shooterCamera")

#ap = argparse.ArgumentParser()
#ap.add_argument("-he", "--height",type=float, required=True,
 #   help="GlobalHeight")
#ap.add_argument("-a", "--angle", type=float, required=True,
#    help="Angle of the camera")
#args = vars(ap.parse_args())


#HEIGHT_ROBOT_TOPTAPE = args["height"]
#CAMERA_ANGLE = args["angle"]

#height from the camera and the top edge of the boiler's tape
HEIGHT_ROBOT_TOPTAPE =20.0
#angle the camera is mounted at
CAMERA_ANGLE = 30.0



def min(val1, val2):
    return val1 if val1<val2 else val2

def deltaAngle(yVal):
    return float((yVal -240)*34/480)

def retrieveYTopContour(contours):
    #we get the two contours and we decide which one is the top one.
    x1,y1,w1,h1 = cv2.boundingRect(contours[0])
    x2,y2,w2,h2 = cv2.boundingRect(contours[1])
    return float(min(y1,y2))

def distanceFromCamera(height, deltAngle):
    distance = height/math.tan(math.radians(CAMERA_ANGLE) - math.radians(deltAngle))
    return distance

def distanceFromCenter(midPoint):
    #we calculate the distance between the center of the camera and the
    #midpoint of the contour
    #in this case we use 320 because we are using a 640*480 resolution
    #we could also use 320 - midPoint if we wanted to have positive angles on the right
    d = midPoint - 320
    return d

def midMidPoint(contour1,contour2):
    #standard opencv midpoint detection
    M1 = cv2.moments(contour1)
    M2 = cv2.moments(contour2)
    #to avoid a division by 0 we check if those moments are null or not
    if M1['m00'] != 0 and M2['m00'] != 0:
        cx1 = int(M1['m10']/M1['m00'])
        cx2 = int(M2['m10']/M2['m00'])
        cy1 = int(M1['m01']/M1['m00'])
        cy2 = int(M2['m01']/M2['m00'])
    else:
        cx1 = 0
        cx2 = 0
        cy1 = 0
        cy2 = 0

    midX = (cx1 +cx2)/2
    midY = (cy1 +cy2)/2
    mid = (midX, midY)
    return mid


def printXY(contour, img):
    #debugging function that prints on the image values such as position of
    #edges and verious heights.
    x,y,w,h = cv2.boundingRect(contour)

    writeText(str(y),(x-10, y),img)
    writeText(str(h),(x-10, y+ h/2),img)
    writeText(str(w), (x + w/2, y), img)




def pixelToDegree(dPix):
    #after camera calibration, we found out that the focal length is 1118 pixels
    #the field of view was calculated to be 60 degrees
    #we have 640/60 pixels per degree
    
    return dPix / 10.67

def drawBoundingBox(contour, img):
    #it draws the bounding box around the contours we find
    x,y,w,h = cv2.boundingRect(contour)

    p1 = (x,y)
    p2 = ((x + w),(y+h))

    cv2.rectangle(img, p1,p2, (255,0,0))


def writeText(text, orig, img, color = (255,255,255)):
    #generic function that writes something on screen
    cv2.putText(img, text, orig, cv2.FONT_HERSHEY_SIMPLEX, 0.5,color)


##################################################################
#hsv range of detection found with GRIP
lower_red = np.array([0, 205, 90])
upper_red = np.array([180, 255, 255])

#it gets the video stream from the main camera (the number 0)
cap = cv2.VideoCapture(0)
#set the resolution
cap.set(3, 640)##widh
cap.set(4, 480)##height

#######################################################################
while True:
    #read a single frame
    _, input = cap.read()
    # RGB TO HSV
    out = cv2.cvtColor(input, cv2.COLOR_BGR2HSV)
    # threshold HSV
    mask = cv2.inRange(out, lower_red, upper_red)
    contours, hierarchy = cv2.findContours(
        mask, cv2.RETR_TREE, cv2. CHAIN_APPROX_SIMPLE)
    #we sort our contours based on their area (how big they are) and we just
    #keep the first two (we have to tapes on the boiler and they are gonna be
    #the biggest shapes on screen
    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:2]
           
            # focal length = 1118

    #we check if we have both the contours (1 is useless)
    if len(contours) == 2:

        #drawBoundingBox(contours[0], input)
        #drawBoundingBox(contours[1], input)

        #cv2.drawContours(input, [contours[0]], -1, (0, 255, 0), 2)
        #cv2.drawContours(input, [contours[1]], -1, (0, 255, 0), 2)

        #we find mid point of the two contours
        mid = midMidPoint(contours[0], contours[1])
        #we find the angle between the camera and the center of the target
        angle = pixelToDegree(distanceFromCenter(mid[0]))
        #we find the distance between camera and boiler
        distance = distanceFromCamera(HEIGHT_ROBOT_TOPTAPE, deltaAngle(retrieveYTopContour(contours)))

        #writeText("Angle: %f" % angle,(50,200), input)
        #writeText("Distance: %f" % distance,(50,300), input)

        #printXY(contours[0], input)
        #printXY(contours[1], input)

        #we write on the table those values so that roborio can access them
        table.putNumber("angle", angle)
        table.putBoolean("targetFound", True)
        table.putNumber("distance", distance)

   


    else:
        #if not both the contours are found, we ignore that
       #writeText("Nothing found",(50,300), input, (0,0,255))
       table.putBoolean("targetFound", False)
       table.putNumber("angle",999)
       table.putNumber("distance",999)


    #cv2.imshow('mask', input)
      
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()




