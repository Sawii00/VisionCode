import cv2
import numpy as np
from networktables import NetworkTables
#import argparse
########################################################

NetworkTables.setIPAddress("10.6.14.2")  # Change the address to your own
NetworkTables.setClientMode()
NetworkTables.initialize()
table = NetworkTables.getTable("gearCamera")
GLOBAL_TAPE_HEIGHT = 5.0  # inches

'''ap = argparse.ArgumentParser()
ap.add_argument("-huemin", "--hueminimum",type=int, required=True,
    help="HUE")
ap.add_argument("-huemax", "--huemaximum",type=int, required=True,
    help="HUE")
ap.add_argument("-satmin", "--saturationminimum", type=int, required=True,
    help="SATURATION")
ap.add_argument("-satmax", "--saturationmaximum", type=int, required=True,
    help="SATURATION")
ap.add_argument("-valmin", "--valueminimum", type=int, required=True,
    help="VALUE")
ap.add_argument("-valmax", "--valuemaximum", type=int, required=True,
    help="VALUE")
args = vars(ap.parse_args())

hmin = args["hueminimum"]
hmax = args["huemaximum"]
smin = args["saturationminimum"]
smax = args["saturationmaximum"]
vmin = args["valueminimum"]
vmax = args["valuemaximum"]'''


def midMidPointStick(contour1, contour2):
    M1 = cv2.moments(contour1)
    M2 = cv2.moments(contour2)
    if M1['m00'] != 0 and M2['m00'] != 0:
        cx1 = int(M1['m10'] / M1['m00'])
        cx2 = int(M2['m10'] / M2['m00'])
    else:
        cx1 = 0
        cx2 = 0
    mid = (cx1 + cx2) / 2
    return mid


def distanceFromCenter(midPoint):
    d = midPoint - 320
    return d


def distance_to_camera(knownWidth, focalLength, perWidth):
    # compute and return the distance from the maker to the camera
    return (knownWidth * focalLength) / perWidth


def pixelToDegree(dPix):
    return dPix / 10.67


def findRectHeight(contour1, contour2):
    x1, y1, w1, h1 = cv2.boundingRect(contour1)
    x2, y2, w2, h2 = cv2.boundingRect(contour2)
    glob_height = float((h1 + h2))/2 -3
    return glob_height if glob_height != 0 else 1

def writeText(text, orig, img, color = (255,255,255)):
    cv2.putText(img, text, orig, cv2.FONT_HERSHEY_SIMPLEX, 0.5,color)

##################################################################
lower_red = np.array([hmin, smin, vmin])
upper_red = np.array([hmax, smax, vmax])

cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

#######################################################################
while True:
    _, input = cap.read()
    # RGB TO HSV
    out = cv2.cvtColor(input, cv2.COLOR_BGR2HSV)
    # threshold HSV
    mask = cv2.inRange(out, lower_red, upper_red)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2. CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:2]
    for c in contours:
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
        area = cv2.contourArea(approx)
        if area > 0:
            cv2.drawContours(input, [c], -1, (0, 255, 0), 2)
            # focal length = 1118
    if len(contours) == 2:
        mid = midMidPointStick(contours[0], contours[1])
        #print "Angle: %f" % pixelToDegree(distanceFromCenter(mid))
       # print "Distance: %f" % distance_to_camera(GLOBAL_TAPE_HEIGHT, 1118.0, findRectHeight(contours[0], contours[1]))

       # print findRectHeight(contours[0], contours[1])
        angle = pixelToDegree(distanceFromCenter(mid))
        distance = distance_to_camera(GLOBAL_TAPE_HEIGHT, 559.0, findRectHeight(contours[0], contours[1]))
        writeText("Angle: %f" % angle,(50,200), input)
        writeText("Distance: %f" % distance,(50,300), input)

        table.putBoolean("targetFound", True)
        table.putNumber("angle", angle)
        table.putNumber("distance", distance)
    else:
        writeText("Nothing found",(50,300), input, (0,0,255))
        table.putBoolean("targetFound", False)
        table.putNumber("angle", 999)
        table.putNumber("distance", 999)
        
    #cv2.imshow('mask', input)
   
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()


