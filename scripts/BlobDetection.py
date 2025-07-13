import cv2
import numpy as np
import math

def detectRed (frame):
    cv2.blur(frame, (5, 5), frame)
    channels = cv2.split(frame)
    red = cv2.subtract(channels[2], channels[1])
    cv2.threshold(red, 80, 255, cv2.THRESH_BINARY, red)
    contours, hierarchy = cv2.findContours(red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    largest_blob = None
    largest_blob_size = 0
    for contour in contours:
        area = cv2.contourArea(contour) 
        if area > largest_blob_size:
            largest_blob_size = area
            largest_blob = contour

    #print (largest_blob_size)

    if largest_blob_size > 200:
        x,y,w,h = cv2.boundingRect(largest_blob)
        frame = cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
        #cv2.imshow("frame", frame)
        ##cv2.waitKey(1)
        return [0, 1, (x, y, w, h)]

    #cv2.imshow("frame", frame)
    #cv2.waitKey(1)

    return None

def detectWhite(frame, maximumArea):
    #hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #minc = (38, 0, 180)
    #maxc = (180, 120, 255)
    #mask_white = cv2.inRange(hsv, minc, maxc)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray,(5,5), 0)
    #result_white = cv2.bitwise_and(frame, frame, mask=mask_white)
    ret, thresh = cv2.threshold(blur, 160, 255, cv2.THRESH_BINARY, blur)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    largest_blob = None
    largest_blob_size = 0
    for contour in contours:
        area = cv2.contourArea(contour)
        x,y,w,h = cv2.boundingRect(contour)
        if area > largest_blob_size and area < maximumArea and min(h, w) / max(h, w) > 0.4:
            largest_blob_size = area
            largest_blob = contour

    if largest_blob_size > 220:
        x,y,w,h = cv2.boundingRect(largest_blob)
        frame_cpy = np.copy(frame)
        frame_cpy = cv2.rectangle(frame_cpy,(x,y),(x+w,y+h),(0,255,0),2)
        #cv2.imshow("frame", frame_cpy)
        #cv2.waitKey(1)
        return (x, x + w, y, y + h)

    #cv2.imshow("frame", frame)
    #cv2.waitKey(1)
    return None
                
