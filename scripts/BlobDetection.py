import cv2
import numpy as np
import math

class BlobDetection:

    def detectRed (self, frame):
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
    
    def detectWhite(self, frame, maximumArea):

        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        cv2.threshold(gray, 240, 255, cv2.THRESH_BINARY, gray)
        #cv2.imshow("thresh", gray)

        contours, hierarchy = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        largest_blob = None
        largest_blob_size = 0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > largest_blob_size and area < maximumArea:
                largest_blob_size = area
                largest_blob = contour
            elif area > largest_blob_size:
                print (maximumArea)

        #print(largest_blob_size)

        if largest_blob_size > 200:
            x,y,w,h = cv2.boundingRect(largest_blob)
            frame = cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
            #cv2.imshow("frame", frame)
            cv2.waitKey(1)
            return [0, 1, (x, y, w, h)]

        #cv2.imshow("frame", frame)
        #cv2.waitKey(1)
        return None
                
