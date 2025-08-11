# -*- coding: utf-8 -*-
"""
Created on Sat Nov 19 09:05:48 2022

@author: penho
"""
import cv2
import numpy as np
import time
def findIntersection(p1,p2,p3,p4):
    px= ( (p1[0]*p2[1]-p1[1]*p2[0])*(p3[0]-p4[0])-(p1[0]-p2[0])*(p3[0]*p4[1]-p3[1]*p4[0]) ) / ( (p1[0]-p2[0])*(p3[1]-p4[1])-(p1[1]-p2[1])*(p3[0]-p4[0]) ) 
    py= ( (p1[0]*p2[1]-p1[1]*p2[0])*(p3[1]-p4[1])-(p1[1]-p2[1])*(p3[0]*p4[1]-p3[1]*p4[0]) ) / ( (p1[0]-p2[0])*(p3[1]-p4[1])-(p1[1]-p2[1])*(p3[0]-p4[0]) )
    return [int(px), int(py)]

s=time.perf_counter()
src = cv2.imread("C:/Users/penho/Downloads/pp4.png", cv2.IMREAD_COLOR)
cv2.imshow("dodo",src)
src=cv2.cvtColor(src,cv2.COLOR_RGB2HSV)
#convert to hsv preproccess
src=cv2.bilateralFilter(src, 15, 175, 175)
channel=1
gray = src[:,:,channel]
#cones show up best on saturation level
kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
gray = cv2.filter2D(gray, -1, kernel)   
grad = cv2.Canny(gray,100,255)    

linesP = cv2.HoughLinesP(grad, 1, np.pi / 180, 30, None, 10, 10)
sorted(linesP,key=lambda a: a[0]/a[1])
if linesP is not None:
    for i in range(0, len(linesP)):
        l = linesP[i][0]
        cv2.line(src, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)
        
cv2.imshow('a',src)
cv2.imshow("sad", grad)
cv2.waitKey(0)
cv2.destroyAllWindows()