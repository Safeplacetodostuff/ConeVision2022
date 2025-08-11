
# -*- coding: utf-8 -*-
"""
Created on Sat Nov 19 09:05:48 2022

@author: AndyTien
"""
import cv2
import numpy as np
import time
def findIntersection(p1,p2,p3,p4):
    px= ( (p1[0]*p2[1]-p1[1]*p2[0])*(p3[0]-p4[0])-(p1[0]-p2[0])*(p3[0]*p4[1]-p3[1]*p4[0]) ) / ( (p1[0]-p2[0])*(p3[1]-p4[1])-(p1[1]-p2[1])*(p3[0]-p4[0]) ) 
    py= ( (p1[0]*p2[1]-p1[1]*p2[0])*(p3[1]-p4[1])-(p1[1]-p2[1])*(p3[0]*p4[1]-p3[1]*p4[0]) ) / ( (p1[0]-p2[0])*(p3[1]-p4[1])-(p1[1]-p2[1])*(p3[0]-p4[0]) )
    return [int(px), int(py)]
s=time.perf_counter()
src = cv2.imread("C:/Users/penho/Desktop/Test for weejion/12.jpg", cv2.IMREAD_COLOR)
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

corners = cv2.goodFeaturesToTrack(grad, 27, 0.01, 10)
corners = np.int0(corners)
src=cv2.cvtColor(src,cv2.COLOR_HSV2RGB)
cx=0
cy=0

size=src.shape
maxx=[0,0]
minx=[999,999]
maxy=[0,0]
miny=[999,999]
counter=0
for i in corners:
    counter+=1
    x, y = i.ravel()
    #draw point
    #cv2.circle(grad, (x, y), 1, 255, 2)
    #check if critical point
    if x>maxx[0]:
        maxx=[x,y]
    if y>maxy[1]:
        maxy=[x,y]
    if x<minx[0]:
        minx=[x,y]
    if y<miny[1]:
        miny=[x,y]
     #cauculate center       
    cx+=x
    cy+=y
center=[cx//counter, cy//counter]
# cv2.circle(src, miny, 1, (100,200,300), 1)
# cv2.circle(src, maxy, 1, (100,200,300), 1)
# cv2.circle(src, minx, 1, (100,200,300), 1)
# cv2.circle(src, maxx, 1, (100,200,300), 1)
# cv2.circle(src, center, 1, (100,200,300), 1)
# cv2.line(src,minx,miny,(100,200,300),1)
# cv2.line(src,maxx,maxy,(100,200,300),1)
# cv2.line(src,minx,maxy,(100,200,300),1)
# cv2.line(src,maxx,miny,(100,200,300),1)
# cv2.line(src,miny,maxy,(100,200,300),1)
# cv2.line(src,minx,maxx,(100,200,300),1)
# grad=cv2.bitwise_not(grad)

#find and draw crititcal points
# if abs(minx[1]-maxy[1])<10 or abs(maxx[1]-miny[1])<30 or abs(minx[1]-miny[1])<30 or abs(minx[1]-miny[1])<30:
#     inter=[(maxx[0]+minx[0])//2,(minx[1]+minx[1])//2]
# elif abs(minx[0]-maxy[0])<30 or abs(maxx[0]-miny[0])<30 or abs(minx[0]-miny[0])<30 or abs(maxx[0]-maxy[0])<30:
#     inter=[(maxy[0]+miny[0])//2,(miny[1]+miny[1])//2]
# else:
inter=findIntersection(minx,maxx,miny,maxy)
e=time.perf_counter()
print("runtime",e-s)
#cv2.line(src,inter,center,(300,200,100),1)
#show all images
cv2.imshow("lines/critpoints",grad)
cv2.imshow("result",gray)
#return runtime and end program on keypress

cv2.waitKey(0)
cv2.destroyAllWindows()