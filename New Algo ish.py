
# -*- coding: utf-8 -*-
"""
Created on Sat Nov 19 09:05:48 2022

@author: penho
"""
import cv2
import numpy as np
import time
import math
def findIntersection(p1,p2,p3,p4):
    px= ( (p1[0]*p2[1]-p1[1]*p2[0])*(p3[0]-p4[0])-(p1[0]-p2[0])*(p3[0]*p4[1]-p3[1]*p4[0]) ) / ( (p1[0]-p2[0])*(p3[1]-p4[1])-(p1[1]-p2[1])*(p3[0]-p4[0]) ) 
    py= ( (p1[0]*p2[1]-p1[1]*p2[0])*(p3[1]-p4[1])-(p1[1]-p2[1])*(p3[0]*p4[1]-p3[1]*p4[0]) ) / ( (p1[0]-p2[0])*(p3[1]-p4[1])-(p1[1]-p2[1])*(p3[0]-p4[0]) )
    return [int(px), int(py)]
def find_dist(points):
    a=(points[0][1]-points[1][1])*(points[0][1]-points[1][1])
    b=(points[0][0]-points[1][0])*(points[0][0]-points[1][0])
    print(points,math.sqrt(a+b))
    return math.sqrt(a+b)
def average_points(points):
    a=0
    b=0
    c=0
    for i in points:
        a+=i[0]
        b+=i[1]
        c+=1
    return [a//c,b//c]
s=time.perf_counter()
src = cv2.imread("C:/Users/penho/Downloads/ConeVerti.png", cv2.IMREAD_COLOR)
cv2.imshow("dodo",src)
#src=cv2.cvtColor(src,cv2.COLOR_RGB2HSV)
#convert to hsv preproccess
src=cv2.bilateralFilter(src, 15, 175, 175)
channel=2
gray = src[:,:,channel]
#cones show up best on saturation level
kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
gray = cv2.filter2D(gray, -1, kernel)   
grad = cv2.Canny(gray,100,255)    

corners = cv2.goodFeaturesToTrack(grad, 100, 0.01, 30)
corners = np.int0(corners)
    
maxx=[0,0]
minx=[999,999]
maxy=[0,0]
miny=[999,999]

for i in corners:
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
#find min center
group1=[]
group2=[]
dists=[[minx,miny],[maxx,maxy],[maxx,miny],[minx,maxy]]
special=0
test=sorted(dists,key=lambda a: find_dist(a),reverse=True)
for i in test[0]:
    if i in test[1]:
        group2.append(i)
        special=1
if special==0:
    group1=test.pop()
    group2=test.pop()
    if find_dist(group1)>find_dist(group2):
        cv2.circle(grad,average_points(group1),5,255,3)
        cv2.circle(grad,average_points([average_points(group1),average_points(group2)]),5,255,3)
    else:
        cv2.circle(grad,average_points(group2),5,255,3)
        cv2.circle(grad,average_points([average_points(group1),average_points(group2)]),5,255,3)
else:
    group1=[minx,maxx,miny,maxy]
    for i in group2:
        group1.remove(i)
    cv2.circle(grad,average_points(group1),5,255,3)
    cv2.circle(grad,average_points([average_points(group2),average_points(group1)]),1,255,3)
cv2.imshow("bobbers",grad)
cv2.waitKey(0)
cv2.destroyAllWindows()
