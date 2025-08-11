# -*- coding: utf-8 -*-
"""
Created on Sat Nov 19 13:26:15 2022

@author: penho
"""

# -*- coding: utf-8 -*-
"""
Created on Sat Nov 19 09:05:48 2022

@author: penho
"""

# Python code to read image
import cv2
import numpy as np
import time
s=time.perf_counter()
# To read image from disk, we use
# cv2.imread function, in below method,
src = cv2.imread("C:/Users/penho/Downloads/ConeHori.png", cv2.IMREAD_COLOR)
src=cv2.cvtColor(src,cv2.COLOR_RGB2HSV)

scale = 0.4
delta = 9
ddepth = cv2.CV_16S
# Creating GUI window to display an image on screen
# first Parameter is windows title (should be in string format)
# Second Parameter is image array
src=cv2.bilateralFilter(src, 15, 150, 150)
    
channel=1
gray = src[:,:,channel]
kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
gray = cv2.filter2D(gray, -1, kernel)
    
grad_x = cv2.Sobel(gray, ddepth, 1, 0, ksize=3, scale=scale, delta=delta, borderType=cv2.BORDER_DEFAULT)
   # Gradient-Y
    # grad_y = cv.Scharr(gray,ddepth,0,1)
grad_y = cv2.Sobel(gray, ddepth, 0, 1, ksize=3, scale=scale, delta=delta, borderType=cv2.BORDER_DEFAULT)
    
    
abs_grad_x = cv2.convertScaleAbs(grad_x)
abs_grad_y = cv2.convertScaleAbs(grad_y)
    
    
grad = cv2.addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0)
    
    
cv2.imshow("bob",grad)

    

# detect corners with the goodFeaturesToTrack function.
corners = cv2.goodFeaturesToTrack(grad, 27, 0.01, 10)
corners = np.int0(corners)
  
# we iterate through each corner, 
# making a circle at each point that we think is a corner.
for i in corners:
    x, y = i.ravel()
    cv2.circle(src, (x, y), 3, 255, -1)
  
cv2.imshow("joe",src[:,:,channel])
cv2.imshow("jim",cv2.cvtColor(src,cv2.COLOR_HSV2RGB))
e=time.perf_counter()
print(e-s)
cv2.waitKey(0)
cv2.destroyAllWindows()