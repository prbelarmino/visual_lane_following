#!/usr/bin/env python
import numpy as np
import cv2
import math

class ClassicalLaneDetector:

    def __init__(self):

        pass

    def threshold_filter(self,image , thresh_low = 75, thresh_high = 200):
    
        hsvimage = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        copy = np.copy(hsvimage[:,:,1])
        filtered_image = np.zeros_like(copy)
        filtered_image[(thresh_low<copy)&(copy<thresh_high)]=255
        cv2.imshow("Detected Lane", filtered_image)
        cv2.waitKey(30)
        return filtered_image

    def canny_edges(self,image,kernel_size=9,thresh_low=180,thresh_high=250):
        
        blur = cv2.GaussianBlur(image,(kernel_size, kernel_size),0)
        edges = cv2.Canny(blur, thresh_low, thresh_high)
        
        return edges

    def mask_region(self,image,vertices = np.array([[(0,360),(0, 80), (480, 80), (480,360)]], dtype=np.int32)):#100

        mask = np.zeros_like(image)   
        ignore_mask_color = 255   
        cv2.fillPoly(mask, vertices, ignore_mask_color)
        masked_edges = cv2.bitwise_and(image, mask)
        return masked_edges

    def get_hough(self,image): 
        lines = cv2.HoughLines(image, 1, np.pi/180, 70)#70
        line_image = np.zeros_like(image) 
        line_image = np.dstack((line_image, line_image, line_image))
        
        right_line = []
        left_line = []
        right_x2 = 10000000
        left_y2 = 10000000
        
        tempr = 0
        templ = 0
        #print("lines")
        for r_theta in lines:
            arr = np.array(r_theta[0], dtype=np.float64)
            r, theta = arr
            
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = int(a*r)
            y0 = int(b*r)
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
            #print(r,theta,a,b,x0,y0)
            if r<=0 & x2<right_x2:
                right_line.append((r,theta))
                right_pt = np.array(((x1,y1),(x2,y2)))
                right_x2=x2
                tempr=1
                #k = math.atan((y2-y1)/(x2-x1))
                #print(r,theta,a,b,x0,y0,k)
                
            elif r>0 & y2<left_y2:
                left_line.append((r,theta))
                left_pt = np.array(((x1,y1),(x2,y2)))
                left_y2=y2
                templ=1
        
        #print(right_pt)
        #print(type(right_pt))
        if tempr == 1:
            cv2.line(line_image, tuple(right_pt[0,:]),tuple(right_pt[1,:]), (255, 0, 0), 10)
        else:
            pass           

        #print(len(line_image))
        if templ == 1:
            cv2.line(line_image, tuple(left_pt[0,:]),tuple(left_pt[1,:]), (255, 0, 0), 10)
        else:
            pass
        
        
        if (templ == 1) & (tempr == 1):
            temprpara = np.polyfit((right_pt[0,0],right_pt[1,0]),(right_pt[0,1],right_pt[1,1]),deg=1)
            templpara = np.polyfit((left_pt[0,0],left_pt[1,0]),(left_pt[0,1],left_pt[1,1]),deg=1)
            
                      
            if (np.abs(templpara[0])<=0.01) or (np.abs(temprpara[0])<=0.01) :
                pass
            else:

                lx_point = 0
                ly_point = int(templpara[0]*lx_point + templpara[1])

                fixed_y = 200
                lx_point = (fixed_y-templpara[1])/templpara[0]
                rx_point = (fixed_y-temprpara[1])/temprpara[0] 

                for delta in range(0,100,5):
                    point = ly_point-delta

                    lx_point = (point-templpara[1])/templpara[0]
                    rx_point = (point-temprpara[1])/temprpara[0]
                    center_point = int(np.mean((lx_point,rx_point)))
                    #print(lx_point,rx_point,center_point,point)
                    cv2.circle(line_image,(int(center_point),int(point)),radius=2,color=(0,255,0),thickness=2)
        else:
            pass

        return line_image

    def detect_pipeline(self,image) :

        filtered = self.threshold_filter(image)
        edges = self.canny_edges(filtered)
        masked_edges = self.mask_region(edges)
        lines = self.get_hough(masked_edges)
        output = cv2.addWeighted(image, 0.8, lines, 1, 0)
        return output