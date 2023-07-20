#!/usr/bin/env python
import numpy as np
import cv2
import math
class ClassicalLaneDetector():

    def __init__(self):

        ### Set thresholds and other variables

        #### For HSV thresholding --> get_thresholded_hsv
        
        self.h_low = 130
        self.h_high = 175
        self.s_low = 55
        self.s_high = 155
        self.v_low = 0
        self.v_high = 15

        #### For canny edge detection --> get_canny_edges
        self.kernel_size = 15 ## Filter size for gaussian blur
        self.threshlow = 180
        self.threshhigh = 250

        #### For region masking --> mask_region
        self.lvertices = np.array([[(0,320),(0, 150), (239, 150), (239,320)]], dtype=np.int32) ### Vertices to mask the right side of the image
        self.rvertices = np.array([[(240,320),(240, 150), (480, 150), (480,320)]], dtype=np.int32) ### Vertices to mask the left side of the image
        self.rs_mask_vertices = np.array([[(0, 180),(0, 140),(424, 140),(424, 180)]], dtype=np.int32)

        #### For HoughLines --> get_HoughP
        self.vote_threshold = 15 # minimum number of votes (intersections in Hough grid cell)
        self.min_linelength = 5 #minimum number of pixels making up a line
        self.max_linegap = 15 # maximum gap in pixels between connectable line segments
        self.rho = 1
        self.theta = np.pi/180

        #### For get_lane_slopes
        #right
        self.rminx2 = 1000
        self.rmaxy2 = 0
        self.rlength = 0
        #left
        self.lminx1 = 1000
        self.lmaxy1 = 0
        self.llength = 0
        self.left_lines_list = []
        self.right_lines_list = []
        self.rs_image_height = 240
        self.rs_y_line_limit = 150
        self.fe_image_height = 360
        self.fe_y_line_limit = 140
        self.window_size = 10

    def get_thresholded_hsv(self,image,flag):
        ### returns thresholded image where the lanes are isolated
        
        if flag == "FISHEYE":

            lower_threshold = np.array([0,75,100],dtype = "uint8")
            upper_theshold = np.array([200,150,200],dtype = "uint8")
            lower_threshold = np.array([0,55,130],dtype = "uint8")
            upper_theshold = np.array([15,155,175],dtype = "uint8")
        
        else:

            lower_threshold = np.array([0,75,100],dtype = "uint8")
            upper_theshold = np.array([200,175,250],dtype = "uint8")
        # image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # clahe = cv2.createCLAHE(clipLimit=10, tileGridSize=(32,32))
        # equ = cv2.equalizeHist(image_gray)
        # res = np.hstack((image_gray,equ)) #stacking images side-by-side

        # cl1 = clahe.apply(image_gray)
        # alpha = 0.5 # Contrast control
        # beta = 0.01 # Brightness control

        # call convertScaleAbs function
        #adjusted = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
        
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        filtered_image = cv2.inRange(image_hsv,lower_threshold,upper_theshold)
        #for i in image_hsv[:,1,1]
        #print(image_hsv.shape)
        if False:
            cv2.imshow("Original", image_hsv)
            cv2.waitKey(50)
            cv2.imshow("Filtered", filtered_image)
            cv2.waitKey(50)
   

        return filtered_image

    def get_canny_edges(self,image,kernel_size=15,thresh_low=None,thresh_high=None):

        if thresh_low is None:
            thresh_low = self.threshlow

        if thresh_high is None:
            thresh_high = self.threshhigh
 
        ### returns the edges dtected from canny method
        blur = cv2.GaussianBlur(image,(kernel_size, kernel_size),0)
        edges = cv2.Canny(blur, thresh_low, thresh_high)
        if False:
            cv2.imshow("Detected Lane", edges)
            cv2.waitKey(30)
        return edges

    def mask_region(self,image,vertices,ignore_mask_color = 255):
        ### takes canny output as input and returns edges isolated from the region of interest
        mask = np.zeros_like(image) 
        cv2.fillPoly(mask, vertices, ignore_mask_color)
        masked_edges = cv2.bitwise_and(image, mask)
        if True:
            cv2.imshow("canny", image)
            cv2.waitKey(30)
            cv2.imshow("masked", masked_edges)
            cv2.waitKey(30)
        
        return masked_edges

    def get_mov_avg(self,line_list,parameters,flag):

        line = self.make_points(parameters,flag)
        line_list.append(line)
        #print(len(line_list))
        if len(line_list) > self.window_size:
        #if flag == "FISHEYE":

            line_list.pop(0) 

        return np.average(np.array(line_list), axis=0)
    
    def get_avg_lines(self,lines,image):

        left = []
        right = []
        x = 0

        # s_lines = lines[:,0,:]
        # s_lines = s_lines[s_lines[:, 0].argsort()]
        # group = np.split(s_lines, np.where(np.diff(s_lines[:,0]) > 10)[0]+1)
        # #print(s_lines)
        # #print(np.where(np.diff(s_lines[:,0]) > 10))
        # len_g = [len(i) for i in group]
        #print(len_g)
        #group_avg = [np.average(i, axis=0) for i in group]
        #print(group_avg)
        if not lines is None:
            for line in lines:
                
                x1, y1, x2, y2 = line.reshape(4)

                cv2.line(image,(x1,y1),(x2,y2),(x,100,2*x),10)
                x += 10
                line_len = math.sqrt((x2-x1)**2 + (y2-y1)**2)
                
                if x1 != x2:

                    parameters = np.polyfit((x1, x2), (y1, y2), 1)
                    slope = parameters[0]
                    y_int = parameters[1]
                    x0 = int((self.rs_image_height - y_int)/slope)

                    #not (abs(slope) > 100) or
                    if  abs(slope) > 0.2:
                        
                        if slope < 0 and x0 < 212:

                            left.append((slope, y_int))
                        elif slope > 0 and x0 >= 212:
                            right.append((slope, y_int))


        left_line_avg = []
        right_line_avg = []
        flag = "REALSENSE"
        
        #print(len(self.left_lines_list))  
        if len(left) > 0:
        
            left_line_avg = self.get_mov_avg(self.left_lines_list,np.average(left, axis=0),flag)

        else:

            if len(self.left_lines_list) >0:

                self.left_lines_list.pop(0) 
        #print(len(self.right_lines_list))
        #print(len(right))
        if len(right) > 0:

            right_line_avg = self.get_mov_avg(self.right_lines_list,np.average(right, axis=0),flag)
        
        else:

            if len(self.right_lines_list) > 0:

                self.right_lines_list.pop(0)

        return left_line_avg,right_line_avg
    
    def make_points(self,parameters,flag): 

        #print(parameters)
        slope, y_int = parameters 

        if flag == "FISHEYE":

            y1 = self.fe_image_height
            y2 = self.fe_y_line_limit

        else:

            y1 = self.rs_image_height
            y2 = self.rs_y_line_limit

        x1 = int((y1 - y_int)//slope)
        x2 = int((y2 - y_int)//slope)
        return np.array([x1, y1, x2, y2, slope])
    
    def get_lanes(self,image,edges):
 
        masked_edges = self.mask_region(edges,self.rs_mask_vertices)
        lines = cv2.HoughLinesP(masked_edges, self.rho, self.theta, self.vote_threshold , np.array([]),self.min_linelength , self.max_linegap)
        left_line_avg, right_line_avg = self.get_avg_lines(lines,image)
        if True:
            cv2.imshow("lines", image)
            cv2.waitKey(50)
        return image, left_line_avg, right_line_avg

    def get_lane_slopes2(self,image,edges, lvertices, rvertices):
 
        draw = False
        lmasked_edges = self.mask_region(edges,lvertices)
        rmasked_edges = self.mask_region(edges,rvertices)
        threshold  = self.vote_threshold 
        min_line_length = self.min_linelength 
        max_line_gap = self.max_linegap
        flag = "FISHEYE"
        l_lines = cv2.HoughLinesP(lmasked_edges, self.rho, self.theta, threshold, np.array([]),min_line_length, max_line_gap)
        r_lines = cv2.HoughLinesP(rmasked_edges, self.rho, self.theta, threshold, np.array([]),min_line_length, max_line_gap)
        
        if r_lines is None:
            r_lane = []
        else:
            rmin_x2 = self.rminx2
            rmax_y2 = self.rmaxy2 
            r_length = self.rlength

            for line in r_lines:
                x1,y1,x2,y2 = line.squeeze()                   
                length = ((x2-x1)**2 + (y2-y1)**2)**0.5
                slope = (y2-y1)/(x2-x1)
                if (abs(slope) == 0) or (abs(slope) > 100) :
                    r_lane = []
                else:
                    if (length>self.rlength) & (rmax_y2<y2) & (rmin_x2>x2):
                        rmax_y2 = y2
                        rmin_x2 = x2
                        r_length = length
                        r_slope = slope
                        right_line = np.array((x2,y2))
                        r_lane = self.get_mov_avg(self.right_lines_list,(r_slope,rmax_y2),flag)
            if (draw): ### Only for visualisation of detected lanes
                x2,y2 = right_line
                x1 = int((-y2+r_slope*x2)/r_slope)
                y1 = 0
                cv2.line(image,(x1,y1),(x2,y2),(0,255,0),10)
            
                
        if l_lines is None:
            l_lane = []
        else:
            lmin_x1 = self.lminx1
            lmax_y1 = self.lmaxy1
            l_length = self.llength
            for line in l_lines:
                x1,y1,x2,y2 = line.squeeze()  
                length = ((x2-x1)**2 + (y2-y1)**2)**0.5
                slope = (y2-y1)/(x2-x1)
                if (abs(slope) == 0) or (abs(slope) > 100) :
                    l_lane = []
                else:
                    if (length>self.llength) & (lmax_y1<y1) & (lmin_x1>x1):
                        lmax_y1 = y1
                        lmin_x1 = x1
                        l_length = length
                        left_line = np.array((x1,y1))
                        l_slope = slope
                        l_lane = self.get_mov_avg(self.left_lines_list,(l_slope,lmax_y1),flag)
            if draw:

                x1,y1 = left_line
                x2 = int((l_slope*x1-y1)/l_slope)
                y2 = 0
                cv2.line(image,(x1,y1),(x2,y2),(0,255,0),10)  

        return image, l_lane, r_lane
    
    def detect_pipeline(self,image,flag) :

        filtered_image = self.get_thresholded_hsv(image,flag)
        edges = self.get_canny_edges(filtered_image)
        if flag == "FISHEYE":      
            
            image,l_lane,r_lane = self.get_lane_slopes2(image,edges,self.lvertices,self.rvertices)

        else:

            image,l_lane,r_lane = self.get_lanes(image,edges)

        return image,l_lane,r_lane