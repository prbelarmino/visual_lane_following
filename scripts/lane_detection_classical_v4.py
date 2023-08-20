#!/usr/bin/env python
import numpy as np
import cv2
from random import randrange
import math

class ClassicalLaneDetector():

    def __init__(self):

        ### Set thresholds and other variables
        self.plot_flag = False
        self.image = np.array([])
        self.hsv_image = np.array([])
        self.filtered_hsv = np.array([])
        self.edges = np.array([])
        self.masked_edges = np.array([])
        self.output = np.array([])
        
        #### For HSV thresholding --> get_thresholded_hsv
        self.lower_threshold =  np.array([0,75,70],dtype = "uint8")
        self.upper_threshold = np.array([200,175,200],dtype = "uint8")

        #### For canny edge detection --> get_canny_edges
        self.kernel_size = 15 ## Filter size for gaussian blur
        self.threshlow = 180
        self.threshhigh = 250

        #### For region masking --> mask_region
        self.mask_points = [(0, 240), (0, 130), (424, 130), (424, 240)]
        self.rs_mask_vertices = np.array([self.mask_points ], dtype=np.int32)

        #### For HoughLines --> get_HoughP
        self.vote_threshold = 20 # minimum number of votes (intersections in Hough grid cell)
        self.min_linelength = 10 #minimum number of pixels making up a line
        self.max_linegap = 15 # maximum gap in pixels between connectable line segments
        self.rho = 0.75
        self.theta = np.pi/180

        #### For get_lane_slopes

        self.left_lines_list = []
        self.right_lines_list = []
        self.left_line_avg = []
        self.right_line_avg = []
        self.ignore_mask_color = 255
        self.rs_image_height = 240
        self.rs_image_width = 424
        self.rs_y_line_limit = 150
        self.window_size = 10
        self.color = (255, 255, 0)
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.fontScale = 1
        self.thickness = 2
        self.d = 200
        self.left_lines_tr = []
        self.right_lines_tr = []
        

    def lane_transformation(self,lane):

        if not lane is None and len(lane):
 
            xp = np.array([lane[0], lane[2]])
            yp = np.array([lane[1], lane[3]])
            xp = 0.5 * self.rs_image_width + 0.5 * self.rs_image_height * (xp - self.rs_image_width / 2) / (yp - self.rs_image_height / 2)
            yp = self.rs_image_height - self.d*(self.rs_image_height - yp)/(yp-self.rs_image_height/2)
            lane_tr = [xp[0],yp[0],xp[1],yp[1]]

        else: 

            lane_tr = []

        return lane_tr
    
    def get_mov_avg(self,line_list,parameters):

        para_len = len(parameters)
        line_len = len(line_list)

        if para_len:

            line_list.append(np.average(parameters, axis=0))

        if line_len > self.window_size or para_len == 0 and line_len > 0:

            line_list.pop(0)

        if len(line_list): 

            line = np.average(np.array(line_list), axis=0)
            a = (line[3] - line[1])/(line[2] - line[0])
            b = line[1] - a*line[0]
            y1 = self.rs_image_height
            x1 = (y1 - b)/a
            y2 = y1/1.5
            x2 = (y1 - b)/a
            angle = math.atan(a)
            
            if a < 0:

                angle += math.radians(180)
            line = np.array([x1,y1,x2,y2,angle])

        else:

            line = np.array([])

        return line
    
    def get_lanes(self,lines):

        left_lines_tr = []
        right_lines_tr = []
        if not lines is None:
            for line in lines:
                
                x1, y1, x2, y2 = line.reshape(4)
                if x1 != x2:
                    parameters = np.polyfit((x1, x2), (y1, y2), 1)
                    slope = parameters[0]
                    y_int = parameters[1]
                    yh = self.rs_image_height
                    xh = int((yh - y_int)/slope)
                    ym = int(self.rs_image_height/1.5)
                    xm = int((ym - y_int)/slope)

                    if  abs(slope) > 0.2:
                        
                        if slope < 0 and x1 < 212:

                            #left.append((xh,yh,xm,ym))
                            left_lines_tr.append(self.lane_transformation([x1,y1,x2,y2]))

                        elif slope > 0 and x2 >= 212:

                            right_lines_tr.append(self.lane_transformation([x1,y1,x2,y2]))

        self.left_lane = self.get_mov_avg(self.left_lines_list,left_lines_tr)
        self.right_lane = self.get_mov_avg(self.right_lines_list,right_lines_tr)

    def detection_pipeline(self,image):

        self.image = image
        self.hsv_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        self.filtered_hsv = cv2.inRange(self.hsv_image,self.lower_threshold ,self.upper_threshold)
        ### returns the edges dtected from canny method
        blur = cv2.GaussianBlur(self.filtered_hsv,(self.kernel_size, self.kernel_size),0)
        self.edges = cv2.Canny(blur, self.threshlow, self.threshhigh)
        ### takes canny output as input and returns edges isolated from the region of interest
        mask = np.zeros_like(self.edges) 
        cv2.fillPoly(mask, self.rs_mask_vertices, self.ignore_mask_color)
        self.masked_edges = cv2.bitwise_and(self.edges, mask)
        lines = cv2.HoughLinesP(self.masked_edges, self.rho, self.theta, self.vote_threshold , np.array([]),self.min_linelength , self.max_linegap)
        self.get_lanes(lines)

        if self.plot_flag:

            self.output = np.stack((self.edges,) * 3, axis=-1)
            cv2.line(self.output, self.mask_points[1], self.mask_points[2], (255, 255, 255), 2)

            if not lines is None:

                for line in lines:

                    line = line[0]
                    cv2.line(self.output, (int(line[0]), int(line[1])), (int(line[2]), int(line[3])), (randrange(255), randrange(255), randrange(255)), 2)
                    
                    
            if self.left_lane.shape[0]:

                self.left_lines_list
                left_slope = math.degrees(self.left_lane[4])
                left_line_avg = self.left_lane.astype(int) 
                #lines = np.array(self.left_lines_list)
                #left_line_avg = np.average(lines, axis=0)
                #left_line_avg = left_line_avg.astype(int) 
                org = (50, 50)
                org2 = (30,85)
                self.output = cv2.putText(self.output, "{:2.2f}".format(left_slope), org, self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA)
                self.output = cv2.putText(self.output, "{:3.2f}".format(self.left_lane[0]), org2, self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA)               
                cv2.line(self.output, (left_line_avg[0], left_line_avg[1]), (left_line_avg[2], left_line_avg[3]), (255,255,0), 8)

            if self.right_lane.shape[0]:
                #right_slope = (self.right_line_avg[3] - self.right_line_avg[1]) / (self.right_line_avg[2] - self.right_line_avg[0])
                
                right_slope = math.degrees(self.right_lane[4])
                right_line_avg = self.right_lane.astype(int)

                org = (300, 50)
                org2 = (300, 85)
                self.output = cv2.putText(self.output, "{:1.2f}".format(right_slope), org, self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA)
                self.output = cv2.putText(self.output, "{:3.2f}".format(self.right_lane[0]), org2, self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA)
                cv2.line(self.output, (right_line_avg[0], right_line_avg[1]), (right_line_avg[2], right_line_avg[3]), (255,255,0), 8)




