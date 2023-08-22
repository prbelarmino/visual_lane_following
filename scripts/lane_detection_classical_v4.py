#!/usr/bin/env python
import numpy as np
import cv2
import math

class ClassicalLaneDetector():

    def __init__(self):

        self.plot_flag = False
        self.image = np.array([])
        self.hsv_image = np.array([])
        self.filtered_hsv = np.array([])
        self.edges = np.array([])
        self.masked_edges = np.array([])
        self.output = np.array([])
        self.left_lines_list = []
        self.right_lines_list = []
        self.right_lines_arr = []
        self.left_lines_arr = []
        self.ignore_mask_color = 255
        self.rs_image_height = 240
        self.rs_image_width = 424
        self.rs_y_line_limit = 130
        self.window_size = 1
        self.color = (255, 255, 0)
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.fontScale = 1
        self.thickness = 2
        self.d = 200 #Why ??????????????????

        #### For HSV thresholding --> get_thresholded_hsv
        self.lower_threshold = np.array([0, 75, 70], dtype="uint8")
        self.upper_threshold = np.array([200, 175, 200], dtype="uint8")

        #### For canny edge detection --> get_canny_edges
        self.kernel_size = 15  ## Filter size for gaussian blur
        self.threshlow = 180
        self.threshhigh = 250

        #### For region masking --> mask_region
        self.mask_points = [(0, self.rs_image_height), (0, self.rs_y_line_limit), (self.rs_image_width, self.rs_y_line_limit), (self.rs_image_width, self.rs_image_height)]
        self.rs_mask_vertices = np.array([self.mask_points], dtype=np.int32)

        #### For HoughLines --> get_HoughP
        self.vote_threshold = 20  # minimum number of votes (intersections in Hough grid cell)
        self.min_linelength = 10  # minimum number of pixels making up a line
        self.max_linegap = 15  # maximum gap in pixels between connectable line segments
        self.rho = 0.75
        self.theta = np.pi / 180

    def lane_transformation(self, lane):

        if not lane is None and len(lane):

            xp = np.array([lane[0], lane[2]])
            yp = np.array([lane[1], lane[3]])
            xp = 0.5 * self.rs_image_width + 0.5 * self.rs_image_height * (xp - self.rs_image_width / 2) / (
                        yp - self.rs_image_height / 2)
            yp = self.rs_image_height - self.d * (self.rs_image_height - yp) / (yp - self.rs_image_height / 2)
            lane_tr = [xp[0], yp[0], xp[1], yp[1]]

        else:

            lane_tr = []

        return lane_tr

    def extrap_line(self,x_points,y_points):

        parameters = np.polyfit(x_points, y_points, 1)
        slope = parameters[0]
        y_int = parameters[1]
        ym = self.rs_y_line_limit
        xm = int((ym - y_int) / slope)
        yh = self.rs_image_height
        xh = int((yh - y_int) / slope)
        angle = math.atan(slope)
        if slope < 0:

            angle += math.radians(180)
        return np.array([xh,yh,xm,ym,angle])
    
    def merge_lines(self,lines_array):

        merged_line = np.array([])
        points_len = lines_array.shape[0]
        if points_len:
            merged_line = np.average(lines_array, axis=0)
        return merged_line
    
    def get_mov_avg(self, line_list, lines_array):

        points_len = lines_array.shape[0]
        if points_len:
            merged_line = self.merge_lines(lines_array)
            line_list.append(merged_line)

        line_len = len(line_list)
        if line_len > self.window_size or points_len == 0 and line_len > 0:
            line_list.pop(0)
        if len(line_list):

            line = np.average(np.array(line_list), axis=0)
            extrap_line = np.array(self.extrap_line((line[0],line[2]),(line[1],line[3])))

        else:
            extrap_line = np.array([])

        return extrap_line

    def set_points_order(self,line):

        if line[4] < 0:

            return(line[0])
    def get_lanes(self, lines):

        left_lines_tr_arr = []
        right_lines_tr_arr = []

        if not lines is None:
            for line in lines:

                line = line[0].astype(float)
                slope = (line[2]-line[0])/ (line[3]-line[1])
                tr_line = self.lane_transformation([line[0], line[1], line[2], line[3]])
                extrap_line = self.extrap_line((tr_line[0],tr_line[2]),(tr_line[1],tr_line[3]))

                #if tr_line[0] != tr_line[2]
                if abs(extrap_line[4]) > 0.7854:
                 
                    if slope <= 0 and line[0] < 180 and extrap_line[0] < 212 and extrap_line[0] * math.sin(extrap_line[4]) > -300:

                        left_lines_tr_arr.append([tr_line[0], tr_line[1], tr_line[2], tr_line[3], extrap_line[4]])
                        self.left_lines_arr.append([line[0], line[1], line[2], line[3], slope])

                    elif  slope >0 and line[2] > 240 and extrap_line[0] >= 212 and  extrap_line[0]* math.sin(extrap_line[4]) < 550:

                        self.right_lines_arr.append([line[0], line[1], line[2], line[3], slope])
                        right_lines_tr_arr.append([tr_line[0], tr_line[1], tr_line[2], tr_line[3], extrap_line[4]])

        self.left_lane = self.get_mov_avg(self.left_lines_list, np.array(left_lines_tr_arr))
        self.right_lane = self.get_mov_avg(self.right_lines_list, np.array(right_lines_tr_arr))

        if self.plot_flag:
            self.left_line = self.merge_lines(np.array(self.left_lines_arr))
            if self.left_line.shape[0]:
                self.left_line = self.extrap_line((self.left_line[0], self.left_line[2]),
                                                  (self.left_line[1], self.left_line[3]))
            self.right_line = self.merge_lines(np.array(self.right_lines_arr))
            if self.right_line.shape[0]:
                self.right_line = self.extrap_line((self.right_line[0], self.right_line[2]),
                                                  (self.right_line[1], self.right_line[3]))

    def detection_pipeline(self, image):

        self.image = image
        self.hsv_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        self.filtered_hsv = cv2.inRange(self.hsv_image, self.lower_threshold, self.upper_threshold)
        ### returns the edges dtected from canny method
        blur = cv2.GaussianBlur(self.filtered_hsv, (self.kernel_size, self.kernel_size), 0)
        self.edges = cv2.Canny(blur, self.threshlow, self.threshhigh)
        ### takes canny output as input and returns edges isolated from the region of interest
        mask = np.zeros_like(self.edges)
        cv2.fillPoly(mask, self.rs_mask_vertices, self.ignore_mask_color)
        self.masked_edges = cv2.bitwise_and(self.edges, mask)
        lines = cv2.HoughLinesP(self.masked_edges, self.rho, self.theta, self.vote_threshold, np.array([]),
                                self.min_linelength, self.max_linegap)
        self.get_lanes(lines)

        if self.plot_flag:

            self.output = np.stack((self.edges,) * 3, axis=-1)
            cv2.line(self.output, (212,0), (212,240), (255, 255, 255), 2)
            cv2.line(self.output, (180, 0), (180, 240), (255, 255, 255), 2)
            cv2.line(self.output, (240, 0), (240, 240), (255, 255, 255), 2)
            cv2.line(self.output, self.mask_points[1], self.mask_points[2], (255, 255, 255), 2)

            if not lines is None:

                for line in lines:
                    line = line[0]
                    cv2.line(self.output, (int(line[0]), int(line[1])), (int(line[2]), int(line[3])),
                             (0, 255, 255), 2)

                for line in self.right_lines_arr:
                    cv2.line(self.output, (int(line[0]), int(line[1])), (int(line[2]), int(line[3])),
                             (255, 0, 0), 2)
                for line in self.left_lines_arr:
                    cv2.line(self.output, (int(line[0]), int(line[1])), (int(line[2]), int(line[3])),
                             (0, 0, 255), 2)

            if self.left_lane.shape[0]:

                left_slope = math.degrees(self.left_lane[4])
                left_line_avg = self.left_line.astype(int)
                left_lane_avg = self.left_lane.astype(int)
                org = (50, 50)
                org2 = (30, 85)
                self.output = cv2.putText(self.output, "{:2.2f}".format(left_slope), org, self.font, self.fontScale,
                                          self.color, self.thickness, cv2.LINE_AA)
                self.output = cv2.putText(self.output, "{:3.2f}".format(self.left_lane[0]), org2, self.font,
                                          self.fontScale, self.color, self.thickness, cv2.LINE_AA)
                cv2.line(self.output, (left_lane_avg[0], left_lane_avg[1]), (left_lane_avg[2], left_lane_avg[3]),
                         (0, 255, 0), 8)
                cv2.line(self.output, (left_line_avg[0], left_line_avg[1]), (left_line_avg[2], left_line_avg[3]),
                         (0, 155, 0), 8)

            if self.right_lane.shape[0]:

                right_slope = math.degrees(self.right_lane[4])
                right_lane_avg = self.right_lane.astype(int)
                right_line_avg = self.right_line.astype(int)
                org = (300, 50)
                org2 = (300, 85)
                self.output = cv2.putText(self.output, "{:1.2f}".format(right_slope), org, self.font, self.fontScale,
                                          self.color, self.thickness, cv2.LINE_AA)
                self.output = cv2.putText(self.output, "{:3.2f}".format(self.right_lane[0]), org2, self.font,
                                          self.fontScale, self.color, self.thickness, cv2.LINE_AA)
                cv2.line(self.output, (right_lane_avg[0], right_lane_avg[1]), (right_lane_avg[2], right_lane_avg[3]),
                         (0, 255, 0), 8)
                cv2.line(self.output, (right_line_avg[0], right_line_avg[1]), (right_line_avg[2], right_line_avg[3]),
                         (0, 155, 0), 8)




