import numpy as np
from matplotlib import pyplot as plt
import cv2
from random import randrange
import rosbag
from pynput import keyboard

class Functions():

    def __init__(self,path):

        self.vote_threshold = 15 # minimum number of votes (intersections in Hough grid cell)
        self.min_linelength = 5 #minimum number of pixels making up a line
        self.max_linegap = 15 # maximum gap in pixels between connectable line segments
        self.rho = 1
        self.theta = np.pi/180
        self.hsv_thresholds =np.array([0, 200, 75, 175, 100,200])
        self.hough_thresholds = np.array([15,5,15,1,np.pi/180]) #vote_threshold,min_linelength,max_linegap,rho,theta
        self.h_low = 130
        self.h_high = 175
        self.s_low = 55
        self.s_high = 155
        self.v_low = 0
        self.v_high = 15

        self.pause_flag = False
        self.delay = 0.1 #It will be overwritten in the get_imeges_from_bag method later
        self.time_list = []
        self.images_list = self.get_images_from_bag(path)
        self.len_images = len(self.images_list)
        self.image_index = 0
        self.factor = 1.0
        self.key_pressed_list = []
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def clahe(self,img):

        clahe = cv2.createCLAHE(clipLimit=1.0, tileGridSize=(16, 16))
        cl1 = clahe.apply(img)
        return cl1

    def equal(self,img):

        equ = cv2.equalizeHist(img)
        return equ

    def merge(self,img):

        fig = plt.figure()
        hist, bins = np.histogram(img.flatten(), 256, [0, 256])
        cdf = hist.cumsum()
        cdf_normalized = cdf * hist.max() / cdf.max()
        plt.plot(cdf_normalized, color='b')
        plt.hist(img.flatten(), 256, [0, 256], color='r')
        plt.xlim([0, 256])
        plt.legend(('cdf', 'histogram'), loc='upper left')
        fig.savefig('temp.png', dpi=fig.dpi)
        hist_img = cv2.imread('temp.png')
        resized_image = cv2.resize(hist_img, (img.shape[1],img.shape[0]))

        if len(resized_image.shape) != len(img.shape):
            resized_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
        res = np.hstack((img, resized_image))
        return res

    def image_play(self):

        if self.is_pressed("2") or self.is_pressed("8") or self.is_pressed("4") or self.is_pressed("6"):

            self.pause_flag =True
            if self.is_pressed("8"):

                self.image_index += self.factor

            elif self.is_pressed("2"):

                self.image_index -= self.factor
        else:

            self.pause_flag = False

        self.clip_index()

    def clip_index(self):

        if self.image_index < 0:
            self.image_index = self.len_images - 1

        if self.image_index >= self.len_images:
            self.image_index = 0

    def get_images_from_bag(self,path):

        bag = rosbag.Bag(path)
        images_list = []
        times_list = []
        for topic, msg, t in bag.read_messages(topics=['/ika_racer/perception/realsense/camera/color/image_raw']):
            cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            images_list.append(cv_image)
            times_list.append(t.to_sec())
        bag.close()

        self.delay = (times_list[-1] - times_list[0])/len(times_list)
        self.time_list = np.asarray(times_list)
        self.time_list = self.time_list - self.time_list[0]

        return images_list

    def get_parameters(self, key, thresholds):

        if key == "a":

            thresholds[0] += 5

        elif key == "z":

            thresholds[0] -= 5

        if key == "s":

            thresholds[1] += 5

        elif key == "x":

            thresholds[1] -= 5

        if key == "d":

            thresholds[2] += 5

        elif key == "c":

            thresholds[2] -= 5

        if key == "f":

            thresholds[3] += 0.25

        elif key == "v":

            thresholds[3] -= 0.25

        if key == "g":

            thresholds[4] += np.pi/900

        elif key == "b":

            thresholds[4] -= np.pi/900

        if key == "a" or key == "z" or key == "s" or key == "x" or key == "d" or key == "c" \
            or key == "f" or key == "v" or key == "g" or key == "b":

            thresholds[0] = np.clip(thresholds[0], 0, 100)
            thresholds[1] = np.clip(thresholds[1], 0, 100)
            thresholds[2] = np.clip(thresholds[2], 0, 100)
            thresholds[3] = np.clip(thresholds[3], 0, 10)
            thresholds[4] = np.clip(thresholds[4], np.pi/1800, np.pi)
            print("vote: " + str(thresholds[0]) + ". min_linelength: " + str(thresholds[1]) + ". max_linegap: "
                  + str(thresholds[2]) + ". rho: " + str(thresholds[3]) + ". theta: " + str(thresholds[4]))
    def is_pressed(self,key):

        if key in self.key_pressed_list:

            print("", end='\r')
            return True

        else:

            return False

    def on_press(self,key):

        try:

            if not key.char in self.key_pressed_list:
                self.key_pressed_list.append(key.char)

        except AttributeError:

            pass
            # print('special key {0} pressed'.format(key))

    def on_release(self,key):

        try:
            self.key_pressed_list.remove(key.char)
            if key.char == "4":

                self.image_index -= self.factor

            elif key.char == "6":

                self.image_index += self.factor

            self.clip_index()
            if key.char == "m":

                self.factor = int(2 * self.factor)

            elif key.char == "n":
                self.factor = int(0.5 * self.factor)
                if self.factor <= 0:
                    self.factor = 1

            self.get_parameters(key.char, self.hough_thresholds)


        except AttributeError:

            pass

    def stop_listener(self):

        self.listener.stop()

    def start_listener(self):

        self.listener.start()
