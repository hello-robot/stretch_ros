#! /usr/bin/python3

import cv2
import imutils
import numpy as np
import math


# Threshold value of the binary thresholding stage
THRESH_VALUE = 120
# The max threshold value each pixel below THRESH_VALUE is set to
MAX_THRESH_VALUE = 255
# Min and max values for contour areas of human body
MIN_CNTR_HUMN_AREA = 4000
MAX_CNTR_HUMN_AREA = 30000
F_T=1.7681;
F_RGB=1.93;

xs_RGB=2.16;
ys_RGB=3.84;
xs_T=1.92;
ys_T=1.44;

Tx=150;
Ty=150;



HEAT_COLORMAP = 2


class KalmanFilter(object):
        def __init__(self, dt, u_x,u_y, std_acc, x_std_meas, y_std_meas):
            """
            :param dt: sampling time (time for 1 cycle)
            :param u_x: acceleration in x-direction
            :param u_y: acceleration in y-direction
            :param std_acc: process noise magnitude
            :param x_std_meas: standard deviation of the measurement in x-direction
            :param y_std_meas: standard deviation of the measurement in y-direction
            """
            # Define sampling time
            self.dt = dt
            # Define the  control input variables
            self.u = np.matrix([[u_x],[u_y]])
            # Intial State
            self.x = np.matrix([[0], [0], [0], [0]])
            # Define the State Transition Matrix A
            self.A = np.matrix([[1, 0, self.dt, 0],
                                [0, 1, 0, self.dt],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])
            # Define the Control Input Matrix B
            self.B = np.matrix([[(self.dt**2)/2, 0],
                                [0, (self.dt**2)/2],
                                [self.dt,0],
                                [0,self.dt]])
            # Define Measurement Mapping Matrix
            self.H = np.matrix([[1, 0, 0, 0],
                                [0, 1, 0, 0]])
            #Initial Process Noise Covariance
            self.Q = np.matrix([[(self.dt**4)/4, 0, (self.dt**3)/2, 0],
                                [0, (self.dt**4)/4, 0, (self.dt**3)/2],
                                [(self.dt**3)/2, 0, self.dt**2, 0],
                                [0, (self.dt**3)/2, 0, self.dt**2]]) * std_acc**2
            #Initial Measurement Noise Covariance
            self.R = np.matrix([[x_std_meas**2,0],
                               [0, y_std_meas**2]])
            #Initial Covariance Matrix
            self.P = np.eye(self.A.shape[1])

            
        def predict(self):
                # Refer to :Eq.(9) and Eq.(10)  in https://machinelearningspace.com/object-tracking-simple-implementation-of-kalman-filter-in-python/?preview_id=1364&preview_nonce=52f6f1262e&preview=true&_thumbnail_id=1795
                # Update time state
                #x_k =Ax_(k-1) + Bu_(k-1)     Eq.(9)
                self.x = np.dot(self.A, self.x) + np.dot(self.B, self.u)
                # Calculate error covariance
                # P= A*P*A' + Q               Eq.(10)
                self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
                return self.x[0:2]

        def update(self, z):
               # Refer to :Eq.(11), Eq.(12) and Eq.(13)  in https://machinelearningspace.com/object-tracking-simple-implementation-of-kalman-filter-in-python/?preview_id=1364&preview_nonce=52f6f1262e&preview=true&_thumbnail_id=1795
                    # S = H*P*H'+R
                    S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
                    # Calculate the Kalman Gain
                    # K = P * H'* inv(H*P*H'+R)
                    K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))  #Eq.(11)
                    self.x = np.round(self.x + np.dot(K, (z - np.dot(self.H, self.x))))   #Eq.(12)
                    I = np.eye(self.H.shape[1])
                    # Update error covariance matrix
                    self.P = (I - (K * self.H)) * self.P   #Eq.(13)
                    return self.x[0:2]




def track (frame1, frame2, KF,hog,backSub):
   
    frame1 = imutils.resize(frame1, 
                       width=min(300, frame1.shape[1]))
    phi=math.pi
    theta=math.pi
    Ry=np.array([[math.cos(phi),0,math.sin(phi)],[0,1,0],[-math.sin(phi),0,math.cos(phi)]])
    Rx=np.array([[1,0,0],[0,math.cos(theta),-math.sin(theta)],[0,math.sin(theta),math.cos(theta)]])
    Rz=np.array([[math.cos(phi),0,math.sin(phi)],[0,1,0],[-math.sin(phi),0,math.cos(phi)]])

    Cint_RGB= np.array([[F_RGB/xs_RGB , 0, frame1.shape[1]/2],[0,F_RGB/ys_RGB, frame1.shape[0]/2],[0,0,1]])
    Cint_T= np.array([[F_T/xs_T , 0, frame2.shape[1]/2],[0,F_T/ys_T, frame2.shape[0]/2],[0,0,1]])
    Cext_RGB_T=np.array([[1,0,Tx],[0,1,Ty],[0,0,1.5]])

    H_RGB_T=np.matmul(Cint_RGB,np.matmul(Cext_RGB_T,np.linalg.inv(Cint_T)))
    
    frame2 = cv2.warpPerspective(frame2, H_RGB_T, [frame1.shape[1],frame1.shape[0]])
    
    #frame2 = imutils.resize(frame2, 
                       #width=frame1.shape[1])
    
    # using a greyscale picture, also for faster detection
    grayimg1 = cv2.cvtColor(frame1, cv2.COLOR_RGB2GRAY)
  

    #RGB detection
    boxes, weights = hog.detectMultiScale(grayimg1, winStride=(8,8),scale=1.02)

    boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])
     
      
    

    #thermal detection
    fgMask = backSub.apply(frame2)

    kernel = np.ones((2,2),np.uint8)
    fgMask = cv2.dilate(fgMask, kernel, iterations=2)

    # Contour detection stage
    contours, _ = cv2.findContours(fgMask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    areas = [cv2.contourArea(c) for c in contours]
    centers=[]
    for idx, val in enumerate(areas):
         for (xA1, yA1, xB1, yB1) in boxes:
        # Human blob filtration stage
            if MIN_CNTR_HUMN_AREA <= val <= MAX_CNTR_HUMN_AREA:
                cntr = contours[idx]
                # Fitting bounding boxes over our contours of interest (humans)
                x,y,w,h = cv2.boundingRect(cntr)
                # Final bounding box coordinates
                xA2 = x
                yA2 = y
                xB2 = x+w
                yB2 = y+h
                if boxes.size==0:
                    cv2.rectangle(frame1,(xA2,yA2),(xB2,yB2),(0,0,255),2)
               
                else :
                    dx = min(xB1, xB2) - max(xA1, xA2)
                    dy = min(yB1, yB2) - max(yA1, yA2)
                    #cv2.rectangle(frame1,(xA1,yA1),(xB1,yB1),(0,255,0),2)
                    cv2.rectangle(frame2,(xA2,yA2),(xB2,yB2),(0,0,255),2)
                
                    if (dx>=0) and (dy>=0):
                        xA = max(xA1, xA2)
                        yA = max(yA1, yA2)
                        xB = min(xB1, xB2)
                        yB = min(yB1, yB2)
                        cv2.rectangle(frame1,(xA,yA),(xB,yB),(255,0,0),2)
                        x=(xA+xB)/2
                        y=(yA+yB)/2
                        centers.append(np.array([[x],[y]]))
   
    if (len(centers) > 0):   
            
            (x1, y1) = KF.update(centers[0])
    else:
            (x1,y1)=KF.predict()

    print(x1,y1)

    cv2.rectangle(frame1, (int (x1) - 30, int (y1) - 80), (int (x1) + 30, int (y1) + 80), (0, 255, 0), 2)

    return frame1


class ConnectToROS:
    def __init__(self):

        cv2.startWindowThread()

        #create output video 
        self.out = cv2.VideoWriter(
            'output_rgb.avi',
            cv2.VideoWriter_fourcc(*'MJPG'),
            15.,
            (640,480))

        #initialize Kalman filter and HOG SVM
        self.KF = KalmanFilter(0.01, 1, 0, 0, .1,.1)
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        #background substractor
        self.backSub = cv2.createBackgroundSubtractorMOG2()

    def processData(self,frame1,frame2):
        frame=track(frame1,frame2,self.KF,self.hog,self.backSub);
        return frame