"""
AutoTill 2.0
McGill University, Department of Bioresource Engineering
Agri-Fusion 2000, Inc.
"""

__author__ = 'Trevor Stanhope'
__version__ = '2.0.'

## Libraries
import cv2, cv
import serial # Electro-hydraulic controller
import pymongo # DB
from bson import json_util # DB
from pymongo import MongoClient # DB
import json
import numpy # Curve
from matplotlib import pyplot as plt # Display
import thread # GPS
import gps # GPS
import time 
import sys
from datetime import datetime
import ast

## Constants
try:
    CONFIG_FILE = sys.argv[1]
except Exception as err:
    CONFIG_FILE = 'settings.json'

## Class
class Cultivator:
    def __init__(self, config_file):

        # Load Config
        print('\tLoading config file: %s' % config_file)
        self.config = json.loads(open(config_file).read())
        for key in self.config:
            try:
                getattr(self, key)
            except AttributeError as error:
                setattr(self, key, self.config[key])
        
        self.__init_cams__()
        self.__init_db__()
        self.__init_pid__()
        self.__init_arduino__()
        self.__init_gps__()
        ##self.__init_display__()
    
    def __init_cams__(self):
        # Cameras
        if self.VERBOSE: print('[Initialing Cameras] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
        if self.VERBOSE: print('\tImage Width: %d px' % self.PIXEL_WIDTH)
        if self.VERBOSE: print('\tImage Height: %d px' % self.PIXEL_HEIGHT)
        if self.VERBOSE: print('\tCamera Height: %d cm' % self.CAMERA_HEIGHT)
        if self.VERBOSE: print('\tCamera FOV: %f deg' % (self.CAMERA_FOV * 180 / numpy.pi))
        self.PIXEL_CENTER = self.PIXEL_WIDTH / 2
        if self.VERBOSE: print('\tImage Center: %d px' % self.PIXEL_CENTER)
        self.GROUND_WIDTH = 2 * self.CAMERA_HEIGHT * numpy.tan(self.CAMERA_FOV / 2.0)
        print('\tGround Width: %d cm' % self.GROUND_WIDTH)
        if self.VERBOSE: print('\tBrush Range: +/- %d cm' % self.BRUSH_RANGE)
        self.PIXEL_PER_CM = self.PIXEL_WIDTH / self.GROUND_WIDTH
        if self.VERBOSE: print('\tPixel-per-cm: %d px/cm' % self.PIXEL_PER_CM)
        self.PIXEL_RANGE = int(self.PIXEL_PER_CM * self.BRUSH_RANGE) 
        if self.VERBOSE: print('\tPixel Range: +/- %d px' % self.PIXEL_RANGE)
        self.cameras = []
        for i in self.CAMERAS:
            if self.VERBOSE: print('\tInitializing Camera: %d' % i)
            cam = cv2.VideoCapture(i)
            cam.set(cv.CV_CAP_PROP_FRAME_WIDTH, self.PIXEL_WIDTH)
            cam.set(cv.CV_CAP_PROP_FRAME_HEIGHT, self.PIXEL_HEIGHT)
            self.cameras.append(cam)
    
    def __init_db__(self):
        # Initialize Database
        if self.VERBOSE: print('[Initializing MongoDB] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
        self.LOG_NAME = datetime.strftime(datetime.now(), self.LOG_FORMAT)
        if self.VERBOSE: print('\tNew session: %s' % self.LOG_NAME)
        self.MONGO_NAME = datetime.strftime(datetime.now(), self.MONGO_FORMAT)
        if self.VERBOSE: print('\tConnecting to MongoDB: %s' % self.MONGO_NAME)
        try:
            self.client = MongoClient()
            self.database = self.client[self.MONGO_NAME]
            self.collection = self.database[self.LOG_NAME]
            self.log = open('logs/' + self.LOG_NAME + '.csv', 'w')
            self.log.write(','.join(['time', 'lat', 'long', 'speed', 'cam0', 'cam1', 'estimate', 'average', 'pwm','\n']))
        except Exception as error:
            print('\tERROR in __init__(): %s' % str(error))
    
    def __init_pid__(self):
        # PWM Response Range for Electrohyraulic-Control
        if self.VERBOSE: print('[Initialing Electro-Hydraulics] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
        if self.VERBOSE: print('\tDefault Number of Averages: %d' % self.NUM_AVERAGES)
        self.offset_history = [self.PIXEL_CENTER] * self.NUM_AVERAGES
    
    def __init_arduino__(self):
        # Arduino Connection
        if self.VERBOSE: print('[Initializing Arduino] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
        try:
            if self.VERBOSE: print('\tDevice: %s' % str(self.SERIAL_DEVICE))
            if self.VERBOSE: print('\tBaud Rate: %s' % str(self.SERIAL_BAUD))
            self.arduino = serial.Serial(self.SERIAL_DEVICE, self.SERIAL_BAUD)
        except Exception as error:
            print('\tERROR in __init__(): %s' % str(error))
    
    def __init_gps__(self):
        # GPS
        if self.VERBOSE: print('[Initializing GPS] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
        if self.GPS_ENABLED:
            try:
                if self.VERBOSE: print('\tWARNING: Enabing GPS')
                self.gpsd = gps.gps()
                self.gpsd.stream(gps.WATCH_ENABLE)
                thread.start_new_thread(self.update_gps, ())
            except Exception as err:
                print('\tERROR in __init__(): GPS not available! %s' % str(err))
                self.latitude = 0
                self.longitude = 0
                self.speed = 0
        else:
            print('\tWARNING: GPS Disabled')
            self.latitude = 0
            self.longitude = 0
            self.speed = 0
    
    def __init_display__(self):
        if self.VERBOSE: print('[Initializing Display] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
        try:
            self.master = Tkinter.Tk()
            pad = 3
            self._geom='640x480+0+0'
            self.master.config(background = "#000000")
            self.master.geometry("{0}x{1}+0+0".format(
                self.master.winfo_screenwidth() - pad,
                self.master.winfo_screenheight() - pad)
            )
            self.master.overrideredirect(False) # make fullscreen
            self.master.focus_set()
            self.master.state("normal")
            self.master.update_idletasks()
            
        except Exception as error:
            print('\tERROR in __init_display__: %s' % str(error))
    
    ## Capture Images
    """
    1. Attempt to capture an image
    2. Repeat for each capture interface
    """
    def capture_images(self):
        if self.VERBOSE: print('[Capturing Images] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
        images = []
        for cam in self.cameras:
            if self.VERBOSE: print('\tCamera ID: %s' % str(cam))
            (s, bgr) = cam.read() 
            if s:
                images.append(bgr)
        if self.VERBOSE: print('\tImages captured: %d' % len(images))
        return images
        
    ## Green Filter
    """
    1. RBG --> HSV
    2. Set minimum saturation equal to the mean saturation
    3. Set minimum value equal to the mean value
    4. Take hues within range from green-yellow to green-blue
    """
    def plant_filter(self, images):
        if self.VERBOSE: print('[Filtering for Plants] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
        masks = []
        for bgr in images:
            try:
                hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
                hue_min = self.HUE_MIN # yellowish
                hue_max = self.HUE_MAX # bluish
                sat_min = hsv[:,:,1].mean() # cutoff for how saturated the color must be
                sat_max = self.SAT_MAX
                val_min = hsv[:,:,2].mean()
                val_max = self.VAL_MAX
                threshold_min = numpy.array([hue_min, sat_min, val_min], numpy.uint8)
                threshold_max = numpy.array([hue_max, sat_max, val_max], numpy.uint8)
                mask = cv2.inRange(hsv, threshold_min, threshold_max)
                masks.append(mask) 
            except Exception as error:
                print('\tERROR in plant_filter(): %s' % str(error))        
        if self.VERBOSE: print('\tNumber of Masks: %d mask(s) ' % len(masks))
        return masks
        
    ## Find Plants
    """
    1. Calculates the column summation of the mask
    2. Calculates the 95th percentile threshold of the column sum array
    3. Finds indicies which are greater than or equal to the threshold
    4. Finds the median of this array of indices
    5. Repeat for each mask
    """
    def find_indices(self, masks):
        if self.VERBOSE: print('[Finding Offsets] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
        indices = []
        for mask in masks:
            try:
                column_sum = mask.sum(axis=0) # vertical summation
                threshold = numpy.percentile(column_sum, self.THRESHOLD_PERCENTILE)
                probable = numpy.nonzero(column_sum >= threshold) # returns 1 length tuble
                num_probable = len(probable[0])
                centroid = int(numpy.median(probable[0]))
                indices.append(centroid)
            except Exception as error:
                print('\tERROR in find_indices(): %s' % str(error))
        if self.VERBOSE: print('\tDetected Indices: %s' % str(indices))
        return indices
        
    ## Row Estimate (P), Average (I) and Change (D)
    """
    1. Calculate the current estimate
    2. Append estimate to history
    3. Calculate the average
    4. Calculate the change since last iteration
    """
    def calculate_eac(self, indices):
        if self.VERBOSE: print('[Calculating Values] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
        try:
            estimate = int(numpy.mean(indices)) - self.PIXEL_CENTER
        except Exception as error:
            print('\ERROR in estimate_row(): %s' % str(error))
            estimate = 0
        self.offset_history.append(estimate)
        while len(self.offset_history) > self.NUM_AVERAGES:
            self.offset_history.pop(0)
        average = int(numpy.mean(self.offset_history))
        change = int(self.offset_history[-1] -  self.offset_history[-2])
        if self.VERBOSE: print('\tEstimate: %s' % str(estimate))    
        if self.VERBOSE: print('\tAverage: %s' % str(average)) 
        if self.VERBOSE: print('\tChange: %s' % str(change)) 
        return (estimate, average, change)
             
    ## Control Hydraulics
    """
    1. Get PWM response corresponding to PID values
    2. Send PWM response over serial to controller
    """
    def calculate_pid(self, estimate, average, change):
        if self.VERBOSE: print('[Calculating PID] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
        P = self.P_COEF * 50 * estimate / float(self.PIXEL_RANGE)
        I = self.I_COEF * 50 * average / float(self.PIXEL_RANGE)
        D = self.D_COEF * 50 * change / float(self.PIXEL_RANGE)
        if self.VERBOSE: print('\tP: %s%%' % str(P))    
        if self.VERBOSE: print('\tI: %s%%' % str(I)) 
        if self.VERBOSE: print('\tD: %s%%' % str(D))
        return (P, I, D)
    
    def control_hydraulics(self, P, I, D):
        pwm = 50 - int(P + I + D)
        if pwm > 100:
            pwm = 100
        elif pwm < 0:
            pwm = 0
        try:
            self.arduino.write(str(pwm) + '\n')
        except Exception as error:
            print('\tERROR in control_hydraulics(): %s' % str(error))
        if self.VERBOSE: print('\tPWM Output: %s' % str(pwm))
        return pwm
    
    ## Logs to Mongo
    """
    1. Log results to the database
    2. Returns Doc ID
    """
    def log_db(self, sample):
        if self.VERBOSE: print('[Logging to Database] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
        try:          
            doc_id = self.collection.insert(sample)
        except Exception as error:
            print('\tERROR in log_db(): %s' % str(error))
        if self.VERBOSE: print('\tDoc ID: %s' % str(doc_id))
        return doc_id
    
    ## Log to File
    """
    1. Open new text file
    2. For each document in session, print parameters to file
    """
    def log_file(self, sample):
        print('[Logging to File] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
        try:
            time = str(sample['time'])
            latitude = str(sample['lat'])
            longitude = str(sample['long'])
            speed = str(sample['speed'])
            cam0 = str(sample['cam0'])
            cam1 = str(sample['cam1'])
            estimate = str(sample['estimate'])
            average = str(sample['average'])
            pwm = str(sample['pwm'])
            self.log.write(','.join([time, latitude, longitude, speed, cam0, cam1, estimate, average, pwm,'\n']))
        except Exception as error:
            print('\tERROR: %s' % str(error))
                
    ## Displays 
    """
    1. Draw lines on RGB images
    2. Draw lines on EGI images (the masks)
    3. Output GUI display
    """
    def update_display(self, images, masks, results):
        if self.VERBOSE: print('[Displaying Images] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
        ## Draw lines on Images
        estimate = results['estimate'] + self.PIXEL_CENTER
        average = results['average'] + self.PIXEL_CENTER
        mod_images = []
        for img in images:
            try:
                cv2.line(img, (estimate, 0), (estimate, self.PIXEL_HEIGHT), (127,0,255), 3)
                cv2.line(img, (average, 0), (average, self.PIXEL_HEIGHT), (127,255,0), 3)
                mod_images.append(img)
            except Exception as error:
                print('\tERROR in display(): %s' % str(error))   

        ## Generate output
        try:
            output = numpy.hstack(mod_images)
            cv2.namedWindow("AutoTill", cv2.WND_PROP_FULLSCREEN)          
            cv2.setWindowProperty("AutoTill", cv2.WND_PROP_FULLSCREEN, cv2.cv.CV_WINDOW_FULLSCREEN)
            cv2.imshow('AutoTill', output)
            if cv2.waitKey(5) == 27:
                pass
        except Exception as error:
            print('\tERROR in display(): %s' % str(error))
                    
    ## Update GPS
    """
    1. Get the most recent GPS data
    2. Set global variables for lat, long and speed
    """
    def update_gps(self):  
        while True:
            print('[Updating GPS] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
            self.gpsd.next()
            self.latitude = self.gpsd.fix.latitude
            self.longitude = self.gpsd.fix.longitude
            self.speed = self.gpsd.fix.speed
    
    ## Close
    """
    Function to shutdown application safely
    1. Close windows
    2. Disable arduino
    3. Release capture interfaces 
    """
    def close(self):
        if self.VERBOSE: print('[Shutting Down] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
        cv2.destroyAllWindows() # Close windows
        try:
            if self.VERBOSE: print('\tClosing Arduino')
            self.arduino.close() # Disable arduino
        except Exception as error:
            print('\tERROR in close(): %s' % str(error))
        for i in range(len(self.cameras)):
            try:
                if self.VERBOSE: print('\tClosing Camera #%d' % i)
                self.cameras[i].release() # Disable cameras
            except Exception as error:
                print('\tERROR in close(): %s' % str(error))
                
    ## Throttle Frequency
    """
    1. While the frequency is less than the limit, wait
    """ 
    def throttle_frequency(self, start_time):
        if self.VERBOSE: print('[Throttling Frequency] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
        while (1 / (time.time() - start_time)) > self.FREQUENCY_LIMIT:
            time.sleep(0.01)
        frequency = 1/(time.time() - start_time)
        if self.VERBOSE: print('\tFrequency: ' + str(frequency))
        return frequency
        
    ## Run  
    """
    Function for Run-time loop
    1. Get initial time
    2. Capture images
    3. Generate mask filter for plant matter
    4. Calculate indices of rows
    5. Estimate row from both images
    6. Calculate moving average
    7. Send PWM response to arduino
    8. Throttle to desired frequency
    9. Log results to DB
    10. Display results
    """     
    def run(self):
        while True:
            try:
                print('---------------------------------------')
                start_time = time.time()
                images = self.capture_images()
                masks = self.plant_filter(images)
                indices = self.find_indices(masks)
                (estimate, average, change) = self.calculate_eac(indices)
                (P, I, D) = self.calculate_pid(estimate, average, change)
                pwm = self.control_hydraulics(P, I, D)
                frequency = self.throttle_frequency(start_time)
                try:
                    cam0 = indices[0] - self.PIXEL_CENTER
                except Exception:
                    cam0 = None
                try:
                    cam1 = indices[1] - self.PIXEL_CENTER
                except Exception:
                    cam1 = None
                sample = {
                    'cam0' : cam0,
                    'cam1' : cam1,
                    'estimate' : estimate,
                    'average' : average,
                    'change': change,
                    'P' : P,
                    'I' : I,
                    'D' : D,
                    'pwm': pwm,
                    'time' : datetime.strftime(datetime.now(), self.TIME_FORMAT),
                    'frequency' : frequency,
                    'long' : self.longitude,
                    'lat' : self.latitude,
                    'speed' : self.speed,
                }
                if self.MONGO_ON: doc_id = self.log_db(sample)
                if self.LOGFILE_ON: self.log_file(sample)
                if self.DISPLAY_ON: self.update_display(images, masks, sample)
            except KeyboardInterrupt as error:
                break

## Main
if __name__ == '__main__':
    session = Cultivator(CONFIG_FILE)
    session.run()
