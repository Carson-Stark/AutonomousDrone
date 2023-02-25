import cv2;
import datetime;

class Recorder:

    def __init__ (self, width, height):
        self.capture = 

        timestamp = datetime.datetime.now()
        path = os.path.expanduser(f'~/Desktop/Flight Recordings/recording{timestamp}.mp4')
        self.writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'XVID'), 5, (int(width/2.0), height);