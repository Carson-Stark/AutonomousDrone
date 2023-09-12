# Code from: https://learnopencv.com/object-detection-using-yolov5-and-opencv-dnn-in-c-and-python/

from time import sleep
import cv2
import numpy as np
import os

class ObjectDetection:

    # Constants.
    INPUT_WIDTH = 416
    INPUT_HEIGHT = 416
    SCORE_THRESHOLD = 0.5
    NMS_THRESHOLD = 0.45
    CONFIDENCE_THRESHOLD = 0.45
    
    # Text parameters.
    FONT_FACE = cv2.FONT_HERSHEY_SIMPLEX
    FONT_SCALE = 0.7
    THICKNESS = 1
    
    # Colors.
    BLACK  = (0,0,0)
    BLUE   = (255,178,50)
    YELLOW = (0,255,255)

    classes = ["plastic bag"]

    def __init__ (self):
        classes = ["plastic bag"]

        # Give the weight files to the model and load the network using them.
        modelWeights = "last.onnx"
        script_dir = os.path.dirname(__file__) #<-- absolute dir the script is in
        weights_file_path = os.path.join(script_dir, modelWeights)
        self.net = cv2.dnn.readNet(weights_file_path)

        #use GPU
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

    def draw_label(self, im, label, x, y):
        """Draw text onto image at location."""
        # Get text size.
        text_size = cv2.getTextSize(label, self.FONT_FACE, self.FONT_SCALE, self.THICKNESS)
        dim, baseline = text_size[0], text_size[1]
        # Use text size to create a BLACK rectangle.
        cv2.rectangle(im, (x,y), (x + dim[0], y + dim[1] + baseline), (0,0,0), cv2.FILLED)
        # Display text inside the rectangle.
        cv2.putText(im, label, (x, y + dim[1]), self.FONT_FACE, self.FONT_SCALE, self.YELLOW, self.THICKNESS, cv2.LINE_AA)

    def process(self, input_image, net):
        # Create a 4D blob from a frame.
        blob = cv2.dnn.blobFromImage(input_image, 1/255,  (self.INPUT_WIDTH, self.INPUT_HEIGHT), [0,0,0], 1, crop=False)
    
        # Sets the input to the network.
        net.setInput(blob)
    
        # Run the forward pass to get output of the output layers.
        outputs = net.forward(net.getUnconnectedOutLayersNames())
        return outputs

    def post_process(self, input_image, outputs):
        # Lists to hold respective values while unwrapping.
        class_ids = []
        confidences = []
        boxes = []
        # Rows.
        rows = outputs[0].shape[1]
        image_height, image_width = input_image.shape[:2]
        # Resizing factor.
        x_factor = image_width / self.INPUT_WIDTH
        y_factor =  image_height / self.INPUT_HEIGHT
        # Iterate through detections.
        for r in range(rows):
            row = outputs[0][0][r]
            confidence = row[4]
            # Discard bad detections and continue.
            if confidence >= self.CONFIDENCE_THRESHOLD:
                classes_scores = row[5:]
                # Get the index of max class score.
                class_id = np.argmax(classes_scores)
                #  Continue if the class score is above threshold.
                if (classes_scores[class_id] > self.SCORE_THRESHOLD):
                    confidences.append(confidence)
                    class_ids.append(class_id)
                    cx, cy, w, h = row[0], row[1], row[2], row[3]
                    left = int((cx - w/2) * x_factor)
                    top = int((cy - h/2) * y_factor)
                    width = int(w * x_factor)
                    height = int(h * y_factor)
                    box = np.array([left, top, width, height])
                    boxes.append(box)

        detections = list()
        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.CONFIDENCE_THRESHOLD, self.NMS_THRESHOLD)
        for i in indices:
            detections.append ((class_ids[i], confidences[i], boxes[i]))

        return detections

    def draw_detections (self, image, detections):
        for detection in detections:
            box = detection[2]
            left = box[0]
            top = box[1]
            width = box[2]
            height = box[3]             
            # Draw bounding box.             
            cv2.rectangle(image, (left, top), (left + width, top + height), self.BLUE, 3*self.THICKNESS)
            # Class label.                      
            label = "{}:{:.2f}".format(self.classes[detection[0]], detection[1])             
            # Draw label.             
            self.draw_label(image, label, left, top)

        return image
    
    def detect (self, image):
        output = self.process(image, self.net)
        detections = self.post_process(image.copy(), output)
        return detections