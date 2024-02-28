import BlobDetection

import cv2

# Open the video file
video_path = '~/DroneVideos/.mp4'  # Replace with your video file path
cap = cv2.VideoCapture(video_path)

# Check if the video file was opened successfully
if not cap.isOpened():
    print("Error opening video file")
    exit()

# Get the frames per second (fps) and frame size of the video
fps = int(cap.get(cv2.CAP_PROP_FPS))
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Define the coordinates for cropping (first quadrant)
crop_x = 0
crop_y = 0
crop_width = frame_width // 2
crop_height = frame_height // 2

# Define the codec and create a VideoWriter object to save the cropped video
output_path = 'cropped_video.mp4'  # Replace with your output video file path
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter(output_path, fourcc, fps, (crop_width, crop_height))

while True:
    ret, frame = cap.read()
    
    if not ret:
        break
    
    # Crop the frame to the first quadrant
    cropped_frame = frame[crop_y:crop_y + crop_height, crop_x:crop_x + crop_width]
    
    # Write the cropped frame to the output video
    out.write(cropped_frame)
    
    # Display the cropped frame (optional)
    cv2.imshow('Cropped Frame', cropped_frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and writer objects
cap.release()
out.release()

# Close any open windows
cv2.destroyAllWindows()
