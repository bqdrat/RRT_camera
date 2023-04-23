import cv2
import os

# Path to the folder containing the images
path = r'C:\Users\Eugene\PycharmProjects\vichFon\yoloBot\output'

# Get a list of all the image file names in the folder
img_names = os.listdir(path)

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
video_name = os.path.join('C:/Users/Eugene/PycharmProjects/vichFon/video/', 'output.mp4')  # Change the path as required
fps = 30.0  # Change this to set the frame rate of the video
frame_size = (640, 480)  # Change this to set the size of the frames in the video
video_writer = cv2.VideoWriter(video_name, fourcc, fps, frame_size)

# Loop through each image and add it to the video
for img_name in img_names:
    img_path = os.path.join(path, img_name)
    img = cv2.imread(img_path)
    video_writer.write(img)

# Release the VideoWriter and destroy all windows
video_writer.release()
cv2.destroyAllWindows()
