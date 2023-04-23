import numpy as np
import cv2
import datetime
# create a folder for storing frames
import os

if not os.path.exists('output_frames_stulya'):
    os.makedirs('output_frames_stulya')
cap = cv2.VideoCapture('hozhu.avi')

# initializing subtractor
fgbg = cv2.bgsegm.createBackgroundSubtractorMOG()


while (1):
    ret, frame = cap.read()

    # applying on each frame
    fgmask = fgbg.apply(frame)

    cv2.imshow('frame', fgmask)
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break

    # save the frame with the current timestamp as filename
    filename = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f") + '.png'
    #cv2.imwrite('output_frames_stulya/' + filename, fgmask)

cap.release()
cv2.destroyAllWindows()
