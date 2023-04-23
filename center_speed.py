import cv2
import numpy as np
from yoloBot.yolo import yolo_ner
from mapping import create_map


# read input video
cap = cv2.VideoCapture('hozhu_2.avi')

# initialize background subtractor
fgbg = cv2.createBackgroundSubtractorMOG2()

# initialize variables for previous center of mass
prev_cx = None
prev_cy = None
k = 0
while True:
    # read frame from video
    ret, frame = cap.read()

    if not ret:
        break

    # apply background subtraction
    # fgmask = fgbg.apply(frame)
    #
    # # apply morphology operations to remove noise
    # kernel = np.ones((5,5),np.uint8)
    # opening = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel)
    # closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
    closing, contours = yolo_ner(frame)
    path_metr, obstacles = create_map(contours)

    # if k >200:
    #     cv2.imshow("fff",closing)
    # k +=1
    # find contours of objects
    # imgray = cv2.cvtColor(closing, cv2.COLOR_BGR2GRAY)
    # ret, thresh = cv2.threshold(imgray, 127, 255, 0)
    # contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
    # draw bounding boxes and calculate center of masses
        for contour in contours:
            # area = cv2.contourArea(contour)
            # if area > 2500: # filter out small objects
                # x,y,w,h = cv2.boundingRect(contour)
                # cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
            if len(contour) > 0:
                M = cv2.moments(contour[0])
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(closing,(cx,cy),3,(0,0,255),-1)

                # calculate speed of center of mass
                if prev_cx is not None and prev_cy is not None:
                    distance = np.sqrt((cx - prev_cx)**2 + (cy - prev_cy)**2)
                    speed = distance / 30 # assuming video is recorded at 1 fps
                    print("Center of mass speed: ", speed)

                # update previous center of mass
                prev_cx = cx
                prev_cy = cy

        # display output video
    cv2.imshow('frame',closing)
    filename = fr'C:\Users\Eugene\PycharmProjects\vichFon\yoloBot\output\image{k}.jpg'
    k += 1
    #cv2.imwrite(filename,closing)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
