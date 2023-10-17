import cv2
import pickle
import cvzone
import numpy as np
import time
# import math

# Video feed
cap = cv2.VideoCapture('parking_video.mp4')

with open('CarParkPos', 'rb') as f:
    posList = pickle.load(f)

width, height = 30, 13

boolpos = []
ContTime = []
for i, pos in enumerate(posList):
    boolpos.append(False)
    ContTime.append(0)


def checkParkingSpace(imgPro):
    spaceCounter = 0
    t1_start = time.perf_counter_ns()
    for i, pos in enumerate(posList):
        t1_start = time.perf_counter_ns()
        x, y = pos

        imgCrop = imgPro[y:y + height, x:x + width]
        # cv2.imshow(str(x * y), imgCrop)
        count = cv2.countNonZero(imgCrop)

        if count < 75:
            color = (0, 255, 0)
            thickness = 1
            spaceCounter += 1
            boolpos[i] = False
            ContTime[i] = 0
        else:
            color = (0, 0, 255)
            thickness = 1
            boolpos[i] = True

        cv2.rectangle(img, pos, (pos[0] + width, pos[1] + height), color, thickness)
        if boolpos[i]:
            ContTime[i] += time.perf_counter_ns() - t1_start

        cvzone.putTextRect(img, f'{(ContTime[i] * 0.00001):.1f}DH', (x, y + height), scale=0.6, thickness=0, offset=0,
                           colorR=color)

    cvzone.putTextRect(img, f'Free: {spaceCounter}/{len(posList)}', (100, 50), scale=3, thickness=5, offset=20,
                       colorR=(0, 200, 0))



while True:

    if cap.get(cv2.CAP_PROP_POS_FRAMES) == cap.get(cv2.CAP_PROP_FRAME_COUNT):cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
    success, img = cap.read()
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    imgBlur = cv2.GaussianBlur(imgGray, (3, 3), 1)
    imgThreshold = cv2.adaptiveThreshold(imgBlur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV, 27, 20)
    imgMedian = cv2.medianBlur(imgThreshold, 5)
    kernel = np.ones((3, 3), np.uint8)
    imgDilate = cv2.dilate(imgMedian, kernel, iterations=1)

    checkParkingSpace(imgDilate)
    cv2.imshow("Image", img)
    # cv2.imshow("ImageBlur", imgBlur)
    # cv2.imshow("ImageThres", imgMedian)
    cv2.waitKey(100)
