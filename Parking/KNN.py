import math
import cv2
import pickle
import numpy as np


cap = cv2.VideoCapture('parking_video.mp4')

with open('CarParkPorte', 'rb') as fs:
    port = pickle.load(fs)

with open('CarParkPos', 'rb') as f:
    posList = pickle.load(f)

width, height = 30, 13
boolpos = []

for i, pos in enumerate(posList):
    boolpos.append(True)


def distance(A,B):
    return math.sqrt((A[1]-B[0])**2 + (A[2]-B[1])**2)


def KNN(port,posList,boolpos):
    posi = []
    dist = []
    for i, pos in enumerate(posList):
        x,y = pos
        posi.append([x + (width / 2), y - (height / 2)])
        #print(f"({centre[i][0]},{centre[i][1]})")
        if boolpos[i]==False:
            dist.append([distance(port,posi[i]),f"S{i+1}"])
        else:
            dist.append([math.inf,f"S{i+1}"])
    for i in range(len(dist)):
        for j in range(i+1,len(dist)):
            if (dist[i][0] > dist[j][0]):
                temp = dist[j][0]
                dist[j][0] = dist[i][0]
                dist[i][0] = temp

                temp = dist[j][1]
                dist[j][1] = dist[i][1]
                dist[i][1] = temp

    return f"la plus proche place a la porte {port[0]} est :{dist[0][1]}"


def checkParkingSpace2 (imgPro):

    spaceCounter = 0
    for i, pos in enumerate(posList):
        x, y = pos

        imgCrop = imgPro[y:y + height, x:x + width]
        # cv2.imshow(str(x * y), imgCrop)
        count = cv2.countNonZero(imgCrop)

        if count < 75:
            color = (0, 255, 0)
            thickness = 1
            spaceCounter += 1
            boolpos[i] = False
        else:
            color = (0, 0, 255)
            thickness = 1
            boolpos[i] = True

        # cv2.rectangle(img, pos, (pos[0] + width, pos[1] + height), color, thickness)

    for i ,prt in enumerate(port):
        print(KNN(prt,posList,boolpos))

    #cvzone.putTextRect(img, f'places libres: {spaceCounter}/{len(posList)}', (0, 50), scale=3, thickness=5, offset=20,
    #                  colorR=(0, 200, 0))




while True:

    if cap.get(cv2.CAP_PROP_POS_FRAMES) == cap.get(cv2.CAP_PROP_FRAME_COUNT):cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
    success, img = cap.read()
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    imgBlur = cv2.GaussianBlur(imgGray, (3, 3), 1)
    imgThreshold = cv2.adaptiveThreshold(imgBlur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV, 27, 20)
    imgMedian = cv2.medianBlur(imgThreshold, 5)
    kernel = np.ones((3, 3), np.uint8)
    imgDilate = cv2.dilate(imgMedian, kernel, iterations=1)
    checkParkingSpace2(imgDilate)
    # cv2.imshow("Image", img)
    # cv2.imshow("ImageBlur", imgBlur)
    # cv2.imshow("ImageThres", imgMedian)
    cv2.waitKey(40)
