import cv2
import pickle

# width, height = 30, 13
count = 0

try:
    with open('CarParkPorte', 'rb') as fs:
        port = pickle.load(fs)
except:
    port = []


def mouseClick(events, x, y, flags, params):
    global count
    if events == cv2.EVENT_LBUTTONDOWN:
        count += 1
        port.append((count, x, y))

    with open('CarParkPorte', 'wb') as fs:
        pickle.dump(port, fs)


while True:
    img2 = cv2.imread('img_parking.jpg')
    # for i , pos in enumerate(port):
    #      cv2.rectangle(img2, pos, (pos[0] + width, pos[1] + height), (255, 0, 255), 1)

    cv2.imshow("Image2", img2)
    cv2.setMouseCallback("Image2", mouseClick)
    cv2.waitKey(1)
