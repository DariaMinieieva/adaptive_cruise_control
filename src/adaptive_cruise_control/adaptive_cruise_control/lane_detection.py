import cv2
import numpy as np

curveList = []
avgVal = 7

def thresholding(img):
    # imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # lowerWhite = np.array([80, 0, 0])
    # upperWhite = np.array([255, 160, 255])
    # maskWhite = cv2.inRange(imgHsv, lowerWhite, upperWhite)
    hls = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hls, (50, 140, 30), (125, 255, 255))
    return mask


def warpImg(img, points, w, h, inv=False):
    pts1 = np.float32(points)
    pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
    if inv:
        matrix = cv2.getPerspectiveTransform(pts2, pts1)
    else:
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
    imgWarp = cv2.warpPerspective(img, matrix, (w, h))
    return img


def nothing(a):
    pass


def initializeTrackbars(intialTracbarVals, wT=480, hT=240):
    cv2.namedWindow("Trackbars")
    cv2.resizeWindow("Trackbars", 360, 240)
    cv2.createTrackbar("Width Top", "Trackbars", intialTracbarVals[0], wT // 2, nothing)
    cv2.createTrackbar("Height Top", "Trackbars", intialTracbarVals[1], hT, nothing)
    cv2.createTrackbar("Width Bottom", "Trackbars", intialTracbarVals[2], wT // 2, nothing)
    cv2.createTrackbar("Height Bottom", "Trackbars", intialTracbarVals[3], hT, nothing)


def valTrackbars(width=250, height=250):
    points = np.float32([
        (width * 0.1, height * 0.2),  # Top-left corner
        (0, height),  # Bottom-left corner
        (width, height),  # Bottom-right corner
        (width * 0.9, height * 0.2)  # Top-right corner
    ])
    # points = np.float32([(widthTop, heightTop), (wT - widthTop, heightTop),
    #                      (widthBottom, heightBottom), (wT - widthBottom, heightBottom)])
    return points


def drawPoints(img, points):
    for x in range(4):
        cv2.circle(img, (int(points[x][0]), int(points[x][1])), 15, (0, 0, 255), cv2.FILLED)
    return img


def getHistogram(img, minPer=0.1, display=False, region=1):
    if region == 1:
        histValues = np.sum(img, axis=0)
    else:
        histValues = np.sum(img[img.shape[0] // region:, :], axis=0)

    # print(histValues)
    maxValue = np.max(histValues)
    minValue = minPer * maxValue

    indexArray = np.where(histValues >= minValue)
    basePoint = int(np.average(indexArray))
    # print(basePoint)

    if display:
        imgHist = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
        for x, intensity in enumerate(histValues):
            cv2.line(imgHist, (x, img.shape[0]), (x, int(img.shape[0] - intensity // 255 // region)), (255, 0, 255), 1)
            cv2.circle(imgHist, (basePoint, img.shape[0]), 20, (0, 255, 255), cv2.FILLED)
        return basePoint, imgHist

    return basePoint


def stackImages(scale, imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range(0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape[:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]),
                                                None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y] = cv2.cvtColor(imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank] * rows
        hor_con = [imageBlank] * rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None, scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor = np.hstack(imgArray)
        ver = hor
    return ver


def getLaneCurve(img, display=2):
    LEN_AVG = 7
    weights = np.linspace(0, 1, LEN_AVG)
    imgCopy = img.copy()
    imgResult = img.copy()
    #### STEP 1
    imgThres = thresholding(img)

    #### STEP 2
    hT, wT, c = img.shape
    points = valTrackbars()
    imgWarp = warpImg(imgThres, points, wT, hT)
    imgWarpPoints = drawPoints(imgCopy, points)

    #### STEP 3
    middlePoint, imgHist = getHistogram(imgWarp, display=True, minPer=0.5, region=5)
    curveAveragePoint, imgHist = getHistogram(imgWarp, display=True, minPer=0.2)
    curveRaw = ((wT // 2) - curveAveragePoint) / wT * 100

    #### STEP 4
    curveList.append(curveRaw)
    if len(curveList) > LEN_AVG:
        curveList.pop(0)
        curve = int(np.average(curveList, weights=weights, axis=-1))
    else:
        curve = int(np.average(curveList))

    #### STEP 5
    if display != 0:
        imgInvWarp = warpImg(imgWarp, points, wT, hT, inv=True)
        imgInvWarp = cv2.cvtColor(imgInvWarp, cv2.COLOR_GRAY2BGR)
        # imgInvWarp[0:hT // 3, 0:wT] = 0, 0, 0
        imgLaneColor = np.zeros_like(img)
        imgLaneColor[:] = 0, 255, 0
        imgLaneColor = cv2.bitwise_and(imgInvWarp, imgLaneColor)
        imgResult = cv2.addWeighted(imgResult, 1, imgLaneColor, 1, 0)
        midY = 450
        cv2.putText(imgResult, str(curve), (wT // 2 - 80, 85), cv2.FONT_HERSHEY_COMPLEX, 2, (255, 0, 255), 3)
        cv2.line(imgResult, (wT // 2, midY), (wT // 2 + (curve * 3), midY), (255, 0, 255), 5)
        cv2.line(imgResult, ((wT // 2 + (curve * 3)), midY - 25), (wT // 2 + (curve * 3), midY + 25), (0, 255, 0), 5)
        for x in range(-30, 30):
            w = wT // 20
            cv2.line(imgResult, (w * x + int(curve // 50), midY - 10),
                     (w * x + int(curve // 50), midY + 10), (0, 0, 255), 2)
        # fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);
        # cv2.putText(imgResult, 'FPS ' + str(int(fps)), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (230, 50, 50), 3);
    if display == 2:
        imgStacked = stackImages(0.7, ([img, imgWarpPoints, imgWarp],
                                             [imgHist, imgLaneColor, imgResult]))
        cv2.imshow('ImageStack', imgStacked)
    elif display == 1:
        cv2.imshow('Resutlt', imgResult)

    #### NORMALIZATION
    curve = curve / 1.25
    curve = min(curve, 90)
    curve = max(curve, -90)

    return curve


if __name__ == '__main__':
    # cap = cv2.VideoCapture('vid1.mp4')
    # intialTrackBarVals = [102, 80, 20, 214]
    # initializeTrackbars(intialTrackBarVals)
    # frameCounter = 0
    # while True:
    #     frameCounter += 1
    #     if cap.get(cv2.CAP_PROP_FRAME_COUNT) == frameCounter:
    #         cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
    #         frameCounter = 0

        # success, img = cap.read()
    img = cv2.imread('/home/turtlehouse/adaptive_cruise_control/outputs/test.png')
    curve = getLaneCurve(img, display=2)
    print(curve)
    # cv2.imshow('Vid', img)
    cv2.waitKey(0)