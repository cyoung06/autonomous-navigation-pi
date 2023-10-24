# do stereo vision bruh but not yet
import math

import numpy as np
import cv2 as cv


cap = cv.VideoCapture(0)
# Parameters for lucas kanade optical flow
lk_params = dict( winSize = (15, 15),
 maxLevel = 2,
 criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))
# Create some random colors
color = np.random.randint(0, 255, (100, 3))
# Take first frame and find corners in it
ret, old_frame = cap.read()
ret, old_frame = cap.read()
ret, old_frame = cap.read()
old_gray = cv.cvtColor(old_frame, cv.COLOR_BGR2GRAY)

cv.imshow('frame', old_gray)
k = cv.waitKey(1000) & 0xff
# Initiate FAST object with default values
# fast = cv.FastFeatureDetector_create()
# # find and draw the keypoints
# p0 = fast.detect(old_gray,None)
# p0 = [val.pt for val in p0]
# p0 = np.array(p0)
# p0 = np.float32(p0.reshape(-1, 1).reshape(-1, 1, 2))
# oldness = p0

height,width,channel = old_frame.shape

print(width, height)
smh = 10
p0 = np.float32(
    np.array(np.meshgrid(np.arange(smh, width-smh, smh), np.arange(smh, height - smh, smh))).T.reshape(-1, 1, 2))

mask = np.zeros_like(old_frame)

fovHeight = 60
fovWidth = width / height * fovHeight
focalLength = height / math.tan(fovHeight)

oldest = p0

culmutativeState = len(p0) * [0]
print(culmutativeState)

while(1):
    ret, frame = cap.read()
    if not ret:
        print('No frames grabbed!')
        break
    frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    # calculate optical flow
    p1, st, err = cv.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
    # Select good points
    # print(st)
    # print(st)
    # print(culmutativeState)
    if p1 is not None:
        culmutativeState = [0 if (culmutativeState[i] == 0 or st[i][0] == 0) else 1 for i in range(0, len(st))]

        good_new = p1[st==1]
        good_old = p0[st==1]

    # draw the tracks
    for i, (new, old) in enumerate(zip(good_new, good_old)):
        a, b = new.ravel()
        c, d = old.ravel()
        if ((a-c)**2 + (b-d)**2 > 1000):
            continue
        frame = cv.line(frame, (int(a), int(b)), (int(c), int(d)), color[i % 100].tolist(), 3)
        frame = cv.circle(frame, (int(a), int(b)), 5, color[i% 100].tolist(), -1)
    # img = cv.add(frame, mask)
    cv.imshow('frame', frame)
    k = cv.waitKey(30) & 0xff
    if k == 27:
        break
    # Now update the previous frame and previous points
    old_gray = frame_gray.copy()
    p0 = p1.reshape(-1, 1, 2)



good_new = p0[st==1]
good_old = oldest[st==1]
frame = old_frame

dist = 150
coords = []

depthMap = frame

for i, (old, new) in enumerate(zip(good_old, good_new)):

    a, b = new.ravel()
    c, d = old.ravel()
    if ((a-c)**2 + (b-d) **2 > 10000):
        continue

    # (d-b)*x - y*(c-a) - a*(d-b)+ b*(c-a)
    # vec1 = (c-width/2, d-width/2) -> center to old
    # vec2 = (b-d, a-c) -> Old to new

    alphaX = (c-width/2) / focalLength
    betaX = (a-width/2) / focalLength
    alphaY = (d-height/2) / focalLength
    betaY = (b-height/2) / focalLength

    if abs(betaY) < abs(alphaY):
        continue
    if abs(betaX) < abs(alphaX):
        continue

    vec1 = [c-width/2, d-height/2]
    vec2 = [a-c, b-d]
    directionality = (vec1[0] * vec2[0] + vec1[1] * vec2[1]) / math.sqrt(vec1[0] ** 2 + vec1[1] ** 2) / math.sqrt(vec2[0] ** 2 + vec2[1] ** 2)
    if directionality < 0.9:
        continue

    xCoord = dist * alphaX * betaX / (betaX - alphaX)
    yCoord = dist * alphaY * betaY / (betaY - alphaY)
    zCoord1 = xCoord / betaX
    zCoord2 = yCoord / betaY

    coords.append([xCoord, yCoord, (zCoord1+zCoord2)/2])
    print(f"{alphaX}, {betaX}, {alphaY}, {betaY}, {xCoord}, {yCoord}, {(zCoord1+zCoord2)/2}, ({zCoord1}, {zCoord2})")

    frame = cv.line(frame, (int(a), int(b)), (int(c), int(d)), color[i % 100].tolist(), 3)
    frame = cv.circle(frame, (int(a), int(b)), 5, color[i% 100].tolist(), -1)

    zCoordCoord =  (zCoord1+zCoord2)/2
    depthMap = cv.circle(depthMap, (int(a), int(b)), smh, [max(0, min(255, zCoordCoord / 10000 * 255))] * 3, -1)


cv.imshow('frame', frame)

cv.waitKey(-1)

cv.imshow('frame', depthMap)

cv.destroyAllWindows()