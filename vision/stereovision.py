# do stereo vision bruh but not yet
import math

import numpy as np
import cv2 as cv



import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt


cap = cv.VideoCapture(0)
cap.read()
cap.read()
ret, imgL = cap.read()
imgL = cv.cvtColor(imgL, cv.COLOR_BGR2GRAY)

cv.imshow('frame', imgL)
cv.waitKey(-1)
ret, imgR = cap.read()
imgR = cv.cvtColor(imgR, cv.COLOR_BGR2GRAY)

cv.imshow('frame', imgR)

stereo = cv.StereoBM.create(numDisparities=16, blockSize=31)
disparity = stereo.compute(imgL, imgR)


cv.waitKey(-1)
plt.imshow(disparity,'gray')
plt.show()
cv.destroyAllWindows()