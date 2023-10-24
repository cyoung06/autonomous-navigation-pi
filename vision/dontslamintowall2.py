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
prev_gray = cv.cvtColor(old_frame, cv.COLOR_BGR2GRAY)

cv.imshow('frame', prev_gray)
k = cv.waitKey(-1) & 0xff
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

mask = np.zeros_like(old_frame)
mask[..., 1] = 255

fovHeight = 60
fovWidth = width / height * fovHeight
focalLength = height / math.tan(fovHeight)


while (cap.isOpened()):

    # ret = a boolean return value from getting
    # the frame, frame = the current frame being
    # projected in the video
    ret, frame = cap.read()

    # Opens a new window and displays the input
    # frame
    cv.imshow("frame", frame)

    # Converts each frame to grayscale - we previously
    # only converted the first frame to grayscale
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # Calculates dense optical flow by Farneback method
    flow = cv.calcOpticalFlowFarneback(prev_gray, gray,
                                       None,
                                       0.5, 3, 15, 3, 5, 1.2, 0)

    # Computes the magnitude and angle of the 2D vectors
    magnitude, angle = cv.cartToPolar(flow[..., 0], flow[..., 1])

    # Sets image hue according to the optical flow
    # direction
    mask[..., 0] = angle * 180 / np.pi / 2

    # Sets image value according to the optical flow
    # magnitude (normalized)
    mask[..., 2] = cv.normalize(magnitude, None, 0, 255, cv.NORM_MINMAX)

    # Converts HSV to RGB (BGR) color representation
    rgb = cv.cvtColor(mask, cv.COLOR_HSV2BGR)

    # Opens a new window and displays the output frame
    cv.imshow("frame", rgb)

    # Updates previous frame
    prev_gray = gray

    # Frames are read by intervals of 1 millisecond. The
    # programs breaks out of the while loop when the
    # user presses the 'q' key
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

frame = old_frame

dist = 150
coords = []

depthMap = frame

prev_gray = cv.cvtColor(old_frame, cv.COLOR_BGR2GRAY)
flow = cv.calcOpticalFlowFarneback(prev_gray, gray,
                                   None,
                                   0.5, 3, 50, 3, 5, 1.2, 0)

# Computes the magnitude and angle of the 2D vectors
magnitude, angle = cv.cartToPolar(flow[..., 0], flow[..., 1])

# Sets image hue according to the optical flow
# direction
mask[..., 0] = angle * 180 / np.pi / 2

# Sets image value according to the optical flow
# magnitude (normalized)
mask[..., 2] = cv.normalize(magnitude, None, 0, 255, cv.NORM_MINMAX)

# Converts HSV to RGB (BGR) color representation
rgb = cv.cvtColor(mask, cv.COLOR_HSV2BGR)

# Opens a new window and displays the output frame
cv.imshow("frame", rgb)

# Updates previous frame
prev_gray = gray

# Frames are read by intervals of 1 millisecond. The
# programs breaks out of the while loop when the
# user presses the 'q' key
cv.waitKey(-1)

prev_gray = cv.cvtColor(old_frame, cv.COLOR_BGR2GRAY)
flow = cv.calcOpticalFlowFarneback(prev_gray, gray,
                                   None,
                                   0.5, 3, 50, 3, 5, 1.2, 0)

distXMap = np.arange(-width/2, width/2)
distYMap = np.arange(-height/2, height/2)

stuff = np.array(np.meshgrid(distYMap, distXMap)).T.reshape(flow.shape)

alphaY = stuff[..., 0] / focalLength
alphaX = stuff[..., 1] / focalLength

betaY = (stuff[..., 0] + flow[..., 0]) / focalLength
betaX = (stuff[..., 1] + flow[..., 1]) / focalLength

zCoord1 = dist * alphaX / (betaX - alphaX)
zCoord2 = dist * alphaY / (betaY - alphaY)
avg = (zCoord1 + zCoord2) / 2


avg = avg / 1000 * 255
avg = np.clip(avg, 0, 255)

cv.imshow('frame', zCoord1)
# cv.imshow('frame', avg)

cv.waitKey(-1)

cv.destroyAllWindows()