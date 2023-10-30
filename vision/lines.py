

import numpy as np
import cv2



cap = cv2.VideoCapture(0)
cap.read()
cap.read()
ret, img = cap.read()


gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
kernel_size = 15
blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size),0)
cv2.imshow("lol", blur_gray)
cv2.waitKey(-1)

low_threshold = 0
high_threshold = 15
edges = cv2.Canny(blur_gray, low_threshold, high_threshold)

cv2.imshow("lol", edges)
cv2.waitKey(-1)

rho = 1  # distance resolution in pixels of the Hough grid
theta = np.pi / 180  # angular resolution in radians of the Hough grid
threshold = 15  # minimum number of votes (intersections in Hough grid cell)
min_line_length = 50  # minimum number of pixels making up a line
max_line_gap = 10  # maximum gap in pixels between connectable line segments
line_image = np.copy(img) * 0  # creating a blank to draw lines on

# Run Hough on edge detected image
# Output "lines" is an array containing endpoints of detected line segments
lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),
                    min_line_length, max_line_gap)

for line in lines:
    for x1,y1,x2,y2 in line:
        cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),5)


lines_edges = cv2.addWeighted(img, 0.8, line_image, 1, 0)

cv2.imshow("lol", lines_edges)
cv2.waitKey(-1)