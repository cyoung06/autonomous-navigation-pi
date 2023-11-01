import numpy
import numpy as np
import cv2



# cap = cv2.VideoCapture(0)
# cap.read()
# cap.read()
# ret, img = cap.read()
# cv2.imwrite('lol.png', img)
img = cv2.imread("lol.png")

blur_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
cv2.imshow("lol", blur_gray)
cv2.waitKey(-1)
kernel_size = 15
blur_gray = cv2.medianBlur(blur_gray,kernel_size, 0)
cv2.imshow("lol", blur_gray)
cv2.waitKey(-1)
# cv2.threshold(cv2.THRESH_OTSU)
blur_gray = cv2.adaptiveThreshold(blur_gray, 255,cv2.ADAPTIVE_THRESH_MEAN_C,\
            cv2.THRESH_BINARY_INV,17,2)
cv2.imshow("lol", blur_gray)
cv2.waitKey(-1)

#
# blur_gray = cv2.morphologyEx(blur_gray, cv2.MORPH_CLOSE, kernel,iterations=1)


# find all of the connected components (white blobs in your image).
# im_with_separated_blobs is an image where each detected blob has a different pixel value ranging from 1 to nb_blobs - 1.
nb_blobs, im_with_separated_blobs, stats, _ = cv2.connectedComponentsWithStats(blur_gray)
# stats (and the silenced output centroids) gives some information about the blobs. See the docs for more information.
# here, we're interested only in the size of the blobs, contained in the last column of stats.
sizes = stats[:, -1]
# the following lines result in taking out the background which is also considered a component, which I find for most applications to not be the expected output.
# you may also keep the results as they are by commenting out the following lines. You'll have to update the ranges in the for loop below.
sizes = sizes[1:]
nb_blobs -= 1

# minimum size of particles we want to keep (number of pixels).
# here, it's a fixed value, but you can set it as you want, eg the mean of the sizes or whatever.
min_size = 900

# output image with only the kept components
im_result = np.zeros_like(im_with_separated_blobs).astype(numpy.uint8)
# for every component in the image, keep it only if it's above min_size
for blob in range(nb_blobs):
    if sizes[blob] >= min_size:
        # see description of im_with_separated_blobs above
        im_result[im_with_separated_blobs == blob + 1] = 255

blur_gray = im_result
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1,2))
blur_gray = cv2.morphologyEx(blur_gray, cv2.MORPH_OPEN, kernel, iterations=3)

cv2.imshow("lol", blur_gray)
cv2.waitKey(-1)

# blur_gray = cv2.GaussianBlur(blur_gray, (kernel_size, kernel_size), 0)
# cv2.imshow("lol", blur_gray)
# cv2.waitKey(-1)

# low_threshold = 20
# high_threshold = 20
# edges = cv2.Canny(blur_gray, low_threshold, high_threshold)


# edges = edges1 + edges2

# cv2.imshow("lol", edges)
# cv2.waitKey(-1)

rho = 1  # distance resolution in pixels of the Hough grid
theta = np.pi / 180  # angular resolution in radians of the Hough grid
threshold = 7  # minimum number of votes (intersections in Hough grid cell)
min_line_length = 300  # minimum number of pixels making up a line
max_line_gap = 15  # maximum gap in pixels between connectable line segments
line_image = np.copy(img) * 0  # creating a blank to draw lines on

# Run Hough on edge detected image
# Output "lines" is an array containing endpoints of detected line segments
lines = cv2.HoughLinesP(blur_gray, rho, theta, threshold, np.array([]),
                    min_line_length, max_line_gap)

for line in lines:
    for x1,y1,x2,y2 in line:
        cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0), 1)


lines_edges = cv2.addWeighted(img, 0.8, line_image, 1, 0)

cv2.imshow("lol", lines_edges)
cv2.waitKey(-1)