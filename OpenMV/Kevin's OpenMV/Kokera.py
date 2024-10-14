import cv2 as cv
import numpy as np

MAP_RES = 80

LOWER_HUE = 0
LOWER_SATURATION = 0
LOWER_VALUE = 0
UPPER_HUE = 255
UPPER_SATURATION = 100
UPPER_VALUE = 280
BLUR = 9
BLUR_ITERATIONS = 3
REGION_OF_DISINTEREST_OFFSET = 130

def getBlur():
    blur = BLUR
    blur = max(1, blur)
    return (blur, blur)

def regionOfDisinterest(img, vertices):
    mask = np.ones_like(img) * 255
    cv.fillPoly(mask, vertices, 0)
    masked_image = cv.bitwise_and(img, mask)
    return masked_image

def flattenImage(img):
    top_left = (int)(img.shape[1] * 0.33), (int)(img.shape[0])
    top_right = (int)(img.shape[1] - img.shape[1] * 0.388), (int)(img.shape[0])
    bottom_left = 0, 0
    bottom_right = (int)(img.shape[1]), 0

    src_pts = np.float32([[top_left], [top_right], [bottom_left], [bottom_right]])
    dest_pts = np.float32([ [0, 1080], [1920, 1080] ,[0, 0], [1920, 0]])

    matrix = cv.getPerspectiveTransform(dest_pts, src_pts)
    output = cv.warpPerspective(img, matrix, (1920, 1080))
    return output


# cv.imshow("Display window", img)
# k = cv.waitKey(0) # Wait for a keystroke in the window

# Blur it up
for _ in range(BLUR_ITERATIONS):
    cv_image = cv.blur(cv_image, getBlur())

# Apply filter and return a mask
img = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)



lower = (
    LOWER_HUE,
    LOWER_SATURATION,
    LOWER_VALUE
)
upper = (
    UPPER_HUE,
    UPPER_SATURATION,
    UPPER_VALUE
)
mask = cv.inRange(img, lower, upper)
mask = 255 - mask

# Apply region of disinterest and flattening
height = img.shape[0]
width = img.shape[1]
region_of_disinterest_vertices=[
    (0, height),
    (width / 2 + REGION_OF_DISINTEREST_OFFSET, height / 2 + REGION_OF_DISINTEREST_OFFSET),
    (width, height)
]
# mask = regionOfDisinterest(mask, np.array([region_of_disinterest_vertices], np.int32))
mask[mask < 250] = 0

mask = flattenImage(mask)

preview_image = cv.cvtColor(mask, cv.COLOR_GRAY2RGB)


# cv.polylines(preview_image, np.array([region_of_disinterest_vertices], np.int32), True, (0, 255, 0), 2)
datamap = cv.resize(preview_image, dsize=(MAP_RES, MAP_RES), interpolation=0) / 2
cv.imshow("datamap window", datamap)
cv.imshow("preview", preview_image)
k = cv.waitKey(0)
