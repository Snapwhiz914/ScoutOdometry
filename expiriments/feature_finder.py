import cv2
import numpy as np

def find_intersection_points(contours):
    print(contours)
    # Create an empty image to draw contours
    img_contours = np.zeros_like(img)

    # Draw contours on the empty image
    cv2.drawContours(img_contours, contours, -1, (255, 0, 0), 5)
    cv2.imshow('Empty image', img_contours)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Find intersection points
    intersection_points = []
    for i in range(len(contours)):
        for j in range(i+1, len(contours)):
            intersection = find_contour_intersection(contours[i], contours[j])
            if intersection is not None:
                intersection_points.append(intersection)

    return intersection_points

def find_contour_intersection(contour1, contour2):
    # Find the intersection between two contours
    result = cv2.matchShapes(contour1, contour2, cv2.CONTOURS_MATCH_I1, 0.0)
    if result < 0.05:  # You may need to adjust this threshold based on your specific use case
        print(contour1)
        print(contour2)
        intersection = find_approx_intersection(contour1, contour2)
        return intersection
    return None

def find_approx_intersection(contour1, contour2):
    # Find the intersection point by approximating the contours
    epsilon = 0.02 * cv2.arcLength(contour1, True)
    approx_contour1 = cv2.approxPolyDP(contour1, epsilon, True)

    epsilon = 0.02 * cv2.arcLength(contour2, True)
    approx_contour2 = cv2.approxPolyDP(contour2, epsilon, True)

    # Find the common area between the approximated contours
    intersection = cv2.bitwise_and(approx_contour1, approx_contour2)
    if np.count_nonzero(intersection) > 0:
        # Find the centroid of the common area
        M = cv2.moments(intersection)
        print(M)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return (cx, cy)
    return None

# Read the image
img = cv2.imread('/Users/liamloughead/Downloads/IMG_0908.JPG')

# Convert the image to grayscale
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (7, 7), 0)
cv2.imshow('Gray', blurred)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Threshold the image
thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 7, 2)
#_, thresh = cv2.threshold(blurred, 128, 255, cv2.THRESH_BINARY)
cv2.imshow('Thresh', thresh)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Find contours in the binary image
contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Find intersection points of contours
intersection_points = find_intersection_points(contours)

# Draw intersection points on the original image
for point in intersection_points:
    cv2.circle(img, point, 5, (0, 0, 255), -1)

# Display the result
cv2.imshow('Intersection Points', img)
cv2.waitKey(0)
cv2.destroyAllWindows()