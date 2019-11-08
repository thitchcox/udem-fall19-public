import cv2
import numpy as np
import rospy

## Software Exercise 6: Choose your category (1 or 2) and replace the cv2 code by your own!

## CATEGORY 1
def inRange(hsv_image, low_range, high_range):
	h = hsv_image[:, :, 0]
	s = hsv_image[:, :, 1]
	v = hsv_image[:, :, 2]

	in_h_bounds = (h >= low_range[0]) & (h <= high_range[0])
	in_s_bounds = (s >= low_range[1]) & (s <= high_range[1])
	in_v_bounds = (v >= low_range[2]) & (v <= high_range[2])

	mask = (in_h_bounds) & (in_s_bounds) & (in_v_bounds)

	return mask.astype('uint8')
	# return cv2.inRange(hsv_image, low_range, high_range)

def bitwise_or(bitwise1, bitwise2):	
	return np.bitwise_or(bitwise1, bitwise2)	
	# return cv2.bitwise_or(bitwise1, bitwise2)

def bitwise_and(bitwise1, bitwise2):
	return np.bitwise_and(bitwise1, bitwise2)
	# return cv2.bitwise_and(bitwise1, bitwise2)

def getStructuringElement(shape, size):
	struct_el = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]])
	return struct_el.astype('uint8')
	# return cv2.getStructuringElement(shape, size)

def dilate(bitwise, kernel):
	# Get shape of objects
	u, v = kernel.shape
	y, x = bitwise.shape
	# Start at the right image indices for convolution.
	y_prime = y - u + 1
	x_prime = x - v + 1
	# Make a new image to return
	dilated_img = np.zeros((y,x))
	# Loop thru and perform convolution
	for i in range(y_prime):
		for j in range(x_prime):
			dilated_img[i][j] = np.sum(bitwise[i : i + u, j : j + v] * kernel)	
	return dilated_img.astype('uint8')
	# return cv2.dilate(bitwise, kernel)


## CATEGORY 2
def Canny(image, threshold1, threshold2, apertureSize=3):
	return cv2.Canny(image, threshold1, threshold2, apertureSize=3)


## CATEGORY 3 (This is a bonus!)
def HoughLinesP(image, rho, theta, threshold, lines, minLineLength, maxLineGap):
	return cv2.HoughLinesP(image, rho, theta, threshold, lines, minLineLength, maxLineGap)