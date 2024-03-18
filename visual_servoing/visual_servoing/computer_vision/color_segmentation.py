import cv2
import numpy as np

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def cd_color_segmentation(img, template):
	"""
	Implement the cone detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
	"""
	########## YOUR CODE STARTS HERE ##########
	min_area = 225

	# Convert BGR to HSV
	hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) ## assume H is [0, 180]
	grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	# blur_gs = cv2.blur(hsv_img, (3,3))

	# Blur image before masking
	blur = cv2.GaussianBlur(hsv_img, (3,3), 0) #change size of kernel as needed

	# Mask to keep what we want
	## Orange HSV, tighten ranges after gaussian blur
	orange_lower = np.array([10, 90, 130]) # best so far 10, 90, 130
	orange_upper = np.array([25, 255, 255]) # best for far 25, 255, 255
	# gauss_lower = np.array([10, 90, 140])

	## Dark Orange HSV
	dark_or_lower = np.array([2, 226, 107])
	dark_or_upper = np.array([13, 255, 201]) # best: 13, 250, 163

	## Gray Mask
	# gray_lower = np.array([0, 0, 50]) # best so far 10, 90, 140
	# gray_upper = np.array([0, 0, 70])
	# gray_mask = cv2.inRange(grayscale, gray_lower, gray_upper)
	gray_blur = cv2.GaussianBlur(grayscale, (5,5), 0)
	ret, gray_mask = cv2.threshold(gray_blur, 50, 255, 0)

	# Implement brown mask
	brown_lower = np.array([18, 100, 138]) # 20, 100, 138
	brown_upper = np.array([30, 222, 165]) # 30, 145/222, 153/172
	brown_mask = cv2.inRange(blur, brown_lower, brown_upper)
	# kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
	# opening = cv2.morphologyEx(brown_mask, cv2.MORPH_OPEN, kernel)
	
	# 3-part Mask
	# mask1 = cv2.inRange(hsv_img, orange_lower, brown_lower)
	# mask2 = cv2.inRange(hsv_img, brown_upper, orange_upper)
	# full_mask = mask1 | mask2 | dark_mask
	
	mask = cv2.inRange(blur, orange_lower, orange_upper)
	dark_mask = cv2.inRange(blur, dark_or_lower, dark_or_upper)
	full_mask = mask | dark_mask
	
	# Intersect full_mask + gray_mask
	# gray_and_bgr = cv2.bitwise_and(full_mask, gray_mask)
	mod_mask = full_mask - brown_mask
	# mod_blur = cv2.GaussianBlur(mod_mask, (3,3), 0)
	# kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
	# opening = cv2.morphologyEx(mod_mask, cv2.MORPH_OPEN, kernel)
	
	# Erode + dilate
	# kernel = np.ones((3,3), np.uint8)
	# img_erode = cv2.erode(full_mask, kernel, iterations = 1)
	# img_dilate = cv2.dilate(img_erode, kernel, iterations = 1)

	# Find contour, bound rectangle
	# contours, hierarchy = cv2.findContours(gray_and_bgr, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
	contours, hierarchy = cv2.findContours(mod_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	print("Number of contours detected:",len(contours))
	max = 0
	cnt = contours[0]
	for c in contours:
		x,y,w,h = cv2.boundingRect(c)
		# if w*h > max:
		# 	max = w*h
		# 	cnt = c
		area = cv2.contourArea(c)
		if area >= max and w/h < 0.9:
			max = area
			cnt = c


	x,y,w,h = cv2.boundingRect(cnt)
	epsilon = 0.01*cv2.arcLength(cnt, True)
	approx = cv2.approxPolyDP(cnt, epsilon, True)
	# x,y,w,h = cv2.boundingRect(approx)
	bounding_box = ((x, y), (x+w, y+h))
	########### YOUR CODE ENDS HERE ###########

	# Draw rectangle
	# img = cv2.rectangle(img, (x,y), (x+w, y+h), (0, 255, 0), 2)
	# gray_and_bgr = cv2.rectangle(gray_and_bgr, (x,y), (x+w, y+h), (0, 255, 0), 2)
	# pmtr = 0.01*cv2.arcLength(cnt, True)
	# approx = cv2.approxPolyDP(cnt, pmtr, True)
	
	res = cv2.cvtColor(mod_mask, cv2.COLOR_GRAY2BGR)
	# res2 = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
	# res2 = cv2.rectangle(res2, (x,y), (x+w, y+h), (0, 255, 0), 2)
	cv2.drawContours(img, [cnt], 0, (0, 255, 0), 3)
	# cv2.drawContours(blur, [cnt], 0, (0, 255, 0), 3)
	# cv2.drawContours(res, [approx], 0, (0,0,255), 3)
	# cv2.imwrite(f'testing/6/mod_mask/{w}x{h}_mod_mask.jpg', img)
	# cv2.imwrite(f'testing/6/mod_mask/noblur/{w}x{h}_mod_mask_cnt.jpg', res)
	# cv2.imwrite(f'testing/6/gauss_blur/{w}x{h}_gauss_blur.jpg', blur)
	# res2 = cv2.cvtColor(gray_mask, cv2.COLOR_GRAY2BGR)
	# cv2.drawContours(res2, [cnt], 0, (0, 255, 0), 3)
	# cv2.imwrite(f'testing/6/gray_mask/gblur5/{w}x{h}_gray_mask_blur5.jpg', res2)
	# Return bounding box
	return bounding_box
