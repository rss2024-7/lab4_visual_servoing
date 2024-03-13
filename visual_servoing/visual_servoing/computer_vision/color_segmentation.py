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
	
	# Blur image before masking
	blur = cv2.blur(hsv_img, (5,5)) #change size of kernel as needed

	# Mask to keep what we want
	## Orange HSV
	orange_lower = np.array([10, 90, 140]) # best so far 10, 90, 140
	orange_upper = np.array([25, 255, 255]) # best for far 25, 255, 255
	# implement dark orange mask 
	dark_or_lower = np.array([2, 226, 107])
	dark_or_upper = np.array([13, 250, 163])

	mask = cv2.inRange(blur, orange_lower, orange_upper)
	dark_mask = cv2.inRange(blur, dark_or_lower, dark_or_upper)
	full_mask = mask | dark_mask

	# Erode + dilate
	kernel = np.ones((3,3), np.uint8)
	# img_erode = cv2.erode(full_mask, kernel, iterations = 1)
	# img_dilate = cv2.dilate(img_erode, kernel, iterations = 1)

	# Find contour, bound rectangle
	contours, hierarchy = cv2.findContours(full_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
	# cnt = contours[0]
	# cnt = np.where()
	max = 0
	cnt = contours[0]
	for c in contours:
		x,y,w,h = cv2.boundingRect(c)
		# if w*h > max:
		# 	max = w*h
		# 	cnt = c
		area = cv2.contourArea(c)
		if area >= max and w < h:
			max = area
			cnt = c

	x,y,w,h = cv2.boundingRect(cnt)
	bounding_box = ((x, y), (x+w, y+h))
	########### YOUR CODE ENDS HERE ###########

	# Draw rectangle
	img = cv2.rectangle(img, (x,y), (x+w, y+h), (0, 255, 0), 2)
	cv2.drawContours(img, [cnt], 0, (0, 255, 0), 3)
	# cv2.imwrite(f'testing/6/{w}x{h}_contours.jpg', img)

	# Return bounding box
	return bounding_box
