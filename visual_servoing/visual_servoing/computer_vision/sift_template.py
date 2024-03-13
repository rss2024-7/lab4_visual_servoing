import cv2
import imutils
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

def image_print(img, bbox1=None, bbox2=None, winname="Image", destroy=True):
	"""
	Helper function to print out images and a bounding box, for debugging.
	Press any key to continue.

	bbox1 and bbox2 should be 2-item tuples. (x1, y1) and (x2, y2)
	"""
	cv2.namedWindow(winname)        # Create a named window
	cv2.moveWindow(winname, 40,30)  # Move it to (40,30)
	if bbox1 is not None:
		cv2.rectangle(img, bbox1, bbox2, (0,0,255), 1)
	cv2.imshow(winname, img)
	cv2.waitKey()
	if destroy:
		cv2.destroyAllWindows()

def cd_sift_ransac(img, template, debug=False):
	"""
	Implement the cone detection using SIFT + RANSAC algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the bottom left of the bbox and (x2, y2) is the top right of the bbox
	"""
	# Minimum number of matching features
	MIN_MATCH = 10 # Adjust this value as needed
	# Create SIFT
	sift = cv2.xfeatures2d.SIFT_create()

	# Compute SIFT on template and test image
	kp1, des1 = sift.detectAndCompute(template,None)
	kp2, des2 = sift.detectAndCompute(img,None)

	# Find matches
	bf = cv2.BFMatcher()
	matches = bf.knnMatch(des1,des2,k=2)

	# Find and store good matches
	good = []
	for m,n in matches:
		if m.distance < 0.75*n.distance:
			good.append(m)

	# If enough good matches, find bounding box
	if len(good) > MIN_MATCH:
		src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
		dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

		# M is homography matrix transforming from template to image
		M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
		matchesMask = mask.ravel().tolist()

		h, w, _ = template.shape

		# Location of bbox pts in template image
		pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)

		print(M)

		########## YOUR CODE STARTS HERE ##########
		transformed_pts = cv2.perspectiveTransform(pts, M)

		transformed_pts = transformed_pts.reshape(-1, 2)

		x_min = int(transformed_pts[:,0].min())
		y_min = int(transformed_pts[:,1].min())
		x_max = int(transformed_pts[:,0].max())
		y_max = int(transformed_pts[:,1].max())

		if debug:
			image_print(img, bbox1=(x_min, y_min), bbox2=(x_max, y_max))

		########### YOUR CODE ENDS HERE ###########

		# Return bounding box
		return ((x_min, y_min), (x_max, y_max))
	else:

		print(f"[SIFT] not enough matches; matches: {len(good)}")

		# Return bounding box of area 0 if no match found
		return ((0,0), (0,0))

def cd_template_matching(img, template, debug=False):
	"""
	Implement the cone detection using template matching algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the bottom left of the bbox and (x2, y2) is the top right of the bbox
	"""
	template_canny = cv2.Canny(template, 50, 200)

	# Perform Canny Edge detection on test image
	grey_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	img_canny = cv2.Canny(grey_img, 50, 200)

	# Get dimensions of template
	(img_height, img_width) = img_canny.shape[:2]

	# Keep track of best-fit match
	best_match = None

	# Loop over different scales of image
	for scale in np.linspace(1.5, .5, 500):
		# Resize the image
		resized_template = imutils.resize(template_canny, width = int(template_canny.shape[1]))
		h, w = resized_template.shape[:2]
		# Check to see if test image is now smaller than template image
		if resized_template.shape[0] > img_height or resized_template.shape[1] > img_width:
			continue

		########## YOUR CODE STARTS HERE ##########
		# Use OpenCV template matching functions to find the best match
		# across template scales.

		# Returns greyscale img denoting how much this region matches with template
		result = cv2.matchTemplate(img_canny, resized_template, cv2.TM_CCOEFF)
		
		# Scan result of scan match to find location with highest match
		_, maxVal, _, maxLoc = cv2.minMaxLoc(result)

        # Update the best match if the new match is better
		if best_match is None or maxVal > best_match[0]:
			best_match = (maxVal, maxLoc, scale)

	# Unpack the best match
	_, maxLoc, scale = best_match
	startX, startY = maxLoc

	# Remember to resize the bounding box using the highest scoring scale
	# x1,y1 pixel will be accurate, but x2,y2 needs to be correctly scaled
	endX = startX + int(w)
	endY = startY + int(h)

	bbox = ((startX, startY), (endX, endY))

	if debug:
		image_print(template, winname="Template", destroy=False)
		image_print(img, bbox1=bbox[0], bbox2=bbox[1])
	########### YOUR CODE ENDS HERE ###########

	return bbox

# pic_num = np.random.randint(1, 15)
# cd_sift_ransac(cv2.imread(f"test_images_citgo/citgo{pic_num}.jpeg"), cv2.imread("test_images_citgo/citgo_template.png"), debug=True)

pic_num = np.random.randint(1, 10)
cd_template_matching(cv2.imread("test_images_localization/basement_fixed.png"), cv2.imread(f"test_images_localization/map_scrap{pic_num}.png"), debug=True)