import cv2
import numpy as np
import math

# image = cv2.imread("4.jpg")

def get_contour(image):
	threshold=130
	
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	blurred = cv2.GaussianBlur(gray, (11, 11), 0)


	canny = cv2.Canny(blurred, 20, 160)
	#canny_dilate = cv2.dilate(canny, None, iterations=1)
	#canny_erode = cv2.erode(canny_dilate, None, iterations=1)

	(_, cnts, _) = cv2.findContours(canny.copy(), cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)

	contours = image.copy()
	cv2.drawContours(contours, cnts, -1, (0, 255, 0), 2)
	centerX = []
	centerY = []
	area = []
	count = 0
	# loop over the contours individually
	centroid = image.copy()
	for c in cnts:
		breakcount=0
		# Area
		# print(cv2.contourArea(c))
		# perimeter
		# print(cv2.arcLength(c,True))

		# centroid from moments
		M = cv2.moments(c)
		if M["m00"] != 0:
			cx = int(M["m10"]/M["m00"])
			cy = int(M["m01"]/M["m00"])
		else:
			cx, cy = 0, 0
		if(abs(0-cx)<threshold):
			breakcount=1
		elif(abs(0-cy)<threshold):
			breakcount=1
		elif(abs(960-cx)<threshold):
			breakcount=1
		elif(abs(720-cy)<threshold):
			breakcount=1
		else:
			for i in range(0,count):
				if(math.sqrt((centerX[i]-cx)**2+(centerY[i]-cy)**2)<threshold):
					breakcount=1
					if(cv2.contourArea(c)>area[i]):
						centerX[i]=cx
						centerY[i]=cy
						area[i]=cv2.contourArea(c)
					break
		if(breakcount==0):
			centerX.append(cx)
			centerY.append(cy)
			area.append(cv2.contourArea(c))
			count=count+1
		cv2.rectangle(centroid, (cx-112, cy-112), (cx + 112, cy + 112), (0, 255, 0), 2)
		cv2.circle(centroid, (cx, cy), 5, (0, 0, 255), -1)
	# print(centerX)
	# print(centerY)
	for i in range(0,count):
		cv2.rectangle(centroid, (centerX[i]-112, centerY[i]-112), (centerX[i] + 112, centerY[i] + 112), (255, 0, 0), 2)
		x=centerX[i]-112
		y=centerY[i]-112
		h=224
		w=224
		crop_img = image[y:y+h, x:x+w]
		cv2.imwrite("crop_outcome/"+str(i)+"_new.jpg", crop_img)
	result = np.hstack([contours, centroid])
	return result,centerX,centerY
# cv2.imwrite("result.jpg", result)