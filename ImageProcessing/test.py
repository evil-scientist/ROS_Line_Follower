import numpy as np
import cv2

def RemoveBackground(image):
	up = 100
	# create NumPy arrays from the boundaries
	lower = np.array([0, 0, 0], dtype = "uint8")
	upper = np.array([up, up, up], dtype = "uint8")
	#----------------COLOR SELECTION-------------- (Remove any area that is whiter than 'upper')
	mask = cv2.inRange(image, lower, upper)
	image = cv2.bitwise_and(image, image, mask = mask)
	image = cv2.bitwise_not(image, image, mask = mask)
	image = (255-image)
	return image

# starting video streaming
cv2.namedWindow('window_frame')
video_capture = cv2.VideoCapture(-1)

while True:
	img = video_capture.read()[1]
	img = RemoveBackground(img)
	imgray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) #Convert to Gray Scale

	ret, thresh = cv2.threshold(imgray,100,255,cv2.THRESH_BINARY_INV) #Get Threshold
	
	contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) #Get contour
	cv2.drawContours(img, contours, -1, (0,255,0), 3)

	cv2.imshow("window_frame", img)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# Clean up the connection
cv2.destroyAllWindows()