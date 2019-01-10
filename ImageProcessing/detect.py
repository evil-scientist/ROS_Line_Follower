import numpy as np
import cv2

Images=[]
N_SLICES = 4
direction = [0,0,0,0]

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

def SlicePart(im, images, slices):
	height, width = im.shape[:2]
	sl = int(height/slices);
	
	for i in range(slices):
		part = sl*i
		crop_img = im[part:part+sl, 0:width]
		# images[i].image = crop_img
		Process(crop_img,i)

def getContourCenter(contour):
		M = cv2.moments(contour)
		
		if M["m00"] == 0:
			return 0
		
		x = int(M["m10"]/M["m00"])
		y = int(M["m01"]/M["m00"])
		
		return [x,y]

def getContourExtent(contour):
		area = cv2.contourArea(contour)
		x,y,w,h = cv2.boundingRect(contour)
		rect_area = w*h
		if rect_area > 0:
			return (float(area)/rect_area)
			
def Aprox(a, b, error):
	if abs(a - b) < error:
		return True
	else:
		return False
				
def Process(img,pos):
	
	contourCenterX = 0
	MainContour = None

	imgray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) #Convert to Gray Scale
	ret, thresh = cv2.threshold(imgray,100,255,cv2.THRESH_BINARY_INV) #Get Threshold
	
	contours,hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) #Get contour
	
	prev_MC = MainContour
	if contours:
		MainContour = max(contours, key=cv2.contourArea)
		
		height, width  = img.shape[:2]

		middleX = int(width/2) #Get X coordenate of the middle point
		middleY = int(height/2) #Get Y coordenate of the middle point
			
		prev_cX = contourCenterX
		if getContourCenter(MainContour) != 0:
			contourCenterX = getContourCenter(MainContour)[0]
			if abs(prev_cX-contourCenterX) > 5:
				if abs(prev_cX-contourCenterX) > 5:
					for i in range(len(contours)):
						if getContourCenter(contours[i]) != 0:
							tmp_cx = getContourCenter(contours[i])[0]
							if Aprox(tmp_cx, prev_cX, 5) == True:
								MainContour = contours[i]
								if getContourCenter(MainContour) != 0:
									contourCenterX = getContourCenter(MainContour)[0]

		else:
			contourCenterX = 0
			
		direction1 =  int((middleX-contourCenterX) * getContourExtent(MainContour))
		direction[pos] = direction1

		cv2.drawContours(img,MainContour,-1,(0,255,0),3) #Draw Contour GREEN
		cv2.circle(img, (contourCenterX, middleY), 7, (255,255,255), -1) #Draw dX circle WHITE
		cv2.circle(img, (middleX, middleY), 3, (0,0,255), -1) #Draw middle circle RED
		
		font = cv2.FONT_HERSHEY_SIMPLEX
		cv2.putText(img,str(middleX-contourCenterX),(contourCenterX+20, middleY), font, 1,(200,0,200),2,cv2.LINE_AA)
		cv2.putText(img,"Weight:%.3f"%getContourExtent(MainContour),(contourCenterX+20, middleY+35), font, 0.5,(200,0,200),1,cv2.LINE_AA)						
							
# starting video streaming
cv2.namedWindow('window_frame')
video_capture = cv2.VideoCapture(-1)

while True:
	img = video_capture.read()[1]

	img = RemoveBackground(img)
	final_direction = 0
	SlicePart(img, Images, N_SLICES)
	for i in range(len(direction)):
		final_direction += direction[i]
	# Process(img)						
	print(final_direction)
	cv2.imshow("window_frame", img)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# Clean up the connection
cv2.destroyAllWindows()