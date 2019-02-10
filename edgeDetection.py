import cv2
import numpy as np
import math 
def objectTrack(frame): #mask the frame to get the blue object
    frame = cv2.GaussianBlur(frame, (5,5), 0)
    
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
    lower_blue = np.array([110,50,50])
    upper_blue = np.array([130,255,255])
    #lower_blue = np.array([0,100,100])
    #upper_blue = np.array([20,255,255])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame, frame, mask = mask)
    
    return res
    
def findBigContour(cnts):
    index = 0
    length = 0
    for i in range(len(cnts)):
        perimeter = cv2.arcLength(cnts[i],True)
        if length < perimeter:
            index = i
            length  = perimeter
    return index
    
        
def findPixelLocation(res): ## return the pixel location of the obj
    #image to be canny edge detected must be masked 
    image = res
    canny = cv2.Canny(image, 30, 150)

    #find contour
    (_, cnts, _) = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    #find the longest contour
    print('I count {} contours in this image'.format(len(cnts)))
    index = findBigContour(cnts) # find the index of the contour with biggest perimeter
    print('index = ', index) #index = 6
    
    #draw the longest contour
    img = cv2.drawContours(image, cnts, index, (0,255,0), 3)
    #cv2.imwrite('contour.jpg', img)
    
    cv2.imshow('dst',img)
    if cv2.waitKey(0) & 0xff == 27:
        cv2.destroyAllWindows()
    
    #find center
    M = cv2.moments(cnts[index])
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    
    return (cx, cy)
    
def distanceToOriginal(p): # not to the original, but to the original of the bottom
    px = p[0] - 0
    py = p[1] - (-8)
    return math.sqrt(px**2 + py**2)
    
def toObjLocation(center): ## return the location of the obj
    cx = center[0]
    cy = center[1]
    
    ## (0, 13) + ((cx-320)*(1/18), (cy-240)*(1/18))
    rx = -(0 + (cx-320)*(1/18.33))
    ry = 13 + (cy-240)*(1/18.33)
    return (rx, ry)
    
if __name__ == '__main__':

    cap = cv2.VideoCapture(0)
    _, frame = cap.read()
    _, frame = cap.read()
    _, frame = cap.read()
    cv2.imwrite('frame.jpg', frame)
    #input("wait...")
    res = objectTrack(frame)
    
    center = findPixelLocation(res)
    print('cx:%d'% center[0])
    print('cy:%d'% center[1])
    
    r = toObjLocation(center)
    print('r:',r)
    
    dis = distanceToOriginal(r)
    print('distance:%s'% str(dis))
    
