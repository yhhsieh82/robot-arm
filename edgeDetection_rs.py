import cv2
import numpy as np
import math 
import pyrealsense2 as rs

# Initialize the parameters
rotate_angle = 30.5
round_value = 4

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

def rotate_coordinate(pos, theta):
    x1=pos[0]
    y1=math.cos(math.radians(theta))*pos[1]+math.sin(math.radians(theta))*pos[2]
    z1=-math.sin(math.radians(theta))*pos[1]+math.cos(math.radians(theta))*pos[2]
    return np.asarray([round(x1, round_value), round(y1, round_value), round(z1, round_value)])
    
def get_deprojection_position(obj_name='bottle'):
    # Create a pipeline
    pipeline = rs.pipeline()

    #  Create a config and configure the pipeline to stream
    #  different resolutions of color and depth streams
    config = rs.config()
    config.disable_stream(rs.stream.color)
    # Start streaming
    profile = pipeline.start(config)

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)
    depth_point = [0,0,0]
    # Streaming loop
    try:
        while True:
            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()
            # frames.get_depth_frame() is a 640x360 depth image
            
            # Align the depth frame to color frame
            aligned_frames = align.process(frames)
            
            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()
            
            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            
            # objs = get_object(color_image, obj_name)
            objs = []
            res = objectTrack(frame)
            objs = objs.append(findPixelLocation(res))
            
            if objs:
                obj_depth = aligned_depth_frame.get_distance(objs[-1][0], objs[-1][1])
                print('%s pixel: (%d, %d) depth: %f' %(obj_name, objs[-1][0], objs[-1][1], obj_depth))

                depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
                # depth_pixel = [depth_intrin.ppx, depth_intrin.ppy]
                depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, objs[-1], obj_depth)
                # print('xyz:', depth_point)
                print('rotate:', rotate_coordinate(depth_point, rotate_angle))

            cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('Align Example', color_image)
            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        pipeline.stop()

    return rotate_coordinate(depth_point, rotate_angle)

if __name__ == '__main__':

    cap = cv2.VideoCapture(0)
    _, frame = cap.read()
    _, frame = cap.read()
    _, frame = cap.read()
    cv2.imwrite('frame.jpg', frame)
    #input("wait...")
    #res = objectTrack(frame)
    
    #center = findPixelLocation(res)
    #print('cx:%d'% center[0])
    #print('cy:%d'% center[1])
    
    get_deprojection_position(obj_name='bottle')
    #r = toObjLocation(center)
    #print('r:',r)
    
    #dis = distanceToOriginal(r)
    #print('distance:%s'% str(dis))