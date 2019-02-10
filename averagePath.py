# cloning the behavior
# restrict to 1 DOF, that is the length between the object and the bottom of the robot
# step1. follow the path to the  
# 
#
import time
import serial
import pickle
import cv2
from image_processing import *
from test_webcam import *
from log_motor import *
from edgeDetection import *
import os
import math 
from crop3 import *
import numpy
import sys

MAX_SIMILARITY = 10
MIN_SIMILARITY = 0.6
TASK_DONE = False
COM="com16"

def distance(x1,y1,x2,y2):
    return math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))
def Ddistance(x1,y1,z1,x2,y2,z2):
    return math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)+(z1-z2)*(z1-z2))
    
def read_instructions(my_serial,path):
    with open(path, "rb") as f:
        lines = pickle.load(f)
        print('data num: '+ str(len(lines)))
        start_time = time.time()
        for line in lines :
            words=line['angles']
            temp=''
            for i in range(6):
                n=words[i]
                temp += ' ' + n
            s = temp.encode()
            while time.time()-start_time<line['time']:
                time.sleep(0.001)
            my_serial.write(s)
            print( str(line['time'])+' : ' + str(s))

def send_command(my_serial,angles):
    temp = ''
    for i in range(6):
        temp += ' ' + str(angles[i])
    s = temp.encode()
    my_serial.write(s)
    print('send : ' + temp)

def get_actions_by_frame(dir_path,img):
    threshold_sim = 0.95
    max_sim = 0
    max_index = None
    max_list = None
    found = False
    for filename in os.listdir(dir_path):
        path = os.path.join(dir_path,filename)
        with open(path, "rb") as f:
            data_list = pickle.load(f)
            print('use data : '+path)
            for i,data in enumerate(data_list):
                similarity = get_image_similarity(img,data['pic'])
                print('frame '+str(i)+': ')
                if similarity > threshold and similarity > max_sim:
                    found = True
                    max_sim = similarity
                    max_index = i
                    max_list = data_list
    if found:
        return max_list[i:]
    return None

def compare_angles(angles_1,angles_2):
    for i in range(len(angles_1)):
        if angles_1[i] != angles_2[i]:
            return False
    return True

def state_compare(cur_angles, path_angles):
    for i in range(len(cur_angles)):
        if cur_angles[i] != path_angles[i] :
            return False
    return True
    
## find the state which have the same state(angle) as current state
def find_same_state(path_list, cur_angles):
    same_states=[]  #list of the states which is same as the current state[i,j] ith path, jth image
    for i in range(len(path_list)):
        for j in range(len(path_list[i])):
            if state_compare(cur_angles, path_list[i][j]['angles']):
                same_state=[i,j]
                same_states.append(same_state)
    return same_states 

## find the state which have the greatest similarity with the current observation
def find_same_observation(img, same_states, path_list):
    path_num = 0    #index to path_list // 0=argv[1]
    frame_num = 0 
    similarity = 0    
    for k in range(len(same_states)):
        i = same_states[k][0]
        j = same_states[k][1]
        hash_similarity = image_hash_similarity(img, path_list[i][j]['pic'], 16, 16)
        
        if similarity < hash_similarity[0]:
            similarity = hash_similarity[0]
            path_num = i
            frame_num = j
            
        print('path: %d' %(i))
        print('pic similarity: %.2f' %(hash_similarity[0])) 
        show_image(np.hstack([make_regular_image(img),path_list[i][j]['pic']]))        
        input('enter to continue')
    same_state = [path_num, frame_num]
    return same_state

#path_list is a list of data
def many_path_behavior_clone(ser, path_list, threshold, delay_time):
    angles_match = False
  
    while not ser.in_waiting:
        pass
    cur_angles = read_angles(ser)
        
    ## find the state which have the same state(angle) as current state
    same_states = find_same_state(path_list, cur_angles)  #list of the states which is same as the current state[i,j] ith path, jth image
    
    ## find the state which have the greatest similarity with the current observation
    same_state = find_same_observation(img, same_states, path_list)
    path_num = same_state[0]    #index to path_list // 0=argv[1]
    frame_num = same_state[1]    

    ## state by state compare
    for j in range(frame_num, len(path_list[path_num])): 
		## sleep and wait the arm to be stable
        time.sleep(3)
        ret = False
        while not ret:
            ret, img = cap.read()
			
        ## compare the picture similarity
        hash_similarity = image_hash_similarity(img, path_list[path_num][j]['pic'], 16, 16)
        print('hash similarity : phash=%.6f , ahash=%.6f , dhash=%.6f , whash=%.6f'%(hash_similarity[0],hash_similarity[1],hash_similarity[2],hash_similarity[3]))
        similarity = hash_similarity[0]
        
        ## stop when the picture similarity < 0.9
        ## distinguish the wrong path and the bad similarity?
        if similarity < threshold:
            print('stop')
            # return
            if similarity > 0.8:
                input('enter to continue')
            else:
                print('the wrong path')
                break
            
        ## command the ser
        send_command(ser, path_list[path_num][j]['angles'])
        #time.sleep(delay_time)
        #input("wait")
        '''
        while ser.in_waiting:
            byte = ser.readline()
            string = byte.decode("utf-8")
            print('arduino -> '+string,end='')
		'''        
    print('end cloning')
    
def findBasicPath(path_list, r):
    path = 0
    length = 100
    for i in range(len(path_list)):
        if path_list[i][0] <= r :
            if r-path_list[i][0] < length:
                path = i
                length = r-path_list[i][0]
    return path

def findNeighborPath(path_list, r):
    path = 0
    length = 100
    for i in range(len(path_list)):
        if path_list[i][0] > r :
            if path_list[i][0]-r < length:
                path = i
                length = path_list[i][0]-r
    return path
    
def averagePath(ser, path_list, r, S6):
    #find the path with the nearest distance
    basicPath = findBasicPath(path_list, r)
    neighborPath = findNeighborPath(path_list, r)
    
    #as long as the demo length is dense enough(1 demo per grid)
    #then ratio is useless
    bpl = path_list[basicPath][0] 
    nbl = path_list[neighborPath][0]
    bratio = (r-bpl)/(nbl-bpl)
    #nratio = (nbl-r)/(nbl-bpl)
    
    
    ##find the S5, S4 average 
    basicS5 = path_list[basicPath][-1]['angles'][4]
    neighborS5 = path_list[neighborPath][-1]['angles'][4] 
    
    basicS4 = path_list[basicPath][-1]['angles'][3]
    neighborS4 = path_list[neighborPath][-1]['angles'][3]
    
    angles = path_list[basicPath][-1]['angles']
    deltaS5 = neighborS5 - basicS5
    deltaS4 = neighborS4 - basicS4
    angles[4] = int(basicS5 + deltaS5 * bratio)
    angles[3] = int(basicS4 + deltaS4 * bratio)
    angles[0] = 800 #open the claw
    angles[5] = S6
    
    ## command the ser
    send_command(ser, angles)
    
    ## grab it
    time.sleep(2)
    angles[0]= 1500
    send_command(ser, angles)
    
    ## pick it up
    for i in range(3,5):
        time.sleep(2)
        angles[i]=1500
        send_command(ser, angles)
    
    ## release the object
    time.sleep(2)
    angles[0]= 800
    send_command(ser, angles)

def directTheta(ser, path_list, r, p):
    theta = 0
    dx = p[0]-0
    dy = p[1]-(-8)
    print("dx:%f dy:%f"%(dx, dy))
    cos = (dx*0+dy*(8))/(math.sqrt(dx**2+dy**2)*8)
    print("cos:", cos)
    theta = math.degrees(math.acos(cos))
    if dx < 0:
        theta = 0-theta
    ## command the ser
    #angles = [1500, 1500, 1500, 1500, 1500+theta*1000/90]
    basicPath = findBasicPath(path_list, r)
    angles = path_list[basicPath][1]['angles']
    angles[5] = 1500-int(theta*1000/(90-5.625))
    send_command(ser, angles)
    time.sleep(2)
    print('theta:',theta)
    #input('wait')
    return angles[5]    
    
def BinSearchTheta(ser, path_list, r, p): # p:obj location, r:obj dis (r, theta)
    x2 = p[0]
    y2 = p[1]
    z2 = 0
    theta = 0
    for i in range(7):
        #1 is current motor location , r+8 = distance from obj to S6 
        x1 = (r+8)*math.sin(math.radians(theta))
        y1 = -8 + (r+8)*math.cos(math.radians(theta))
        z1 = 9.5 
        dis = Ddistance(x1,y1,z1,x2,y2,z2)
        
        #1 is the location after 1 degree left 
        x1 = 20.5*math.cos(math.radians(45))*math.sin(math.radians(theta-2))
        y1 = -8 + 20.5*math.cos(math.radians(45))*math.sin(math.radians(theta-2))
        ldis = Ddistance(x1,y1,z1,x2,y2,z2)
        x1 = 20.5*math.cos(math.radians(45))*math.sin(math.radians(theta+2))
        y1 = -8 + 20.5*math.cos(math.radians(45))*math.sin(math.radians(theta+2))
        rdis = Ddistance(x1,y1,z1,x2,y2,z2)
        if ldis < dis:# turn left if left will let the distance decrease
            theta = theta - 45/(2**i)
        else:
            theta = theta + 45/(2**i)
        print("ldis:%2f rdis:%2f"%(ldis, rdis))    
    ## command the ser
    #angles = [1500, 1500, 1500, 1500, 1500+theta*1000/90]
    basicPath = findBasicPath(path_list, r)
    angles = path_list[basicPath][1]['angles']
    angles[5] = 1500-int(theta*1000/90)
    send_command(ser, angles)
    time.sleep(2)
    print('theta:',theta)
    input('wait')
    return angles[5]    
    
def main(argv):
    try:
        ser=serial.Serial(COM , 9600 , timeout=1 , write_timeout=3)
        print(COM + " is open")
        time.sleep(5)
    except serial.SerialException:
        serial.Serial(COM, 9600).close()
        print(COM + " is closed")
        serial_port = serial.Serial(COM , 9600 , timeout=1 , write_timeout=3)
        print(COM + " is open again")
    
    #serial w
    ser.write(str(0).encode())  #command from the pc asking the arm to read command from pc not the arduino control
    while not ser.in_waiting:   #ser(pc) in_waiting get the buffer with info transfered from the arduino(reponse to the 0000)
        print('wait response ...')
        time.sleep(1.0)
    byte = ser.readline()
    string = byte.decode("utf-8")
    print(string,end='')
    
    ## load every path into an array called path_list
    path_list = [] 
    for i in range(1, len(argv)):
        path = argv[i]
        with open(path, "rb") as f:
            data = pickle.load(f)
            #print(type(data))   #list
            #print(len(data))  #lengh of the path
            #print(len(data[0])) #length = 6
            #print(data)
            path_list.append(data) #data[0] = r, data[i]=dic
            #path_list[i-1] = data   #append the path to the path_list
            print('path num: %d ; data num: %d' % (i, len(data)))
            
    #input('after init servo position ,enter to start behavior cloning')
    #many_path_behavior_clone(ser, path_list, 0.9, 0.35)
    ##r = input ('please enter the length between bottom and obj:')
    _, frame = cap.read()
    res = objectTrack(frame)
    center = findPixelLocation(res)
    r = toObjLocation(center)
    print("obj location:", r)
    dis = distanceToOriginal(r)
    
    # theta determination
    #S6 = BinSearchTheta(ser, path_list, int(float(dis)-8), r)
    S6 = directTheta(ser, path_list, int(float(dis)-8), r)
    
    #test theta
    angles = path_list[0][-1]['angles']
    angles[4] = 2100
    angles[3] = 1500
    angles[0] = 800 #open the claw
    angles[5] = S6
    send_command(ser, angles)
    input("check the theta")
    # r determination
    averagePath(ser, path_list, int(float(dis)-8), S6)
    ser.close()

if __name__ == "__main__":
    main(sys.argv[0:])