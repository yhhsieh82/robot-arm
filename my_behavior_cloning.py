# my inplementation of multi-path cloning
# can init the arm in a staate and make it continue move
# can't do the task that wasn't demo
# can't make a big angle move 
# step1. find the state with the same angles
# step2. choose the path with biggest pic similarity

import time
import serial
import pickle
import cv2
from image_processing import *
from test_webcam import *
from log_motor import *
import os
import math 
from crop3 import *
import numpy
import sys

MAX_SIMILARITY = 10
MIN_SIMILARITY = 0.6
TASK_DONE = False
COM="com22"

def distance(x1,y1,x2,y2):
    return math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))

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
    
def behavior_clone(ser, data_list, threshold, delay_time):
    angles_match = False
    while ser.in_waiting:
        cur_angles = read_angles(ser)
    
    # data_list is an array of  dictionary {('pic',[]),('angles',[])} 
    for i,data in enumerate(data_list):  
        ret = False
        while not ret:
            ret,img = cap.read()
            
        show_image(np.hstack([make_regular_image(img),data['pic']]))
        data_angles = data['angles']
        
        ## base on the angle comparison at last state:
        ## if not matched, then compare the angle
        ## if matched, then do the picture similarity '''
        if not angles_match: 
            print('cur_angles : %d %d %d %d %d %d'%(cur_angles[0],cur_angles[1],cur_angles[2],cur_angles[3],cur_angles[4],cur_angles[5]))
            print('data_angles : %d %d %d %d %d %d'%(data_angles[0],data_angles[1],data_angles[2],data_angles[3],data_angles[4],data_angles[5]))
            if not compare_angles(cur_angles,data_angles):
                print('angles not equal')
            else:
                print('angles match')
                angles_match = True
            continue
        
        ## compare the picture similarity
        # pixel_similarity = get_image_similarity(img,data['pic'])
        # print('pixel similarity : %.6f'%pixel_similarity)
        hash_similarity = image_hash_similarity(img, data['pic'], 16, 16)
        print('hash similarity : phash=%.6f , ahash=%.6f , dhash=%.6f , whash=%.6f'%(hash_similarity[0],hash_similarity[1],hash_similarity[2],hash_similarity[3]))
        similarity = hash_similarity[0]
        
        ## stop when the picture similarity < 0.9
        ## how to distinguish the wrong path and the bad similarity?
        if similarity < threshold:
            print('stop')
            # return
            if similarity > 0.82:
                input('enter to continue')
            else:
                print('the wrong path')
                break
            
        ## command the ser
        send_command(ser,data['angles'])
        #time.sleep(delay_time)
        
        
        ## if the last images are the same, then TASK_DONE=True
        #TODO
        if (i==len(data_list) and angles_match == True and not(similarity < threshold)):
            TASK_DONE = True
            print('')
        
        
        '''????'''
        while ser.in_waiting:
            byte = ser.readline()
            string = byte.decode("utf-8")
            print('arduino -> '+string,end='')
    print('end cloning')

#path_list is a list of data
def many_path_behavior_clone(ser, path_list, threshold, delay_time):
    angles_match = False
  
    while not ser.in_waiting:
        pass
    cur_angles = read_angles(ser)
    
    #time.sleep(1)
    #read in the image 
    ret = False
    while not ret:
        ret, img = cap.read()
        
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
    """
    ## none state by state compare (direct jump to turning point of the state) 
    ## how do you get the img after u command the serial?
    #for j in range(frame_num, len(path_list[path_num])):    
    j = frame_num
    while (j < len(path_list[path_num])):
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
            if similarity > 0.82:
                input('enter to continue')
            else:
                print('the wrong path')
                break
        
        ## command the ser
        current_frame = j   
        current_action = path_list[path_num][j]['action']
        while ( j < len(path_list[path_num]) and path_list[path_num][j]['action']==current_action):
            j += 1
        send_command(ser, path_list[path_num][j]['angles'])
        
        ##　????
        while ser.in_waiting:
            byte = ser.readline()
            string = byte.decode("utf-8")
            print('arduino -> '+string,end='')
    """          
    print('end cloning')
        
    
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
            path_list.append(data)
            #path_list[i-1] = data   #append the path to the path_list
            print('path num: %d ; data num: %d' % (i, len(data)))
            
    input('after init servo position ,enter to start behavior cloning')
    many_path_behavior_clone(ser, path_list, 0.9, 0.35)
       
    # when there is only one path
    ''' 
    path = argv[1]
    with open(path, "rb") as f:
        data = pickle.load(f)
        print('data num: '+ str(len(data)))
    input('after init servo position ,enter to start behavior cloning')
    behavior_clone(ser,data,0.9,0.35)
    '''
    ser.close()

if __name__ == "__main__":
    main(sys.argv[0:])