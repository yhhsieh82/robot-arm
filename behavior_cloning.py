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
COM="com22"

def distance(x1,y1,x2,y2):
    return math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))

def recognize_first_img(dir_path, img):
    MAX = 0.0
    MIN = 100000
    MAX_filename = ''
    img1,x1,y1 = get_contour(img)
    for filename in os.listdir(dir_path):
        path = os.path.join(dir_path,filename)
        with open(path, "rb") as f:
            data_list = pickle.load(f)
            print('data num: '+ str(len(data_list)))
            # sim = Get_Similarity_By_Path(img, data_list[0]['pic'])
            # print('%s , similarity: %f' % (filename, sim))
            cv_image = cv2.cvtColor(numpy.array(data_list[0]['pic']), cv2.COLOR_RGB2BGR)
            img2,x2,y2 = get_contour(cv_image)
            d = distance(x1[0],y1[0],x2[0],y2[0])
            print('%s , distance: %f' % (filename, d))
            if d < MIN:
                MIN = d
                MAX_filename = filename

            # if sim > MAX:
            #     MAX = sim
            #     MAX_filename = filename
    # if MAX > MIN_SIMILARITY:
    if MIN < MAX_SIMILARITY:
        return MAX_filename
    else:
        return 'error'

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

def find_similar_state(data,cur_angles,cur_img,threshold):
    # Find the state that the angles are the same.
    buffer = []
    for frame in data:
        data_angles = frame['angles']
        if compare_angles(cur_angles, data_angles):
            buffer.append(frame)
    if not buffer:
        print('can not find matched angles')
    max_similarity = 0
    max_frame = None
    for frame in buffer:
        hash_similarity = image_hash_similarity(cur_img, frame['pic'], 16, 16)
        print('hash similarity : phash=%.6f , ahash=%.6f , dhash=%.6f, whash=%.6f'\
            %(hash_similarity[0], hash_similarity[1], hash_similarity[2], hash_similarity[3]))
        similarity = hash_similarity[0]
        if similarity > max_similarity and similarity >= threshold:
            max_similarity = similarity
            max_frame = frame
    if max_frame == None:
        print('similarity is lower than threshold')
    return max_frame,buffer

def load_data(dir_path):
    data_list = []
    for filename in os.listdir(dir_path):
        path = os.path.join(dir_path,filename)
        with open(path, "rb") as f:
            data = pickle.load(f)
            data_list += data
    return data_list

def data_preprocess(data_list):
    print('data preprocess ...')
    for i, data in enumerate(data_list):
        k = i
        if k+1 < len(data_list):
            while data['action'] == data_list[k+1]['action']:
                k += 1
        data['next_angles'] = data_list[k]['next_angles']
    return data_list

def multi_behavior_clone(ser,data_list,threshold,delay_time):
    data_list = data_preprocess(data_list)
    cur_angles = None
    while not ser.in_waiting:
        pass
    # Read current angles.
    cur_angles = read_angles(ser)
    print(cur_angles)
    while 1:
        ret, cur_img = cap.read()
        frame,imgs = find_similar_state(data_list,cur_angles,cur_img,threshold)
        if frame == None:
            all_image = [make_regular_image(cur_img)]
            for img in imgs:
                all_image.append(img['pic'])
            show_image(np.hstack(all_image))
            end = input('try again?')
            if end =='q':
                break
            else:
                continue
        if frame['next_angles'] == None:
            print('end state')
            break
        show_image(np.hstack([make_regular_image(cur_img), frame['pic']]))
        send_command(ser, frame['next_angles'])
        time.sleep(delay_time)
        input("wait")
        cur_angles = frame['next_angles']
        print('cur_angles : %d %d %d %d %d %d'%(cur_angles[0], cur_angles[1], \
                cur_angles[2], cur_angles[3], cur_angles[4], cur_angles[5]))
    

def behavior_clone():
    angles_match = False
    # while ser.in_waiting:
    #     # Read current angles.
    #     cur_angles = read_angles(ser)
    while ser.in_waiting:
        # Read current angles.
        cur_angles = read_angles(ser)
    i = 0
    while i+1 < len(data_list):
    # for i in range(len(data_list)-1):
        # Check angles and picture similarity at first state, then check picture only.
        ret, img = cap.read()
        show_image(np.hstack([make_regular_image(img), data_list[i]['pic']]))
        data_angles = data_list[i]['angles']
        
        if not angles_match: 
            print('cur_angles : %d %d %d %d %d %d'%(cur_angles[0], cur_angles[1], \
                cur_angles[2], cur_angles[3], cur_angles[4], cur_angles[5]))
            print('data_angles : %d %d %d %d %d %d'%(data_angles[0], data_angles[1], \
                data_angles[2], data_angles[3], data_angles[4], data_angles[5]))

            if not compare_angles(cur_angles, data_angles):
                print('angles not equal')
            else:
                print('angles match')
                angles_match = True
            # continue

        # pixel_similarity = get_image_similarity(img,data_list[i]['pic'])
        # Get the maximun similarity of all the states.
        hash_similarity = image_hash_similarity(img, data_list[i]['pic'], 16, 16)
        
        print('hash similarity : phash=%.6f , ahash=%.6f , dhash=%.6f, whash=%.6f'\
            %(hash_similarity[0], hash_similarity[1], hash_similarity[2], hash_similarity[3]))
        similarity = hash_similarity[0]
        if similarity < threshold:
            print('stop')
            # return
            input('enter to continue')
        # Take the action (to next state).
        # i += data_list[i]['action_rep'] (For simplify states.)
        send_command(ser, data_list[i+1]['angles'])
        i += 1
        time.sleep(delay_time)
        while ser.in_waiting:
            byte = ser.readline()
            string = byte.decode("utf-8")
            print('arduino -> '+string, end='')
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

    ser.write(str(0).encode())
    while not ser.in_waiting:
        print('wait response ...')
        time.sleep(1.0)
    byte = ser.readline()
    string = byte.decode("utf-8")
    print(string,end='')

    # print('calculate centers...')
    # imgs = []
    # for i in range(10):
    #     ret, frame = cap.read()
    #     frame = cv2.resize(frame,(256,256))
    #     imgs.append(frame)
    # init_quantization_color(np.hstack(imgs),4)
    # print('create table finish')

    input('after init servo position ,enter to start behavior cloning')

    path = argv[1]
    # with open('data/'+path, "rb") as f:
    #     data = pickle.load(f)
    #     print('data num: '+ str(len(data)))
    # behavior_clone(ser, data, 0.9, 0.35)

    data = load_data(path)
    multi_behavior_clone(ser, data, 0.9, 0.35)
    # Start mapping the first frame of camera to data_list.
    # path = input("\nEnter file name: "
    # img = read_frame_with_pil(cap)
    # ret , img = cap.read()
    # img = make_regular_image(img)
    # filename = recognize_first_img('data',img)
    # if filename != 'error':
    #     # go next state with idx
    #     ser.write(str(0).encode())
    #     while not ser.in_waiting:
    #         print('wait response ...')
    #         time.sleep(1.0)
    #     byte = ser.readline()
    #     string = byte.decode("utf-8")
    #     print(string,end='')
    #     path = os.path.join('data',filename)
    #     read_instructions(ser,path)
    # else:
    #     # stop
    #     print('can not find action!')


    # while 1:
    #     path = input("\nEnter file name: ")
    #     if path=="exit":
    #         break
    #     read_instructions(ser,path)
        
    
    ser.close()

if __name__ == "__main__":
    main(sys.argv[0:])