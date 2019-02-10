import os
import pickle
import time
import cv2
import keyboard
import serial
from test_webcam import *
from image_processing import *
from edgeDetection import *

# COM="/dev/cu.usbmodem1411"
COM='com16'


def read_angles(ser):
    byte = ser.readline()
    string = byte.decode("utf-8")
    angles = string.split(' ')
    angles = angles[1:7]
    for i in range(len(angles)):
        angles[i] = int(angles[i])
    return angles

def log_data(ser):
    data_list=[]
    start_time = time.time()
    
    ## the first element stores the distance between the bottom and the obj
    #r = input('input the length between the obj and the bottom of the arm:')# r is the legth between the obj and the bottom of the arm
    ret, frame = cap.read()
    ret, frame = cap.read()
    ret, frame = cap.read()
    show_image(frame)
    
    res = objectTrack(frame)
    center = findPixelLocation(res)
    #print('cx:%d'% center[0])
    #print('cy:%d'% center[1])
    r = toObjLocation(center)
    #print('rx:%s'% str(r[0]))
    #print('ry:%s'% str(r[1]))
    dis = distanceToOriginal(r)
    data_list.append(int(dis))
    #data_list.append(int(r))
    
    while not keyboard.is_pressed('q'):             
        if not ser.in_waiting:
            continue
        
        angles = read_angles(ser)
        print(angles)
        
        time_str = '%.3f'%(time.time()-start_time)
        print('time: '+  time_str +' -> ',end='')
        
        dic = {}
        dic['time'] = float(time_str)
        #dic['pic'] = make_regular_image(frame)
        dic['angles'] = angles
        dic['next_angles'] = None
        dic['action'] = -1
        
        data_list.append(dic)
        
    return data_list
    

def main():  
    try:
        ser=serial.Serial(COM , 9600 , timeout=5 , write_timeout=3)
        print(COM + " is open")
        time.sleep(5)
    except serial.SerialException:
        serial.Serial(COM, 9600).close()
        print(COM + " is closed")
        serial_port = serial.Serial(COM , 9600 , timeout=5 , write_timeout=3)
        print(COM + " is open again")
    
    while 1:
        ser.reset_input_buffer()
        file_name = input('enter the name of the file to save: ')
        ser.write(str(1).encode())
        while not ser.in_waiting:
            print('wait response ...')
            time.sleep(1.0)
        byte = ser.readline()
        string = byte.decode("utf-8")
        print(string,end='')

        data_list = log_data(ser)

        print('save data ...')
        if not os.path.exists('data') or not os.path.isdir('data'):
            os.mkdir('data')

        file_name = os.path.join('data',file_name)
        with open(file_name,'wb+') as f:
            pickle.dump(data_list, f)

    ser.close()
    cap.release()

if __name__ == "__main__":
    main()