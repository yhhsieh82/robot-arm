import os
import pickle
import time
import cv2
import keyboard
import serial
from test_webcam import *
from image_processing import *

# COM="/dev/cu.usbmodem1411"
COM='com16'
# cap.set(cv2.CAP_PROP_FRAME_WIDTH,720)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH,720)

# import pygame
# pygame.init()
# BLACK = (0,0,0)
# WIDTH = 1280
# HEIGHT = 1024
# windowSurface = pygame.display.set_mode((WIDTH, HEIGHT), 0, 32)

# windowSurface.fill(BLACK)

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
    # pressed = False
    #  
    while not keyboard.is_pressed('q'):
    # while not pressed:       
    #     for event in pygame.event.get():
    #         if event.type == pygame.KEYDOWN and event.key == pygame.K_w:
    #             pressed = True               
        if not ser.in_waiting:
            continue
        dic = {}
        angles = read_angles(ser)
        #time.delay
        '''
        ret, frame = cap.read()
        show_image(frame)
        '''

        time_str = '%.3f'%(time.time()-start_time)
        print('time: '+  time_str +' -> ',end='')
        print(angles)
        dic['time'] = float(time_str)
        #dic['pic'] = make_regular_image(frame)
        dic['angles'] = angles
        dic['next_angles'] = None
        dic['action'] = -1
        
        # Mark the actions.
        if len(data_list)>=1:
            for i in range(len(angles)):
                if angles[i] - data_list[-1]['angles'][i] != 0:
                    data_list[-1]['action'] = i
                    data_list[-1]['next_angles'] = angles
                    break

        data_list.append(dic)
    # cv2.destroyAllWindows()
    for i, data in enumerate(data_list):
        k = i
        if k+1 < len(data_list):
            while data['action'] == data_list[k+1]['action']:
                k += 1
        data['action_rep'] = k-i
        print('angles: ', end='')
        print(data['angles'])
        print('action: %d, rep: %d' % (data['action'], data['action_rep']))

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
