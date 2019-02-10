import time
import serial


COM="com3"
file_path="action.txt"
    

def read_instructions(my_serial,path):
    with open(path, "r") as text_file:
        lines = text_file.read().split('\n')
        print(lines)

        for line in lines :
            words=line.split(',')
            print(words)
            temp=''
            for i in range(6):
                n=words[i]
                print(n)
                temp += ' ' + n
            s = temp.encode()
            my_serial.write(s)
            print('serial write: ' + str(s))
            # log_motors_angle(my_serial)
            delay_s=float(words[6])
            time.sleep(delay_s)

def main():
    
    try:
        ser=serial.Serial(COM , 9600 , timeout=1 , write_timeout=3)
        print(COM + " is open")
        time.sleep(5)
    except serial.SerialException:
        serial.Serial(COM, 9600).close()
        print(COM + " is closed")
        serial_port = serial.Serial(COM , 9600 , timeout=1 , write_timeout=3)
        print(COM + " is open again")

    while 1:
        path = input("Enter file name: ")
        if path=="exit":
            break
        read_instructions(ser,path)
        
    
    ser.close()

if __name__ == "__main__":
    main()
        
