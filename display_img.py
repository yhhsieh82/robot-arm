import pickle
from PIL import Image
import os

dir_path = input("\nEnter file path: ")

for filename in os.listdir(dir_path):
    with open(filename, "rb") as f:
        data_list = pickle.load(f)
        print('data num: '+ str(len(data_list)))
        for data in data_list:
            data['pic'].show()