# editer : MT
# image processing for opencv image (ndarray)

import time
import cv2
import numpy as np
import imagehash
from PIL import Image

start_time = 0
def tic():
    global start_time
    start_time = time.time()

def toc():
    print("time: %.6f"%(time.time()-start_time))

regular_w = 256
regular_h = 256
pw, ph = (64, 64)
rgb_table = None
lab_table = None

def make_regular_image(img):
    global regular_h,regular_w
    return cv2.resize(img, (regular_w,regular_h), interpolation=cv2.INTER_LINEAR)

# use k-means in rbg and lab color space 
# @img : image samples
# @K : number of colors in each channel
# @return : centers in each channel
def init_quantization_color(imgs,K):
    global rgb_table,lab_table
    Z = imgs.copy()
    
    rgb_img = Z.reshape((-1,3))
    rgb_img = np.float32(rgb_img)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 1.0)
    r_ret,r_label,r_center=cv2.kmeans(rgb_img[:,0],K,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
    g_ret,g_label,g_center=cv2.kmeans(rgb_img[:,1],K,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
    b_ret,b_label,b_center=cv2.kmeans(rgb_img[:,2],K,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
    # print(r_ret)
    # print(r_label)
    print('rgb centers')
    print(sorted(r_center.ravel()))
    print(sorted(g_center.ravel()))
    print(sorted(b_center.ravel()))
    rgb_table = create_color_mapping_table((r_center,g_center,b_center))

    Z = cv2.cvtColor(Z, cv2.COLOR_BGR2LAB)
    lab_img = Z.reshape((-1,3))
    lab_img = np.float32(lab_img)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 1.0)
    L_ret,L_label,L_center=cv2.kmeans(lab_img[:,0],K,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
    A_ret,A_label,A_center=cv2.kmeans(lab_img[:,1],K,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
    B_ret,B_label,B_center=cv2.kmeans(lab_img[:,2],K,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
    lab_table = create_color_mapping_table((L_center,A_center,B_center))
    print('Lab centers')
    print(sorted(L_center.ravel()))
    print(sorted(A_center.ravel()))
    print(sorted(B_center.ravel()))

# use centers to create color mapping table
def create_color_mapping_table(centers):
    table = []
    for c in range(3):
        color_256 = []
        for i in range(256):
            color_256.append(color_quantization(i,centers[c]))
        table.append(color_256)
    # print(table)
    return table

# get color in mapping table
def get_color_from_table(color,table):
    r,g,b = color
    return [table[0][r],table[1][g],table[2][b]]

# @pixel : r,g,b
# @centers : colors
def color_quantization(pixel,centers):
    min_dist = cv2.norm((255,255,255))
    nn = centers[0]
    for i in range(len(centers)):
        neighbor = centers[i]
        dist = cv2.norm(pixel - neighbor)
        if dist < min_dist:
            min_dist = dist
            nn = neighbor
    return nn

# @img : image (rgb)
# @color_space : input image color space 
def image_quantization(img,color_space='rgb'):
    global rgb_table,lab_table
    if color_space == 'rgb':
        table = rgb_table
    else:
        table = lab_table
    Z = img.copy()
    Z = Z.reshape((-1,3))
    for i in range(Z.shape[0]):
        Z[i,:] = get_color_from_table(Z[i,:],table)
    Z = Z.reshape((img.shape))
    return Z


def getRGB(image):
    r = image[:,:,2]
    g = image[:,:,1]
    b = image[:,:,0]
    return r,g,b

def image_similarity_preprocess(img,img_size=(256,256),blur_size=5,color_space='lab'):
    img = cv2.resize(img, img_size, interpolation=cv2.INTER_LINEAR)
    img_blur = cv2.GaussianBlur(img, (blur_size, blur_size), 0)
    if color_space == 'rgb':
        return image_quantization(img,'rgb')
    elif color_space == 'lab':
        img = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        return image_quantization(img,'lab')
    elif color_space == 'all':
        img_rgb = image_quantization(img,'rgb')
        img_lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        img_lab = image_quantization(img_lab,'lab')
        return img,img_blur,img_rgb,img_lab
    else:
        return img_blur

def get_image_similarity(first_img, second_img):

    img1 = image_similarity_preprocess(first_img)
    img2 = image_similarity_preprocess(second_img)
    percentage = compare_image_pixels(img1,img2,3)
 
    return percentage

def compare_image_pixels(img1,img2,diff_range):
    cnt = 0
    diff = abs( np.int32(img1) - np.int32(img2) )
    for pixel_diff in np.nditer(diff):
        if pixel_diff <= diff_range:
            cnt += 1
    pixel_number = 1
    for i in img1.shape:
        pixel_number *= i
    # print (pixel_number)
    # print(cn
    return cnt/pixel_number

def image_hash(img,size,freq):
    phash = imagehash.phash(Image.fromarray(cv2.cvtColor(img,cv2.COLOR_BGR2RGB)),hash_size=size,highfreq_factor=freq)
    ahash = imagehash.average_hash(Image.fromarray(cv2.cvtColor(img,cv2.COLOR_BGR2RGB)),hash_size=size)
    dhash = imagehash.dhash(Image.fromarray(cv2.cvtColor(img,cv2.COLOR_BGR2RGB)),hash_size=size)
    whash = imagehash.whash(Image.fromarray(cv2.cvtColor(img,cv2.COLOR_BGR2RGB)),hash_size=size)
    return phash,ahash,dhash,whash

def compare_hash(hash1,hash2):
    if hash1 == hash2:
        return True
    return False

def image_hash_similarity(img1,img2,hash_size=8,freq=8):
    hash_1 = image_hash(img1,hash_size,freq)
    hash_2 = image_hash(img2,hash_size,freq)
    sim = []
    for i in range(4):
        # print(hash_1[i])
        # print(hash_2[i])
        sim.append( 1 - (hash_1[i] - hash_2[i])/len(hash_1[i].hash)**2 )
        # print(sim)
    return sim

def main():
    img1 = cv2.imread('img1.png')
    img2 = cv2.imread('img2.png')
    # print(get_image_similarity(img1,img2))
    image_hash_similarity(img1,img2,16)


if __name__ == "__main__":
    main()
