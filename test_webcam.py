import cv2
from crop3 import *
from image_processing import *
import os

camera_id = 0
cap = cv2.VideoCapture(camera_id)
print('open camera %d'%camera_id)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

def similarity(img1,img1_blur,img1_rgb,img1_lab,img2,img2_blur,img2_rgb,img2_lab):
        # v = compare_image_pixels(img1,img2,3)
        # v1 = compare_image_pixels(img1_blur,img2_blur,3)
        # v2 = compare_image_pixels(img1_rgb,img2_rgb,3)
        # v3 = compare_image_pixels(img1_lab[:,:,:1],img2_lab[:,:,:1],3)
        # print('pixel: %f , %f , %f , %f'%(v,v1,v2,v3))
        # img1_lab = cv2.cvtColor(img1_lab, cv2.COLOR_LAB2BGR)
        # img2_lab = cv2.cvtColor(img2_lab, cv2.COLOR_LAB2BGR)

        hash_size = 8
        hash_v = image_hash_similarity(img1,img2,hash_size)
        hash_v1 = image_hash_similarity(img1_blur,img2_blur,hash_size)
        hash_v2 = image_hash_similarity(img1_rgb,img2_rgb,hash_size)
        hash_v3 = image_hash_similarity(img1_lab,img2_lab,hash_size)
        print('hash: %f , %f , %f , %f'%(hash_v[0],hash_v1[0],hash_v2[0],hash_v3[0]))
        
        # img_up = np.hstack([img1,img1_blur,img1_rgb,img1_lab])
        img_down = np.hstack([img2,img2_blur,img2_rgb,img2_lab])
        # img = np.vstack([img_up,img_down])
        # return img
        return img_down

def show_image(img):
    cv2.namedWindow('frame')
    cv2.imshow('frame',img)
    if cv2.waitKey(1) & 0xFF == ord('w'):
        cv2.destroyWindow("frame")
    

def main():
    while 1:
        print('try to read frame1')
        ret ,frame1 = cap.read()
        print('ret = '+str(ret))
        if ret:
            break
    ret ,frame1 = cap.read()
    cv2.imwrite('img1.png',frame1)
    ret ,frame2 = cap.read()
    cv2.imwrite('img2.png',frame2)

    # print('calculate centers...')
    # imgs = []
    # for i in range(10):
    #     ret, frame = cap.read()
    #     frame = cv2.resize(frame,(256,256))
    #     imgs.append(frame)
    # init_quantization_color(np.hstack(imgs),4)
    # print('create table finish')

    # v = get_image_similarity(frame1,frame2)
    # print(v)

    print('start while loop')
    _,last_frame = cap.read()
    # last_frame = cv2.resize(last_frame,(256,256))
    # last_frame,last_frame_blur,last_frame_rgb,last_frame_lab = image_similarity_preprocess(last_frame,img_size=(256,256),color_space='all')
    while 1:
        tic()
        # cv2.namedWindow('frame')
        ret, frame = cap.read()
        hashes = image_hash_similarity(last_frame,frame,8)
        print('hash: %f , %f , %f , %f'%(hashes[0],hashes[1],hashes[2],hashes[3]))
        # frame = cv2.resize(frame,(256,256))
        # frame,frame_blur,frame_rgb,frame_lab = image_similarity_preprocess(frame,img_size=(256,256),color_space='all')
        # img = similarity(last_frame,last_frame_blur,last_frame_rgb,last_frame_lab,frame,frame_blur,frame_rgb,frame_lab)
        # hash_v = image_hash_similarity(last_frame,frame,16)
        # print('hash: %f'%hash_v)
        img = np.hstack([last_frame,frame])
        # cv2.imshow('frame',img)
        show_image(img)
        # print(frame.shape)
        last_frame = frame
        # last_frame_blur = frame_blur
        # last_frame_rgb = frame_rgb
        # last_frame_lab = frame_lab
        # if cv2.waitKey(1) & 0xFF == ord('w'):
        #     cv2.destroyWindow("frame")
        toc()


if __name__ == "__main__":
    main()

