from json.tool import main
import cv2
import numpy as np
import random, os
file_name = random.choice(os.listdir('/home/vishal_mandadi/iros_rendering/rendering_setup_iros22/images'))
print(file_name)
file = os.path.join('/home/vishal_mandadi/iros_rendering/rendering_setup_iros22/images', file_name)
back_img = cv2.imread(file)
main_img = cv2.imread('/home/vishal_mandadi/iros_rendering/rendering_setup_iros22/data_models/potato_chip_2_359886/projection.png')

f_img = main_img

b_img = back_img

color_limits = [[0, 1], [0, 1], [0, 1]]

img_res_array = []

# counter = 0
print('1')
alpha_channel = np.ones(shape=(f_img.shape[0], f_img.shape[1], 3))
b_img = cv2.resize(b_img, ( f_img.shape[1], f_img.shape[0]))

red_ch_mask = np.logical_and(f_img[:, :, 0] >= 0, f_img[:, :, 0] <= 1)
green_ch_mask = np.logical_and(f_img[:, :, 1] >= 0, f_img[:, :, 1] <= 1)
blue_ch_mask = np.logical_and(f_img[:, :, 2] >= 0, f_img[:, :, 2] <= 1)

for i in range(3):
    alpha_channel[:, :, i] = np.logical_not(np.logical_and(np.logical_and(red_ch_mask, green_ch_mask), blue_ch_mask))

img_res = np.multiply(f_img, alpha_channel) + np.multiply(b_img, (1 - alpha_channel))

img_res = img_res.astype(np.uint8)
# img_res_array.append(img_res)
print('2')
# counter += 1
# if counter == 5:
#     break
cv2.imshow('img', img_res)
cv2.waitKey(0)
