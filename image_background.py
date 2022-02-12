from json.tool import main
import cv2
import numpy as np
import random, os

def backg(main_img):
    f_img = main_img

    back_img = random.choice(os.listdir('/root/ocrtoc_ws/src/rendering_setup_iros22/images'))
    back_img = os.path.join('/root/ocrtoc_ws/src/rendering_setup_iros22/images', back_img)
    print(back_img)
    b_img = cv2.imread(back_img)

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
    return img_res

def main():

    folder = '/root/ocrtoc_ws/src/rendering_setup_iros22/data_models'
    for root, subdirectories, files in os.walk(folder):
        for subdirectory in subdirectories:
            if subdirectory =='black_marker_446':
                continue
            else:
                folder_path = os.path.join(root,subdirectory)
                for rt, subdirectories, files in os.walk(folder_path):
                    orig_img = os.path.join(rt, 'projection.png')
                    print(orig_img)

                    img = cv2.imread(orig_img)

                    if not os.path.exists(os.path.join(rt, 'projection_orig.png')):
                        save_orig = os.path.join(rt, 'projection_orig.png')
                        print(save_orig)
                        cv2.imwrite(save_orig, img)
                    else:
                        print("exists")
                    
                    
                    backimg = backg(img)
                    cv2.imwrite(os.path.join(rt, 'projection.png'), backimg)
                        # exit()

main()




