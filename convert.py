import cv2, os



folder = '/root/ocrtoc_ws/src/rendering_setup_iros22/cache_data'
folderimg = '/root/ocrtoc_ws/src/rendering_setup_iros22/cache_data1'

for root, subdirectories, files in os.walk(folder):
        # print(root)
        for fold in subdirectories:
            # print(fold)
            folder_ = os.path.join(folder,fold)
            for root, subdirectories, files in os.walk(folder_):
                
                # folder_name = os.path.join(folder,fold)
                # file = filename.rsplit('.', 1)
                # print(file)
                # print(folder_name)
                file_name = os.path.join(folder_,'projection.png')
                print(file_name)
                img = cv2.imread(file_name)
               
                if img is not None:
                    cv2.imwrite(file_name, cv2.cvtColor(img, cv2.COLOR_BGR2RGB))  
                # # return images