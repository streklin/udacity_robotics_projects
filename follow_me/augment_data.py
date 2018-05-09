import os
import glob
from scipy import misc
import numpy as np

def flip_and_save_images(img_dir, extension):
  os.chdir(img_dir)
  files = glob.glob("*." + extension)
  for i, file in enumerate(files):
    print(i)
    img = misc.imread(file, flatten=False, mode='RGB')
    flipped_img = np.fliplr(img)
    misc.imsave("flipped" + file, flipped_img)

################

#data_folder_mask = os.path.join('..', 'data', 'train', 'masks')
data_folder_images = os.path.join('..', 'data', 'train', 'images')

#flip_and_save_images(data_folder_mask, "png")
flip_and_save_images(data_folder_images, "jpeg")