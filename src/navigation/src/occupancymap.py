import sys
import numpy as np
import matplotlib.pyplot as plt
import PIL
from PIL import Image
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'

if ros_path in sys.path:
	sys.path.remove(ros_path)

import cv2

sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')


gridline  = np.ones((3000,3000))
gridline = gridline * 255
pot_position_arr1 = []

# replace nasa.world with file name
my_file = open('NASA-MINDS1.world')

for line in my_file:
	# removes leading spaces ' ' and '<' in the line
    line = line.lstrip(' <')
    # gets the line starting with pose
    if line.startswith('model name=\'plant'):
        # gets to the next line
        line = my_file.__next__()
        # removes leading spaces ' ' and '<' in the line
        line = line.lstrip(' <')
        if line.startswith('pose'):
            pot_position_arr1.append(line.split('>')[1].split("<")[0])
x = []
y = []

for i in pot_position_arr1:
	x.append(float(i.split()[0]))
	y.append(float(i.split()[1]))
    

for i in range(0,len(x)):
	x_cen = round(1500+x[i]*100)
	print(x_cen)
	y_cen = round(1500-y[i]*100)
	print(y_cen)
	cv2.circle(gridline,(x_cen, y_cen), 25, 0, -1)

im = Image.fromarray(gridline)

im = im.convert("L")
im.save('occupancy3.jpg')
plt.imshow(im)
plt.show()

