import numpy as np
import cv2

WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
BLACK = (0, 0, 0)

width = 512
height = 512

# Create a black image
#img = np.zeros((width,height,3), np.uint8)
img = 255*np.ones((width,height,3), np.uint8)

# Draw a diagonal blue line with thickness of 5 px
img = cv2.line(img, (0,0), (511,511), RED, 5)

#img = cv2.rectangle(img, (384,0), (510,128), GREEN, 3)
img = cv2.rectangle(img, (384,0), (510,128), GREEN, -1)

img = cv2.circle(img, (447,63), 63, BLUE, -1)

img = cv2.ellipse(img,(256,256),(100,50),0,0,180,255,-1)


pts = np.array([[10,5],[20,30],[70,20],[50,10]], np.int32)
pts = pts.reshape((-1,1,2))
img = cv2.polylines(img,[pts],True,(0,255,255))


font = cv2.FONT_HERSHEY_SIMPLEX
cv2.putText(img, 'OpenCV', (10,500), font, 4, BLACK, 2, cv2.LINE_AA)

# http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_gui/py_drawing_functions/py_drawing_functions.html

cv2.imshow('image',img)
cv2.waitKey(0)
cv2.destroyAllWindows()

from PIL import Image, ImageDraw, ImageColor

print ImageColor.getrgb('white')
mode = 'RGB' # 'L'

img = Image.new(mode=mode, size=(width, height), color=WHITE)
#ImageDraw.Draw(img).polygon(pts, outline=1, fill=1)
ImageDraw.Draw(img).rectangle([(384,0), (510,128)], outline=1, fill=RED)

mask = np.array(img)

cv2.imshow('image',mask)
cv2.waitKey(0)
cv2.destroyAllWindows()

# http://pillow.readthedocs.io/en/3.4.x/reference/Image.html
