import numpy as np
import cv2
import os
import shutil

def mkdir(d):
  if not os.path.exists(d):
      os.makedirs(d)

def rm(d):
  if os.path.exists(d):
    shutil.rmtree(d)

def main():
  cap = cv2.VideoCapture('vision.avi')
  directory = 'images'
  template = 'image%s.png'

  rm(directory)
  mkdir(directory)

  i = 0
  while cap.isOpened():
      ret, frame = cap.read()

      if frame.empty():
        break

      #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

      cv2.imshow('frame',frame)

      key = cv2.waitKey(1) # Milliseconds
      if key == ord('s'):
          path = os.path.join(directory, template%i)
          print 'Saved', path
          cv2.imwrite(path,frame)
          i += 1
      if key & 0xFF == ord('q'): # Unicode
          break

  cap.release()
  cv2.destroyAllWindows()

if __name__ == '__main__':
  main()
