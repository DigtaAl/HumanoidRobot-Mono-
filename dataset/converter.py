import cv2
import sys
import os

n = len(sys.argv)
if n <= 1 or not sys.argv:
    print("converter.py [file video] [folder dataset]")
    exit()

file_name = sys.argv[1]
folder_name = sys.argv[2]
os.mkdir("./"+folder_name)

print(file_name)
video = cv2.VideoCapture(file_name)
count = 0

while True:
    ret, frame = video.read()
    if ret:
        count += 1
        cv2.imwrite(folder_name+"/"+file_name.split('.')[0]+"-%d.jpg" % count, frame)
    else:
        print("done")
        break
