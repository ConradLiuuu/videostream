#! usr/bin/env python
import cv2
cap = cv2.VideoCapture(0)
if cap.isOpened() == 0:
    print("camera not open")
    
while True:
    print("aa")
    ret, frame = cap.read()
    #cv2.namedWindow("imgae", 0)
    cv2.imshow("imgae",frame)
    c = cv2.waitKey(1)
    
    if c == 27:
        break
        
cap.release()
cv2.destroyAllWindows()

