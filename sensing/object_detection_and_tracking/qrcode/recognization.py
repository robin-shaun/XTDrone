import cv2 as cv
import numpy as np
im = cv.imread('up.jpg')
det = cv.QRCodeDetector()
retval, decoded_info, points, straight_qrcode = det.detectAndDecodeMulti(im)
print(retval)