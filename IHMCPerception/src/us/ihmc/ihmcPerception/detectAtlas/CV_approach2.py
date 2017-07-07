import cv2
import numpy as np
import os

if __name__ == '__main__':
	path = "/home/shadylady/Repositories/IHMCOpenRoboticsSoftware/IHMCPerception/resources/detectAtlas/extractedImages-Video2/"
	image_names = os.listdir(path)

	for img in image_names:
		image = cv2.imread(path + img, 0)
		cv2.imshow("Image OG", image)
		image = (image - 127.5)/255.0
		cv2.imshow("Normalized", image)

		blur = cv2.blur(image, (5,5))
		print(np.min(np.min(blur, axis=1), axis=0))
		print(np.max(np.max(blur, axis=1), axis=0))
		# canny = cv2.Canny(blur, (100-128.0)/128.0, (200-128.0)/128.0)

		cv2.imshow("Blur", blur)
		# cv2.imshow("Canny", canny)

		k = cv2.waitKey(0) & 0xff
		if (k == 27):
			break