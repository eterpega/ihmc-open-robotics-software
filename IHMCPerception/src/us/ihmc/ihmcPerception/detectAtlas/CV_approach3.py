import cv2
import numpy as np
import os

fourcc = cv2.VideoWriter_fourcc(*'MJPG')
out = cv2.VideoWriter('output.avi', fourcc, 20.0, (1280, 400))

if __name__ == '__main__':
	path = "/home/shadylady/Repositories/IHMCOpenRoboticsSoftware/IHMCPerception/resources/detectAtlas/extractedImages-Video1/"
	image_names = os.listdir(path)

	prev_x = None
	prev_y = None
	curr_x = None
	curr_y = None

	for img in image_names:
		image = cv2.imread(path+img)
		h, w, c = image.shape
		grid_size = 5

		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		blur = cv2.blur(gray, (5,5))
		canny = cv2.Canny(blur, 100, 200)

		orb = cv2.ORB_create()
		key_points, descriptors = orb.detectAndCompute(canny, None)

		grid_x = []
		grid_y = []

		for i in range(grid_size+1):
			if i == 0:
				grid_x.append(0)
				grid_y.append(0)
			elif i == grid_size:
				grid_x.append(w)
				grid_y.append(h)
			else:
				grid_x.append(int(w*i/grid_size))
				grid_y.append(int(h*i/grid_size))

		count = [[None]*grid_size for _ in range(grid_size)]

		for kp in key_points:
			x = 0
			y = 0

			for i in range(len(grid_x)-1):
				if kp.pt[0] >= grid_x[i] and kp.pt[0] < grid_x[i+1]:
					x = i
			for i in range(len(grid_y) - 1):
				if kp.pt[1] >= grid_y[i] and kp.pt[1] < grid_y[i+1]:
					y = i

			if count[x][y] == None:
				count[x][y] = [kp]
			else:
				count[x][y].append(kp)

		max = 0
		for c1 in range(len(count)):
			for c2 in range(len(count[0])):
				if count[c1][c2] is not None:
					c = len(count[c1][c2])
					if c > max:
						max = c
						curr_x = c1
						curr_y = c2

		if prev_x is None and prev_y is None:
			prev_x = curr_x
			prev_y = curr_y

		image_kp = canny.copy()
		if (abs(curr_x - prev_x) < 2 or abs(curr_y - prev_y) < 2):
			image_kp = cv2.drawKeypoints(canny, count[curr_x][curr_y], image_kp, color=(0, 255, 0), flags=0)
			prev_x = curr_x
			prev_y = curr_y

		for i in range(grid_size):
			for j in range(grid_size):
				image = cv2.rectangle(image, (grid_x[i], grid_y[j]), (grid_x[i+1], grid_y[j+1]), color=(255,0,0), thickness=2)

		image = cv2.rectangle(image, (grid_x[prev_x], grid_y[prev_y]), (grid_x[prev_x + 1], grid_y[prev_y + 1]), color=(0, 255, 0), thickness=2)

		# cv2.imshow("Image_KP", image_kp)
		# cv2.imshow("Image", image)

		joined = np.concatenate((image, image_kp), axis=1)
		# cv2.imwrite('out.png', vis)
		out.write(joined)

		k = cv2.waitKey(1) & 0xff
		if (k == 27):
			break

	out.release()
	print("Done!")

