import cv2
import os
import numpy as np
import collections

def SIFT(image_in):
	sift = cv2.xfeatures2d.SIFT_create()
	kp, des = sift.detectAndCompute(image_in, None)
	return kp, des

def ORB(image_in):
	orb = cv2.ORB_create()
	kp, des = orb.detectAndCompute(image_in, None)
	return kp, des

def draw_keypoints(image_in, kp, color=None, flags=None):
	image_out = image_in.copy()
	image_out = cv2.drawKeypoints(image_in, kp, image_out, color, flags)
	return image_out

def BFMatcher(des1, des2):
	bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
	matches = bf.match(des1, des2)
	matches = sorted(matches, key = lambda x:x.distance)
	return matches

def get_templates():
	templates = []

	for i in range(4):
		template = cv2.imread("/home/shadylady/Repositories/IHMCOpenRoboticsSoftware/IHMCPerception/resources/detectAtlas/template" + str(i+1) + ".jpg", 0)
		# template[(template - 127.5)/255.0 < 0] = 0
		templates.append(template)

	return templates

def blur_templates(templates):
	for i in range(len(templates)):
		templates[i] = cv2.blur(templates[i], (5,5))
		templates[i] = cv2.Canny(templates[i], 100, 200)
	return templates

if __name__ == '__main__':

	path = "/home/shadylady/Repositories/IHMCOpenRoboticsSoftware/IHMCPerception/resources/detectAtlas/extractedImages-Video3/"
	image_names = os.listdir(path)

	templates = get_templates()
	blur_templates = blur_templates(templates)

	kp_t1, des_t1 = ORB(blur_templates[0])
	kp_t2, des_t2 = ORB(blur_templates[1])
	kp_t3, des_t3 = ORB(blur_templates[2])
	kp_t4, des_t4 = ORB(blur_templates[3])

	template1_orb = draw_keypoints(blur_templates[0], kp_t1, flags=0)
	template2_orb = draw_keypoints(blur_templates[1], kp_t2, flags=0)
	template3_orb = draw_keypoints(blur_templates[2], kp_t3, flags=0)
	template4_orb = draw_keypoints(blur_templates[3], kp_t4, flags=0)

	cv2.imshow("template1", template1_orb)
	cv2.imshow("template2", template2_orb)
	cv2.imshow("template3", template3_orb)
	cv2.imshow("template4", template4_orb)

	for img in image_names:
		image = cv2.imread(path + img, 0)
		print(image.shape)
		# image[(image - 127.5)/255.0 < 0] = 0
		blur = cv2.blur(image, (5,5))
		blur = cv2.Canny(blur, 100, 200)

		kp1, des1 = ORB(blur)
		image_orb = draw_keypoints(blur, kp1, flags=0)
		cv2.imshow("ORB", image_orb)

		# kp2, des2 = SIFT(blur)
		# image_sift = draw_keypoints(blur, kp2, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
		# cv2.imshow("SIFT", image_sift)

		matches1 = BFMatcher(des_t1, des1)
		matches2 = BFMatcher(des_t2, des1)
		matches3 = BFMatcher(des_t3, des1)
		matches4 = BFMatcher(des_t4, des1)

		keypoint_indexes = []
		for i in range(3):
			keypoint_indexes.append(matches1[i].trainIdx)
			keypoint_indexes.append(matches2[i].trainIdx)
			keypoint_indexes.append(matches3[i].trainIdx)
			keypoint_indexes.append(matches4[i].trainIdx)

		# counter = collections.Counter(keypoints)
		# counter1 = counter.most_common(3)
		# print(counter1)

		new_kp = []
		for index in keypoint_indexes:
			new_kp.append(kp1[index])

		print(new_kp[0].pt)

		image_new = draw_keypoints(blur, new_kp, color=(0,255,0), flags=0)
		cv2.imshow("NEW IMAGE", image_new)

		final1 = blur.copy()
		final2 = blur.copy()
		final3 = blur.copy()
		final4 = blur.copy()

		final1 = cv2.drawMatches(templates[0], kp_t1, blur, kp1, matches1[:10], final1, flags=2)
		final2 = cv2.drawMatches(templates[1], kp_t2, blur, kp1, matches2[:10], final2, flags=2)
		final3 = cv2.drawMatches(templates[2], kp_t3, blur, kp1, matches3[:10], final3, flags=2)
		final4 = cv2.drawMatches(templates[3], kp_t4, blur, kp1, matches4[:10], final4, flags=2)

		cv2.imshow("Final1", final1)
		cv2.imshow("Final2", final2)
		cv2.imshow("Final3", final3)
		cv2.imshow("Final4", final4)

		k = cv2.waitKey(0) & 0xff
		if (k == 27):
			break