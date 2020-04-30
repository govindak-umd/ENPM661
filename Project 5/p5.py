# -*- coding: utf-8 -*-
"""
Created on Fri Apr 24 18:24:24 2020

@author: nsraj
"""
import cv2
import numpy as np
import glob
import matplotlib.pyplot as plt
from ReadCameraModel import *
from UndistortImage import *
import random
from skimage.measure import ransac

fx , fy , cx , cy , camera_image , LUT = ReadCameraModel ( 'model/')

K = np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1])
K = np.reshape(K,(3,3))
print(K)
images = np.load('image_list.npy')


# Initiate SIFT detector
orb = cv2.ORB_create()
img1 = images[300]
print('shape of img1 is :', img1.shape)
img1_orig = img1
img1 = cv2.cvtColor(img1,cv2.COLOR_RGB2GRAY)
img2 = images [301]
print('shape of img2 is :', img2.shape)
img2_orig = img2
img2 = cv2.cvtColor(img2,cv2.COLOR_RGB2GRAY)

# find the keypoints and descriptors with SIFT
kp1, des1 = orb.detectAndCompute(img1,None)
kp2, des2 = orb.detectAndCompute(img2,None)

# FLANN parameters
FLANN_INDEX_KDTREE = 0
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks=50)   # or pass empty dictionary

flann = cv2.FlannBasedMatcher(index_params,search_params)

des1 = np.float32(des1)
des2 = np.float32(des2)

matches = flann.knnMatch(des1,des2,k=2)

# Need to draw only good matches, so create a mask
matchesMask = [[0,0] for i in range(len(matches))]


features_1 = []
features_2 = []
# ratio test as per Lowe's paper
for i,(m,n) in enumerate(matches):
    if m.distance < 0.7*n.distance:
        matchesMask[i]=[1,0]
        features_1.append(kp1[m.queryIdx].pt)
        features_2.append(kp2[m.trainIdx].pt)

draw_params = dict(matchColor = (0,255,0),
                   singlePointColor = (255,0,0),
                   matchesMask = matchesMask,
                   flags = 0)

# img3 = cv2.drawMatchesKnn(img1,kp1,img2,kp2,matches,None,**draw_params)
# plt.imshow(img3,)
# plt.show()

random_8_feat_1 = []
random_8_feat_2 = []


def fundamental_matrix(feat_1,feat_2):

    A = np.zeros((8,9))

    for k in range(len(feat_1)):
        x1 = feat_1[k][0]
        x2 = feat_2[k][0]
        y1 = feat_1[k][1]
        y2 = feat_2[k][1]
        A[k] = np.array([x1*x2,x2*y1,x2,y2*x1,y1*y2,y2,x1,y1,1])
    U,S,V = np.linalg.svd(A)
    V = np.transpose(V)
    V = V[:,-1]
    F = np.reshape(V,(3,3))
    U_F, S_F, V_F = np.linalg.svd(F)
    S_F[-1] = 0
    S_new = np.zeros((3,3))
    for i in range(3):
        S_new[i,i]=S_F[i]
    F_new = np.matmul(np.matmul(U_F,S_new),V_F)
    F_new = F_new/F_new[2,2]

    #NORMALZIED PART
    all_x_1 = 0
    all_x_2 = 0
    all_y_1 = 0
    all_y_2 = 0

    for point_1 in feat_1:
        all_x_1+=point_1[0]
        all_y_1+=point_1[1]

    for point2 in feat_2:
        all_x_2+=point2[0]
        all_y_2+=point2[1]

    centr_1 = ((all_x_1/len(feat_1)),(all_y_1/len(feat_1)))
    centr_2 = ((all_x_2/len(feat_2)),(all_y_2/len(feat_2)))

    points_feat_1 = []
    points_feat_2 = []

    for point in feat_1:
        x = point[0] - centr_1[0]
        y = point[1] - centr_1[1]
        point_new = (x,y)
        points_feat_1.append(point_new)

    for point in feat_2:
        x = point[0] - centr_2[0]
        y = point[1] - centr_2[1]
        point_new = (x,y)
        points_feat_2.append(point_new)

    den = 0

    for point in points_feat_1:
        den += ((point[0])**-2+ (point[1])**-2)
    den = np.sqrt((1/len(points_feat_1)) * den)

    s = np.sqrt(2)/ den

    den2 = 0

    for point in points_feat_2:
        den2 += ((point[0])**-2+ (point[1])**-2)
    den2 = np.sqrt((1/len(points_feat_2)) * den2)

    s_prime = np.sqrt(2)/ den2

    T1 = np.matmul(np.array([[s,0,0],[0,s,0],[0,0,1]]),np.array([[1,0,-(all_x_1/len(feat_1))],[0,1,-(all_y_1/len(feat_1))],[0,0,1]]))
    T2 = np.matmul(np.array([[s_prime, 0, 0], [0, s_prime, 0], [0, 0, 1]]),
                   np.array([[1, 0, -(all_x_2 / len(feat_2))], [0, 1, -(all_y_2 / len(feat_2))], [0, 0, 1]]))


    F_normalized = np.matmul(np.matmul(T2.T,F_new),T1)

    F_normalized  = F_normalized/F_normalized[2,2]




    return F_normalized

num = len(features_1)
size = 8
p = 0.99
outlier_ratio = 0.5
times = np.log(1 - p) / np.log(1-(1 - outlier_ratio) ** 9)
inlier_present = 0
threshold = 0.05
Best_Fmatrix = np.zeros((3, 3))

for i in range(2355):
    random_8_feat_1 = []
    random_8_feat_2 = []
    for k in range(8):
        rand = random.randint(0,len(features_1)-1)
        random_8_feat_1.append(features_1[rand])
        random_8_feat_2.append(features_2[rand])
    F = fundamental_matrix(random_8_feat_1,random_8_feat_2)
    inlier_count = 0
    for j in range(len(features_1)):
        new_list = np.array([features_1[j][0], features_2[j][1], 1])
        new_list = np.reshape(new_list, (3, 1))
        new_list_2 = np.array([features_1[j][0], features_2[j][1], 1])
        new_list_2 = np.reshape(new_list_2, (1, 3))
        distance = abs(np.matmul(np.matmul(new_list_2,F),new_list))
        if distance<threshold:
            inlier_count+=1
    if inlier_count/num > inlier_present:
        inlier_present = inlier_count / num
        Best_Fmatrix = F
distances = {}

for j in range(len(features_1)):
    new_list = np.array([features_1[j][0], features_2[j][1], 1])
    new_list = np.reshape(new_list, (3, 1))
    new_list_2 = np.array([features_1[j][0], features_2[j][1], 1])
    new_list_2 = np.reshape(new_list_2, (1, 3))
    distances[j] = abs(np.matmul(np.matmul(new_list_2,F),new_list))

distances_sorted = {k: v for k, v in sorted(distances.items(), key=lambda item: item[1])}

distances_list = []
v_list = []
for k,v in distances_sorted.items():
    #print(v[0][0])
    if v[0][0]<0.05: #threshold distance
        distances_list.append(v[0][0])
        v_list.append(k)
print(distances_list)


len_distance_list = len(distances_list)

len_distance_list = min(len_distance_list,30) #30 might have to be changed

inliers_1 = []
inliers_2 = []

for x in range(len_distance_list):
    inliers_1.append(features_1[v_list[x]])
    inliers_2.append(features_2[v_list[x]])
print('F : ', F)
print('inliers_1',inliers_1)
print('inliers_2',inliers_2)

for pts in inliers_1:
    int_x = int(pts[0])
    int_y = int(pts[1])
    pts2 = (int_x,int_y)
    cv2.circle(img1_orig,pts2,5,[0,0,255],1)

for pts in inliers_2:
    int_x = int(pts[0])
    int_y = int(pts[1])
    pts2 = (int_x, int_y)
    cv2.circle(img2_orig,pts2,5,[0,0,255],1)

# cv2.imshow('IMG1',img1_orig)
#
# cv2.imshow('img2',img2_orig)
#
# cv2.waitKey(0)
# cv2.destroyAllWindows()


#Essential Matrix calc

def Essential_Matrix(F,K):
    E = np.matmul(np.matmul(K.T, F), K)
    U,S,V = np.linalg.svd(E)

    S[0]= 1
    S[1] = 1
    S[2] = 0
    S_new = np.zeros((3, 3))
    for i in range(3):
        S_new[i, i] = S[i]

    # if det(np.matmul(U,V.T))==1:
    #     V = V.T
    # elif det(np.matmul(U,V.T))== -1:
    #     V = (-V.T)

    E_new = np.matmul(np.matmul(U, S_new), V)
    E_new = E_new / E_new[2, 2]

    return(E_new)

E = Essential_Matrix(F,K)

print('E : : : > > > ', E)

#DECOMPOSING THE ESSENTIAL MATRIX TO TRANSLATION AND ROTATION MATRIX

U_decompose, S_decompose, V_decompose = np.linalg.svd(E)
W = np.array([0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]).reshape(3, 3)

# new_list = np.array([features_1[5][0] ,features_2[5][1] ,1])
# new_list = np.reshape(new_list,(3,1))
# print(new_list)
# print(new_list.shape)
#
# new_list_2 = np.array([features_1[5][0] ,features_2[5][1] ,1])
# new_list_2 = np.reshape(new_list_2,(1,3))
# print(new_list_2)
# print(new_list_2.shape)
# F=  np.reshape(F , (3,3))
#
#print(np.matmul(np.matmul(new_list_2,F),new_list))