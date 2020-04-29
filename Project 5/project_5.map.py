import numpy as np
import cv2


def IsOutsideMap(x,y):
	clerance = 0
	radius = 0
	sd = clerance + radius
	ob1 = (x >= 15 - sd) and (y <= 240 -sd) and (x <= 665 +sd) and (y >= 15 + sd)
	if (ob1):
		return 0
	else:
		if (y < 1015 + sd) and ( x < 1115 - sd ) and (y > 15 - sd) and ( x > 15 + sd):
			return 1
		else:
			return 0

def IsObestacle(x,y):
	clerance = 0
	radius = 0
	sd = clerance + radius
	circle = (x - 865)**2 + (y - 315)**2 - (100 + sd)**2 <= 0
	wall1 = x >= 365 - sd and y <= 1015 - sd and x <= 380 + sd and y >= 840 + sd
	wall2 = x >= 315 - sd and y >= 240 + sd and x <= 330 + sd and y <= 500 - sd
	wall3 = x >= 315 - sd and y >= 500 + sd and x <= 665 + sd and y <= 515 - sd
	wall4 = x >= 725 - sd and y >= 690 + sd and x <= 740 + sd and y <= 1015 - sd
	if(circle or wall1 or wall2 or wall3 or wall4):
		return 1
	else:
		return 0

def Animate(start):
	image = np.zeros((1030, 1130, 3), dtype=np.uint8)
	
	for i in range(0,1130):
		for j in range(0,1030):
			if(IsOutsideMap(i,j) == 1):
				image[j][i] = (0,128,255)
			if(IsObestacle(i,j) == 1):
				image[j][i] = (0,0,0)
	image = cv2.circle(image, (start[0],start[1]), 5, (255,255,255), 5)
	height = int(image.shape[0] * 0.5)
	width = int(image.shape[1] * 0.5)
	dim = (width,height)
	resized = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
	cv2.imshow('result', resized)			
	cv2.waitKey(0)
	cv2.destroyAllWindows()


def main():
	startRow = int(input("Enter the x coordinate for start node (between 0 and 1130) : "))
	startRow = startRow 
	startCol = int(input("Enter the y coordinate for start node (between 0 and 1030) : "))
	startCol = 1030 - (startCol)
	start = (startRow,startCol)
	print(start)
	ans = IsOutsideMap(start[0],start[1])
	if(ans == 1):
		print("valid input")
	else:
		print("IsOutsideMap")
	ans2 = IsObestacle(start[0],start[1])
	if(ans2 == 0):
		print("Good")
	else:
		print("IsObestacle")
	Animate(start)
	# print("done")


if __name__ == '__main__':
	main()

