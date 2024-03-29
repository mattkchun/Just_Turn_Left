import cv2
#import imutils
import numpy as np
import pdb

def cd_color_segmentation(img, show_image=False, color="r"):
    """
	Implement the cone detection using color segmentation algorithm
	    Input:
	    img: np.3darray; the input image with a cone to be detected
	Return:
	    bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
		    (x1, y1) is the bottom left of the bbox and (x2, y2) is the top right of the bbox
    """
    # convert from rgb to hsv color space (it might be BGR)
    new_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    new_img = new_img[new_img.shape[0]/2:5*new_img.shape[0]/6]
#    new_img = new_img[new_img.shape[1]/4:3*new_img.shape[1]/4]

    # define lower and upper bound of image values
    if color=="r":
        low_range  = np.array( [ 0, 100, 150] ) #105, 200, 140
        high_range = np.array( [ 8, 250, 255] ) #130, 255, 255
    elif color=="b":
        low_range  = np.array( [ 100, 110, 150] )
        high_range = np.array( [ 115, 255, 255] )
    elif color=="y":
        low_range  = np.array( [ 20, 150, 150] )
        high_range = np.array( [ 30, 255, 255] )
    else:
        low_range  = np.array( [ 0, 0, 0] )
        high_range = np.array( [ 255, 255, 255] )

    # create mask for image with overlapping values
    mask = cv2.inRange(new_img, low_range, high_range)
#    mask2 = cv2.inRange(new_img, low_range_2, high_range_2)
#    mask = mask1+mask2

    # filter the image with bitwise and
    filtered = cv2.bitwise_and(new_img, new_img, mask=mask)

    # find the contours in the image
    _, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    x1, y1, x2, y2 = 0, 0, 0, 0
    if len(contours) != 0:
	# find contour with max area, which is most likely the cone
        # Solution note: max uses an anonymous function in this case, we can also use a loop...
        contours_max = max(contours, key = cv2.contourArea)

	# Find bounding box coordinates
        x1, y1, x2, y2 = cv2.boundingRect(contours_max)

	# Draw the bounding rectangle
        cv2.rectangle(img, (x1, y1), (x1 + x2, y1 + y2), (0, 255, 0), 2)

    if show_image:
        #cv2.imshow("Color segmentation", img)
        key = cv2.waitKey()
#        if key == 'q':
            #cv2.destroyAllWindows()

    # Return bounding box
    return ((x1, y1), (x1 + x2, y1 + y2)), img
