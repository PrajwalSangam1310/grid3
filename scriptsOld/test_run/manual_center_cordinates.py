# importing the module
import cv2
import json

point_list = []
# function to display the coordinates of
# of the points clicked on the image
def click_event(event, x, y, flags, params):
    global point_list
	# checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:

        # displaying the coordinates
        # on the Shell
        print(x, ' ', y)    
        # displaying the coordinates
        # on the image window
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img, str(x) + ',' +str(y), (x,y), font,1, (255, 0, 0), 2)
        point_list += [(x,y)]
        cv2.imshow('image', img)

	# checking for right mouse clicks	
    if False:
    	# displaying the coordinates
    	# on the Shell
    	print(x, ' ', y)
    	# displaying the coordinates
    	# on the image window
    	font = cv2.FONT_HERSHEY_SIMPLEX
    	b = img[y, x, 0]
    	g = img[y, x, 1]
    	r = img[y, x, 2]
    	cv2.putText(img, str(b) + ',' +
    				str(g) + ',' + str(r),
    				(x,y), font, 1,
    				(255, 255, 0), 2)
    	cv2.imshow('image', img)

# driver function
if __name__=="__main__":

    # reading the image
    image_path = r"D:\Robotics\FlipkartD2C\grid 3.0\ohm_scripts\bot_det\cam_feed2.jpeg"
    img = cv2.imread(image_path)
    # displaying the image
    cv2.imshow('image', img)
    # setting mouse hadler for the image
    # and calling the click_event() function
    cv2.setMouseCallback('image', click_event)
    # wait for a key to be pressed to exit
    cv2.waitKey(0)
    # close the window
    print(point_list)
    with open("trajectory.txt","w") as fp:
        json.dump(point_list, fp)
    with open("trajectory.txt","r") as fp:
        x = json.load(fp)

    cv2.destroyAllWindows()
    print("read data is:",x)
