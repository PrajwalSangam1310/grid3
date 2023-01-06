import cv2


def size_mod(img,times):
    h,w = img.shape[:2]
    dim = (int(w*times), int(h*times))
    return cv2.resize(img, dim)

if __name__=='__main__':
    vdo = cv2.VideoCapture(1)
    vdo.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    vdo.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    key = 0
    while key != 27:
        print("in loop")
        img,ret = vdo.read()
        if ret is False:
            print("ret is false breaking")
            break
        new_img = cv2.resize(img, (500,500)) 
        cv2.imshow("plot_img",img)
        key = cv2.waitKey(10)
    print("test completed")
