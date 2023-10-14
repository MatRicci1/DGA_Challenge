#!/usr/bin/env python

#################################################################################################################
# Node that identifies people
# Draws rectangles over each person in the image obtained by the front camera of the drone
# Publishes a topic with the horizontal distance to the centre of the image from the rectangle with the greater
# height  
#################################################################################################################

import rospy
import cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg    import Float64

############################################
### Functions
############################################

### Find Best Window for specific label
def Best_Window(img, label):
    indexes = np.argwhere(img==label)
    idx = indexes[:,0]
    idy = indexes[:,1]
    r, c, h, w = [0,0,0,0]

    if idx.shape[0] > 0 and idy.shape[0] > 0: 
        idx_max = np.max(idx) + 1
        idx_min = np.min(idx) - 1

        idy_max = np.max(idy) + 1
        idy_min = np.min(idy) - 1

        r = idy_min
        c = idx_min
        h = idy_max - idy_min
        w = idx_max - idx_min

    return r, c, h, w

### Create track window over image
def track_display(img_0, window):
    img = img_0.copy()
    r, c, h, w = window
    frame_tracked = cv2.rectangle(img, (r,c), (r+h,c+w), (255,0,0) ,2)

    return frame_tracked

###  Shrink image
def pyram_Down(img, n):
    i = 0
    img_D = img.copy()
    while(i<n):
        img_D = cv2.pyrDown(img_D)
        i = i + 1
    return img_D

###  Expand image
def pyram_Up(img, n):
    i = 0
    img_U = img.copy()
    while(i<n):
        img_U = cv2.pyrUp(img_U)
        i = i + 1
    return img_U

### Show Image
def img_show(name,img):
    cv2.imshow(name,normalize(img))

### Normalize 
def normalize(img):
    img = img/img.max().max()

    return img

### Get pixels of specific color
def get_color(img, bond):
    ### Color Bondaries
    lower = np.array(bond[0], dtype = "uint8")
    upper = np.array(bond[1], dtype = "uint8")

    ### Get color
    mask = cv2.inRange(img, lower, upper)
    Selected_color = cv2.bitwise_and(img, img, mask = mask)

    return Selected_color

### Get contour of elements
def get_contour(gray_img, real_img):
    ret, thresh = cv2.threshold(gray_scale,0,255,cv2.THRESH_BINARY)

    ### Closing holes 
    kernel = np.ones((3,3),np.uint8)
    opening = cv2.morphologyEx(thresh,cv2.MORPH_CLOSE,kernel, iterations = 3)

    ### Sure background area
    sure_bg = cv2.dilate(opening,kernel,iterations=4)

    ### Finding sure foreground area
    dist_transform = cv2.distanceTransform(opening,cv2.DIST_L2,5)
    ret, sure_fg = cv2.threshold(dist_transform,0.04*dist_transform.max(),255,0)

    ### Finding unknown region
    sure_fg = np.uint8(sure_fg)
    unknown = cv2.subtract(sure_bg,sure_fg)

    ### Marker labelling
    num_labels, labels_im = cv2.connectedComponents(sure_fg)
    
    ### Add one to all labels so that sure background is not 0, but 1
    labels_im = labels_im + 1
    ### Now, mark the region of unknown with zero
    labels_im[unknown==255] = 0
    ### Get contour
    contour = cv2.watershed(real_img,labels_im)
    ### New labels
    labels_im = contour.copy()
    ### Set bg and contour to 0
    labels_im[labels_im<2] = 0

    ### Show Process
    # img_show('Tresh', thresh)
    # img_show('Opening', opening)
    # img_show('Sure bg', sure_bg)
    # img_show('Transform', dist_transform)
    # img_show('Sure fg', sure_fg)
    # img_show('Unknown', unknown)
    # img_show('Labels of Contour', labels_im)
    # img_show('Contour', contour)

    return contour, num_labels, labels_im

### Show image of different components
def imshow_components(labels):

    ### Map component labels to hue val
    label_hue = np.uint8(179*labels/np.max(labels))
    blank_ch = 255*np.ones_like(label_hue)
    labeled_img = cv2.merge([label_hue, blank_ch, blank_ch])

    ### cvt to BGR for display
    labeled_img = cv2.cvtColor(labeled_img, cv2.COLOR_HSV2BGR)

    ### set bg label to black
    labeled_img[label_hue==0] = 0
    
    return labeled_img

class drone_camera:
    def __init__(self, drone_N):
        assert int(drone_N) in {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}
        rospy.init_node("drone{}_camera_color".format(drone_N), anonymous=False)
        self.track_flag = False
        self.default_pose_flag = True
        self.tracked = False
        self.x_tracking = 320/2
        self.y_tracking = 240/2

        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/drone{}/front_cam/camera/image'.format(drone_N), Image, self.image_callback)

    def image_callback(self,msg):

        color_boundaries = [[0, 100, 0], [10, 255, 10]] # Green

        # BEGIN BRIDGE
        # 320x240 Image
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        height, width = image.shape[:2]
        track = image.copy()
        image_contour = image.copy()
        ### Get Color
        Selected_color = get_color(image, color_boundaries)

        ### Get Grayscale from color
        gray_scale = cv2.cvtColor(Selected_color, cv2.COLOR_BGR2GRAY)

        A, A_max = 0, 0
        if not cv2.countNonZero(gray_scale) == 0:
            self.tracked = True  

            ### Get Contour and labels
            contour, num_labels, labels_im = get_contour(gray_scale, image)

            ### Get Labeled image
            labeled_img = imshow_components(labels_im)

            ### Finde windows for each element
            i = 2
            while(i<=num_labels):
                ### Track_window : x left top corner, y left top corner, width, height
                track_window = Best_Window(labels_im, i)
                track_w_list = list(track_window)
                A = t[2]*t[3]
                if A == 0:
                    A = t[3]
                if A > A_max:
                    A_max = A
                    self.x_tracking, self.y_tracking = track_w_list[0] + track_w_list[2]/2, track_w_list[1] + track_w_list[3]/2
                track = track_display(track, track_window)
                i = i+1
            
            ### Overlap contour over image
            image_contour[contour  == -1] = [255,0,0]

            # ### Show Image Process
            #img_show('Image Contour', image_contour)
            #img_show('Labeled Image',labeled_img)
                
        # ### Show Image Process
        #img_show('Frame', image)
        #img_show('Selected color', Selected_color)
        #img_show('Gray Scale', gray_scale)
        #img_show('Track', track)

        ### Publishes horizontal distance in pixels from the centre of the biggest window to the centre of the image
        window = rospy.Publisher('Window', Float64, queue_size=1, latch= True)
        window.publish(self.xp-width/2)

        cv2.namedWindow("drone{}_camera_window".format(drone_N), 1)

        cv2.imshow("drone{}_camera_window".format(drone_N), track)
        cv2.waitKey(1)


if __name__ == '__main__':
    print("Let's view your drone camera")
    drone_N = input("Select a drone camera to view. (Options 1, 2, 3, 4, 5, 6, 7, 8, 9, 10): ")
    follower = drone_camera(drone_N)
    rospy.spin()


