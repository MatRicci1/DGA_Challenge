from djitellopy import Tello
import cv2
import time
import numpy as np
# import rospy
# # from geometry_msgs.msg import Twist
# # from hector_uav_msgs.srv import EnableMotors
# from std_msgs.msg    import Float64
# from sensor_msgs.msg import Imu

def empty(a):
    pass

cv2.namedWindow("HSV")
cv2.resizeWindow("HSV",320,240)
cv2.createTrackbar("HUE Min","HSV",30,179,empty)
cv2.createTrackbar("HUE Max","HSV",52,179,empty)
cv2.createTrackbar("SAT Min","HSV",202,255,empty)
cv2.createTrackbar("SAT Max","HSV",255,255,empty)
cv2.createTrackbar("VALUE Min","HSV",42,255,empty)
cv2.createTrackbar("VALUE Max","HSV",127,255,empty)

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
        idx_max = np.max(idx)
        idx_min = np.min(idx)

        idy_max = np.max(idy)
        idy_min = np.min(idy)

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

# ### Get pixels of specific color
def get_color(img, bond):
#     ### Color Bondaries


    imgHsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

    h_min = cv2.getTrackbarPos("HUE Min","HSV")
    h_max = cv2.getTrackbarPos("HUE Max", "HSV")
    s_min = cv2.getTrackbarPos("SAT Min", "HSV")
    s_max = cv2.getTrackbarPos("SAT Max", "HSV")
    v_min = cv2.getTrackbarPos("VALUE Min", "HSV")
    v_max = cv2.getTrackbarPos("VALUE Max", "HSV")

    # h_min = 72
    # h_max = 89
    # s_min = 86
    # s_max = 221
    # v_min = 50
    # v_max = 73

    # h_min = 32
    # h_max = 51
    # s_min = 192
    # s_max = 255
    # v_min = 50
    # v_max = 73

    #print(h_min)

    lower = np.array([h_min,s_min,v_min])
    upper = np.array([h_max,s_max,v_max])
    mask = cv2.inRange(imgHsv,lower,upper)

    ### Get color
    selected_color = cv2.bitwise_and(imgHsv, imgHsv, mask = mask)
    return selected_color
    

### Get contour of elements
def get_contour(gray_img, real_img):
    ret, thresh = cv2.threshold(gray_scale,0,255,cv2.THRESH_BINARY)

    ### Noise removal
    kernel = np.ones((3,3),np.uint8)
    opening = cv2.morphologyEx(thresh,cv2.MORPH_CLOSE,kernel, iterations = 3)

    ### Sure background area
    #sure_bg = cv2.erode(opening,kernel,iterations=1)
    sure_bg = cv2.dilate(opening,kernel,iterations=4)
    #sure_bg = opening.copy()

    ### Finding sure foreground area
    dist_transform = cv2.distanceTransform(opening,cv2.DIST_L2,5)
    ret, sure_fg = cv2.threshold(dist_transform,0.04*dist_transform.max(),255,0)

    ### Finding unknown region
    sure_fg = np.uint8(sure_fg)
    unknown = cv2.subtract(sure_bg,sure_fg)

    ### Marker labelling
    #sure_fg = cv2.bitwise_not(sure_fg)
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
    img_show('Transform', dist_transform)
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


if __name__ == '__main__':

    #rospy.init_node('color_detection')

    ################################
    width = 320 
    height = 240
    startcounter = 1
    ################################


    # Connect to Tello
    me = Tello()
    me.connect()
    time.sleep(1)
    me.for_back_velocity = 0
    me.left_right_velocity = 0
    me.up_down_velocity = 0
    me.yaw_velocity = 0
    me.speed = 0

    print(me.get_battery())

    me.streamoff()
    me.streamon()


    while True:
        ### RGB verde escuro: 29.4 28.6 1.2
        ### RGB verde Claro: 24.3 43.1 4.7

        ### Colors Boundaries
        color_boundaries = [[0, 0, 0], [4.7, 43.1, 24.3]] # Custom2

        # Get the image from Tello
        frame_read = me.get_frame_read()
        myFrame = frame_read.frame
        img = cv2.resize(myFrame,(width,height))
        
        ### Copy images for tracking
        track = img.copy()
        image_contour = img.copy()

        ### Get Color
        Selected_color = get_color(img, color_boundaries)

        ### Get Grayscale from color
        gray_scale = cv2.cvtColor(Selected_color, cv2.COLOR_BGR2GRAY)

        #img_show('Color', Selected_color)
        #img_show('Grey', gray_scale)

        if not cv2.countNonZero(gray_scale) == 0:    
            ### Get Contour and labels
            contour, num_labels, labels_im = get_contour(gray_scale, img)

            #print(labels_im)
            ### Get Labeled image
            labeled_img = imshow_components(labels_im)

            ### Find windows for each element
            i = 2
            while(i<=num_labels):
                track_window = Best_Window(labels_im, i)
                track = track_display(track, track_window)
                i = i+1

            ### Overlap contour over image
            image_contour[contour  == -1] = [255,0,0]

            # ### Show Image Process
            #img_show('Image Contour', image_contour)
            img_show('Labeled Image',labeled_img)

        # ### Show Image Process
        img_show('Frame', img)
        img_show('Selected color', Selected_color)
        img_show('Gray Scale', gray_scale)
        img_show('Track', track)

        #time.sleep(6)
        # To go up in the beginning
        if startcounter == 0:
            me.takeoff()
            time.sleep(6)
            me.move_forward(20)
            time.sleep(6)
            me.rotate_clockwise(180)
            me.move_forward(20)
            time.sleep(6)
            me.rotate_clockwise(180)
            me.land()
            startcounter = 1

        # # Send velocities values to Tello
        # if me.send_rc_control:
        #     me.send_rc_control(me.left_right_velocity,me.for_back_velocity,me.up_down_velocity,me.yaw_velocity)

        # Wait for the "Q" button to stop
        if cv2.waitKey(1) & 0xFF ==ord('q'):
            me.land()
            break

