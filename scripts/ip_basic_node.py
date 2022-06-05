#!/usr/bin/env python
import glob
import os
import sys
import time

import cv2
import numpy as np
import png

from ip_basic import depth_map_utils
from ip_basic import vis_utils

import rospy
from sensor_msgs.msg import Image

class depth_completer:
    def __init__(self):
        mode = rospy.get_param(~mode, 'fast_gaussian')
        if mode == 'fast_gaussian':
            self.fill_type = 'fast'
            self.extrapolate = True
            self.blur_type = 'gaussian'
        elif mode == 'fast_bilateral':
            self.fill_type = 'fast'
            self.extrapolate = True
            self.blur_type = 'gaussian'
        elif mode == 'multiscale':
            self.fill_type = 'fast'
            self.extrapolate = True
            self.blur_type = 'gaussian'
        else:
            print("Invalid mode.")
            return       

        self.sub = rospy.Subscriber("input", Image, self.complete_depth)
        self.pub = rospy.Publisher("output", Image, queue_size=1)
        
        self.bridge = CvBridge()

    def complete_depth(self, img_msg):
        fill_type = self.fill_type
        extrapolate = self.extrapolate
        blur_type = self.blur_type

        ## Load depth projections from uint16 image
        try:
            depth_image = self.bridge.imgmsg_to_cv2(img_msg, cv2.IMREAD_ANYDEPTH)
        except CvBridgeError as e:
            # print(e)
            return
        projected_depths = np.float32(depth_image / 256.0)

        ## Fill in
        if fill_type == 'fast':
            final_depths = depth_map_utils.fill_in_fast(
                projected_depths, extrapolate=extrapolate, blur_type=blur_type)
        elif fill_type == 'multiscale':
            final_depths, process_dict = depth_map_utils.fill_in_multiscale(
                projected_depths, extrapolate=extrapolate, blur_type=blur_type,
                show_process=False)
        else:
            raise ValueError('Invalid fill_type {}'.format(fill_type))

def main(args):
    dp = depth_completer()
    rospy.init_node("ip_basic_node", anonymous=True)
    print("Initialized.")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down.")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)





# def ip_basic():
#     sub = 
#     pub = rospy.Publisher('output', Image, queue_size=1)
#     rospy.init_node('ip_basic_node', anonymous=True)

#     while not rospy.is_shutdown():

# def complete_depth(img_msg, fill_type='fast', extrapolate=True, blur_type='gaussian'):

    ## Fast fill with Gaussian blur @90Hz (paper result)
    # fill_type = 'fast'
    # extrapolate = True
    # blur_type = 'gaussian'

    ## Fast Fill with bilateral blur, no extrapolation @87Hz (recommended)
    # fill_type = 'fast'
    # extrapolate = False
    # blur_type = 'bilateral'

    ## Multi-scale dilations with extra noise removal, no extrapolation @ 30Hz
    # fill_type = 'multiscale'
    # extrapolate = False
    # blur_type = 'bilateral'

    # ## Load depth projections from uint16 image
    # depth_image = bridge.imgmsg_to_cv2(img_msg, cv2.IMREAD_ANYDEPTH)
    # projected_depths = np.float32(depth_image / 256.0)

    # ## Fill in
    # start_fill_time = time.time()
    # if fill_type == 'fast':
    #     final_depths = depth_map_utils.fill_in_fast(
    #         projected_depths, extrapolate=extrapolate, blur_type=blur_type)
    # elif fill_type == 'multiscale':
    #     final_depths, process_dict = depth_map_utils.fill_in_multiscale(
    #         projected_depths, extrapolate=extrapolate, blur_type=blur_type,
    #         show_process=false)
    # else:
    #     raise ValueError('Invalid fill_type {}'.format(fill_type))

    ## Display images from process_dict
    # if fill_type == 'multiscale':
    #     img_size = (570, 165)

    #     x_start = 80
    #     y_start = 50
    #     x_offset = img_size[0]
    #     y_offset = img_size[1]
    #     x_padding = 0
    #     y_padding = 28

    #     img_x = x_start
    #     img_y = y_start
    #     max_x = 1900

    #     row_idx = 0
    #     for key, value in process_dict.items():

    #         image_jet = cv2.applyColorMap(
    #             np.uint8(value / np.amax(value) * 255),
    #             cv2.COLORMAP_JET)
    #         vis_utils.cv2_show_image(
    #             key, image_jet,
    #             img_size, (img_x, img_y))

    #         img_x += x_offset + x_padding
    #         if (img_x + x_offset + x_padding) > max_x:
    #             img_x = x_start
    #             row_idx += 1
    #         img_y = y_start + row_idx * (y_offset + y_padding)