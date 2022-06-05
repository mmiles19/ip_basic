#!/usr/bin/env python
import glob
import os
import sys
import time

import cv2
import numpy as np
import png

try:
    from ip_basic import depth_map_utils
    from ip_basic import vis_utils
except ImportError as error:
    print(error.__class__.__name__ + ": " + error.message)

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class depth_completer:
    def __init__(self):
        self.fill_type = rospy.get_param("fill_type", 'fast')
        self.extrapolate = rospy.get_param("extrapolate", False)
        self.blur_type = rospy.get_param("blur_type", 'gaussian')

        self.sub = rospy.Subscriber("input", Image, self.complete_depth)
        self.pub = rospy.Publisher("output", Image, queue_size=1)
        
        self.bridge = CvBridge()

        print("Initialized.")

    def complete_depth(self, img_msg):
        fill_type = self.fill_type
        extrapolate = self.extrapolate
        blur_type = self.blur_type

        ## Load depth projections from uint16 image
        try:
            depth_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)
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
            print("Failed to fill.")
            raise ValueError('Invalid fill_type {}'.format(fill_type))
            # return

        depth_image = (final_depths * 256).astype(np.uint16)

        try:
            new_img_msg = self.bridge.cv2_to_imgmsg(depth_image, "16UC1")
            new_img_msg.header = img_msg.header
            self.pub.publish(new_img_msg)
            # print("Published image.")
        except CvBridgeError as e:
            print(e)

def main():
    dp = depth_completer()
    rospy.init_node("ip_basic_node", anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down.")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()





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