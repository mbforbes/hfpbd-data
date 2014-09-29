'''Extracts images from ROS bag files, assuming directory structure
found in hfpbd-data repo.

Topic is: /head_mount_kinect/rgb/image_color

NOTE(mbforbes): Run from desktop only; PR2 lacks required python
libraries.
'''

# ROS/OpenCV required
import rosbag
import cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# System builtins.
import glob
import os
import sys


def fcmp(x, y):
    '''
    String comparison function to provide numerical sorting of directory
    contents (e.g. so 2 follows directly after 1, instead of 10).

    Args:
        x (str): Full path of bag file.
        y (str): Full path of another bag file.

    Returns:
        int: As spec'd by sort function.
    '''
    bn1 = int(x.split('/')[-1].split('.')[-2])
    bn2 = int(y.split('/')[-1].split('.')[-2])
    return bn1 - bn2


def process():
    '''
    Finds all bag files in sub-subdirectories of the directory this
    script is run in, then calls convert(...) on each. See convert(...)
    for more details.

    '''
    # Extract all bags from subdirectories.
    d = os.path.dirname(os.path.realpath(__file__)) + '/'
    months = sorted(glob.glob(d + '*'))
    for m in months:
        m += '/'
        days = sorted(glob.glob(m + '*'))
        for d in days:
            d += '/'
            bags = sorted(glob.glob(d + '*.bag'), cmp=fcmp)
            for bag in bags:
                convert(bag)


def convert(filename):
    '''
    Creates directory (if there are images to be saved) for filename and
    save any images that it finds in this bag file from the spec'd
    topic.

    Args:
        filename (str): Full system path of bag file.
    '''
    save_dir = filename.split('.bag')[-2] + '-images/'
    # print 'bag file:        ', filename
    # print 'saving images in:', save_dir

    # Use a CvBridge to convert ROS images to OpenCV images so they can
    # be saved.
    bridge = CvBridge()
    with rosbag.Bag(filename, 'r') as bag:
        idx = 1
        for topic, msg, t in bag.read_messages():
            if topic == "/head_mount_kinect/rgb/image_color":
                # Wait to create image dir until we know we have one.
                if not os.path.exists(save_dir):
                    print 'creating dir:    ', save_dir
                    os.makedirs(save_dir)
                image_name = save_dir + str(idx) + ".jpg"
                if not os.path.exists(image_name):
                    print 'saving:          ', image_name
                    cv_image = bridge.imgmsg_to_cv(msg, "bgr8")
                    cv.SaveImage(image_name, cv_image)
                else:
                    pass
                    # print 'skipping:        ', image_name
                idx += 1

if __name__ == '__main__':
    process()
