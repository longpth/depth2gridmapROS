#!/usr/bin/python
"""
Copyright (c) 2012,
Systems, Robotics and Vision Group
University of the Balearican Islands
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Systems, Robotics and Vision Group, University of
      the Balearican Islands nor the names of its contributors may be used to
      endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""


PKG = 'depth2gridmap' # this package name

import roslib; roslib.load_manifest(PKG)
import rospy
import sensor_msgs.msg
import cv_bridge
import glob
import cv2
import numpy as np
from numpy import genfromtxt
import math
import tf
import os
import datetime
import time
import roslib; roslib.load_manifest(PKG)
import yaml
import sensor_msgs.msg

fake_green = False

timestamp = time.strftime("%Y%m%d_%H%M%S", time.gmtime())

log = open('/mnt/ubuntu_18.04/home/long/work/4.ROS/catkin_ws/src/depth2gridmap/transform_{}.log'.format(timestamp),'w')

def load_poses(pose_file_arg):
    """Load poses (T_w_cam0) from file."""
    pose_file = os.path.join(pose_file_arg)

    # Read and parse the poses
    poses = []
    try:
        with open(pose_file, 'r') as f:
            lines = f.readlines()

            for line in lines:
                T_w_cam0 = np.fromstring(line, dtype=float, sep=' ')
                T_w_cam0 = T_w_cam0.reshape(3, 4)
                T_w_cam0 = np.vstack((T_w_cam0, [0, 0, 0, 1]))
                poses.append(T_w_cam0)

    except FileNotFoundError:
        print('Poses are not avaialble for sequence ' +
              pose_file_arg)

    return poses

def collect_image_files(image_dir,file_pattern):
  images = glob.glob(image_dir + '/' + file_pattern)
  images.sort()
  #images = images[:1101]
  #images = images[1102:2202]
  # images = images[2203:]
  return images

def parse_yaml(filename):
  stream = file(filename, 'r')
  calib_data = yaml.load(stream)
  print(calib_data)
  cam_info = sensor_msgs.msg.CameraInfo()
  cam_info.width = calib_data['image_width']
  cam_info.height = calib_data['image_height']
  cam_info.header.frame_id = calib_data['camera_name']
  cam_info.K = calib_data['camera_matrix']['data']
  # cam_info.D = calib_data['distortion_coefficients']['data']
  # cam_info.R = calib_data['rectification_matrix']['data']
  # cam_info.P = calib_data['projection_matrix']['data']
  # cam_info.distortion_model = calib_data['distortion_model']
  return cam_info

def playback_images(image_dir,file_pattern,camera_info_file,pose_file,publish_rate):
  poses = []
  if camera_info_file != "":
    print("Read camera info .................................")
    cam_info = parse_yaml(camera_info_file)
    publish_cam_info = True
    print("Read camera info done ******************************************")
  else:
    publish_cam_info = False
  if pose_file != "":
    publish_poses = True
    print(pose_file)
    poses = load_poses(pose_file)
  else:
    publish_poses = False
  image_files = collect_image_files(image_dir+"/rgb","*.png")
  depth_files = collect_image_files(image_dir+"/depth",file_pattern)
  rospy.loginfo('Found %i images.',len(image_files))
  bridge = cv_bridge.CvBridge()
  rate = rospy.Rate(publish_rate)
  image_publisher = rospy.Publisher('/camera/rgb/image_color', sensor_msgs.msg.Image, queue_size = 5)
  # depth_publisher = rospy.Publisher('/camera/depth_registered/sw_registered/image_rect_raw', sensor_msgs.msg.Image, queue_size = 5)
  depth_publisher = rospy.Publisher('/camera/depth/image_depth', sensor_msgs.msg.Image, queue_size = 5)
  if publish_cam_info:
    cam_info_publisher = rospy.Publisher('/camera/rgb/camera_info', sensor_msgs.msg.CameraInfo, queue_size = 5)
  if publish_poses:
    tf_pose_publisher = tf.TransformBroadcaster()
  rospy.loginfo('Starting playback.')

  for timestamp_idx in range(0,len(image_files)):
    if rospy.is_shutdown():
      break
    now = rospy.Time.now()
    print('[Image publisher] {}'.format(now))
    image = cv2.imread(image_files[timestamp_idx])
    depth = cv2.imread(depth_files[timestamp_idx], -cv2.IMREAD_ANYDEPTH)
    if fake_green:
      image[:,:,0] = 0
      image[:,:,2] = 0
    image_msg = bridge.cv2_to_imgmsg(np.asarray(image[:,:]), encoding='bgr8')
    image_msg.header.stamp = now
    image_msg.header.frame_id = "/camera_color"

    depth_msg = bridge.cv2_to_imgmsg(np.asarray(depth[:,:]), encoding='mono16')
    depth_msg.header.stamp = now
    depth_msg.header.frame_id = "/camera_depth"

    depth_publisher.publish(depth_msg)
    image_publisher.publish(image_msg)
    if publish_cam_info:
      cam_info.header.stamp = now
      cam_info.header.frame_id = "/openni_rgb_optical_frame"
      cam_info_publisher.publish(cam_info)
    if publish_poses:
      trans3 = tf.transformations.translation_from_matrix(poses[timestamp_idx])
      log.write(str(trans3))
      rot3 = tf.transformations.quaternion_from_matrix(poses[timestamp_idx])
      rot3_debug_x, rot3_debug_y, rot3_debug_z = tf.transformations.euler_from_matrix(poses[timestamp_idx],'rxyz')
      # print('{} {} {}'.format(rot3_debug_x*180/3.14,rot3_debug_y*180/3.14,rot3_debug_z*180/3.14))
      log.write('{} {} {}\n'.format(rot3_debug_x*180/3.14,rot3_debug_y*180/3.14,rot3_debug_z*180/3.14))
      # rot3 = [0,0,0,1] # stop rotation
      # print(poses[timestamp_idx])
      # print(trans3)
      # print(rot3)
      tf_pose_publisher.sendTransform( 
                                       trans3,
                                       rot3,
                                       now,
                                       'cam',
                                       'world'
                                      )
      # print img_name + ' =? ' + str(poses[idx, 0]) + " pose (" + str(poses[idx, 1]) + ", " + str(poses[idx, 2]) + ", " + str(poses[idx, 3]) + ") orientation (" + str(poses[idx, 4]) + ", " + str(poses[idx, 5]) + ", " + str(poses[idx, 6]) + ")"
    rate.sleep()
    # cv2.imshow("origin",image)
    # cv2.imshow("depth",depth)
    # key = cv2.waitKey(0)
    # if key == 27:
    #   break
  rospy.loginfo('No more images left. Stopping.')

if __name__ == "__main__":
  rospy.init_node('image_sequence_publisher')
  try:
    image_dir = rospy.get_param("~image_dir")
    file_pattern = rospy.get_param("~file_pattern")
    camera_info_file = rospy.get_param("~camera_info_file", "")
    pose_file = rospy.get_param("~pose_file", "")
    frequency = rospy.get_param("~frequency", 10)
    fake_green = rospy.get_param("~fake_green", False)
    playback_images(image_dir, file_pattern, camera_info_file, pose_file, frequency)
  except KeyError as e:
    rospy.logerr('Required parameter missing: %s', e)
  except Exception, e:
    import traceback
    traceback.print_exc()
  log.close()

