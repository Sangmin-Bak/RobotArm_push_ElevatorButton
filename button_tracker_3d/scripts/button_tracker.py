#!/usr/bin/env python
import os
import cv2
import rospy
import numpy as np
import PIL.Image as Image
import PIL.ImageOps as ImageOps
from button_recognition.srv import *
from sensor_msgs.msg import Image as Image_
from sensor_msgs.msg import CompressedImage
# from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from button_recognition_msgs.msg import BoundingBox, BoundingBoxes
# from button_tracker.msg import BoundingBox, BoundingBoxes
from cv_bridge import CvBridge, CvBridgeError
import time
# import pycuda.driver as cuda
# import pycuda.autoinit
# from pycuda.compiler import SourceModule

from threading import Thread
from multiprocessing import Queue

counter = 0
prevTime = 0

rgb_images = None
frame_ = None
input_q = Queue(5000)

class ButtonTracker:
      
  def __init__(self):
    self.detected_box = None
    self.tracker = None

  def init_tracker(self, image, box_list):
    self.tracker = None
    self.tracker = cv2.MultiTracker_create()
    for box_item in box_list:
      self.tracker.add(cv2.TrackerKCF_create(), image, tuple(box_item))

  @staticmethod
  def call_for_service(image):
    rospy.wait_for_service('recognition_service')
    compressed_image = CompressedImage()
    compressed_image.header.stamp = rospy.Time.now()
    compressed_image.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
    try:
      recognize = rospy.ServiceProxy('recognition_service', recog_server)
      response = recognize(compressed_image)
      if response is None:
        print("None service response!")
      boxes, scores, texts, beliefs = [], [], [], []

      for pred in response.box.data:
        boxes.append([pred.x_min,  pred.y_min, pred.x_max, pred.y_max])
        scores.append(pred.score)
        text = pred.text
        texts.append(text.replace(' ', ''))
        beliefs.append(pred.belief)
      return boxes, scores, texts, beliefs
    except rospy.ServiceException, e:
      print "recognition service failed: {}".format(e)

  @staticmethod
  def visualize_recognitions(frame, box, text):

    # draw bounding boxes
    p1, p2 = (int(box[0]), int(box[1])), (int(box[2]), int(box[3]))
    cv2.rectangle(frame, p1, p2, (50, 220, 100), thickness=2)
    # draw text at a proper location
    btn_width = (box[2] - box[0]) / 2.0
    btn_height = (box[3] - box[1]) / 2.0
    font_size = min(btn_width, btn_height) * 0.6
    text_len = len(text)
    font_pose = int(0.5*(box[0]+box[2]) - 0.5 * text_len * font_size), int(0.5*(box[1]+box[3]) + 0.5 * font_size)
    # font_pose is the bottom_left of the text
    cv2.putText(frame, text, font_pose, cv2.FONT_HERSHEY_SIMPLEX, 0.6, thickness=2, color=(255, 0, 255))

  @staticmethod
  def resize_to_480x680(img):
    if img.shape != (480, 640):
      img_pil = Image.fromarray(img)
      img_thumbnail = img_pil.thumbnail((640, 480), Image.ANTIALIAS)
      delta_w, delta_h= 640 - img_pil.size[0], 480 - img_pil.size[1]
      padding = (delta_w // 2, delta_h // 2, delta_w - (delta_w // 2), delta_h - (delta_h // 2))


class ButtonRead:
    
    def __init__(self):
        self.bridge = CvBridge()
        

        # self.mod = SourceModule("""
        #     __global__ void doublify(float *a)
        #     {
        #         int idx = threadIdx.x + threadIdx.y*4;
        #         a[idx] *= 1;
        #     }
        #     """)

        rospy.Subscriber('/camera/color/image', Image_, self.imageCallback, queue_size=1)

    def imageCallback(self, ros_image):
        global rgb_images
        try:
            color_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            rgb_images = color_image
            input_q.put(rgb_images)
        except CvBridgeError, e:
            print e
            #Convert the depth image to a Numpy array
            color_array = np.array(color_image, dtype=np.float32)

                
    # def cudaImg(self):
    #     img = input_q.get()
    #     img_gpu = cuda.mem_alloc(img.nbytes)
    #     cuda.memcpy_htod(img_gpu, img)

    #     func = self.mod.get_function("doublify")
    #     func(img_gpu, block=(4, 4, 1))

    #     img_doubled = np.empty_like(img)
    #     cuda.memcpy_dtoh(img_doubled, img_gpu)

    #     return img_doubled

    def showImages(self):
        button_tracker = ButtonTracker()
        
        while True:

            # boundingbox = BoundingBox()
            # boundingboxes = BoundingBoxes()

            # frame = self.cudaImg()
            frame = input_q.get()
            # frame = button_tracker.resize_to_480x680(img)
            (boxes, scores, texts, beliefs) = button_tracker.call_for_service(frame)
            button_tracker.init_tracker(frame, boxes)
            
            for box, text in zip(boxes, texts):
                button_tracker.visualize_recognitions(frame, box, text)
               
            self.show_fps(frame)
          
            cv2.imshow('button_recognition', frame)
              
            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                break
            
        cv2.destroyAllWindows()	 
        return 0

    def show_fps(self, frame):
        global curTime
        global prevTime
        curTime = time.time()

        sec = curTime - prevTime
        prevTime = curTime
        fps = 1 / (sec)
        fps_text = "FPS : %0.1f" % fps
        cv2.putText(frame, fps_text, (0, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0))


if __name__ == '__main__':
    
    img_only = rospy.get_param('button_tracker/img_only', False)

    read_btn = ButtonRead()
    rospy.init_node('button_tracker', anonymous=True)
    try:
        show_video = read_btn.showImages()
        if show_video == 0:
            rospy.signal_shutdown ("reason")

        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()	 
        
    
    rospy.loginfo('Process Finished!')
