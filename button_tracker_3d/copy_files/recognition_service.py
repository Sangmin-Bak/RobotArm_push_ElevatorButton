#!/usr/bin/env python
import cv2
import rospy
import numpy as np
import PIL.Image as Image
import PIL.ImageOps as ImageOps
from button_recognition.msg import recognition
from button_recognition.msg import recog_result
from button_recognition.srv import *
from ocr_rcnn_lib.button_recognition import ButtonRecognizer
from button_recognition_msgs import BoundingBox, BoundingBoxes

class RecognitionService:
  def __init__(self, model):
    self.model = model
    assert isinstance(self.model, ButtonRecognizer)
    self.pub_button = rospy.Publisher('/button_recognition/bounding_boxes', BoundingBoxes, queue_size=10)

  def perform_recognition(self, request):
    image = request.image
    if len(image.data) == 0:
      rospy.logerr('None image received!')
      return recog_serverResponse(None)
    start = rospy.get_time()
    np_arr = np.fromstring(image.data, np.uint8)

    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    if image_np.shape != (480, 640):
      img_pil = Image.fromarray(image_np)
      img_thumbnail = img_pil.thumbnail((640, 480), Image.ANTIALIAS)
      delta_w, delta_h= 640 - img_pil.size[0], 480 - img_pil.size[1]
      padding = (delta_w // 2, delta_h // 2, delta_w - (delta_w // 2), delta_h - (delta_h // 2))
      new_im = ImageOps.expand(img_pil, padding)
      image_np = np.copy(np.asarray(new_im))

    boundingboxes = BoundingBoxes()
    recognitions = self.model.predict(image_np)
    recog_resp = recog_result()
    for item in recognitions:
      boundingbox = BoundingBox()
      sample = recognition()
      sample.y_min = int(item[0][0] * image_np.shape[0])
      sample.x_min = int(item[0][1] * image_np.shape[1])
      sample.y_max = int(item[0][2] * image_np.shape[0])
      sample.x_max = int(item[0][3] * image_np.shape[1])
      
      boundingbox.xmin = sample.x_min
      boundingbox.ymin = sample.y_min
      boundingbox.xmax = sample.x_max
      boundingbox.ymax = sample.y_max

      sample.score = item[1] # float
      sample.text = item[2]
      sample.belief = item[3]

      boundingbox.id = 1
      boundingbox.Class = sample.text
      boundingbox.probability = sample.belief

      recog_resp.data.append(sample)
      boundingboxes.bounding_boxes.append(boundingbox)

    boundingboxes.header.stamp = rospy.Time.now()
    boundingboxes.header.frame_id = "detection"

    self.pub_button.publish(boundingboxes)
    end = rospy.get_time()
    rospy.loginfo('Recognition finished: {} buttons are detected using {} seconds!'.format(
      len(recognitions), end-start))
    return recog_serverResponse(recog_resp)


def button_recognition_server():
  rospy.init_node('button_recognition_server', anonymous=True)
  model = ButtonRecognizer(use_optimized=True)
  recognizer = RecognitionService(model)
  service = rospy.Service('recognition_service',
                          recog_server,
                          recognizer.perform_recognition)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    recognizer.model.clear_session()
    rospy.logdebug('Shutting down ROS button recognition module!')
  cv2.destroyAllWindows()


if __name__ == '__main__':
  button_recognition_server()
