import pickle
import numpy as np
from PIL import Image

import keras
from keras.applications.imagenet_utils import preprocess_input
from keras.preprocessing import image
from scipy.misc import imresize

from ssd import SSD300
from ssd_utils import BBoxUtility

from styx_msgs.msg import TrafficLight


class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        NUM_CLASSES = 3 + 1
        input_shape = (300, 300, 3)

        # "prior boxes" in the paper
        priors = pickle.load(open('/home/student/.ros/prior_boxes_ssd300.pkl', 'rb'))
        self.bbox_util = BBoxUtility(NUM_CLASSES, priors)
        self.model = SSD300(input_shape, num_classes=NUM_CLASSES)
        self.model.load_weights('/home/student/.ros/weights.180314.hdf5', by_name=True)

    def get_classification(self, img):
        """Determines the color of the traffic light in the image

        Args:
            img (cv::Mat): image containing the traffic light
            assumed 3D numpy.array (800, 600, 3)
            bgr8: CV_8UC3, color image with blue-green-red color order

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        pilImg = Image.fromarray(np.uint8(img)).resize((300, 300))
        img = np.array(pilImg)
        img = image.img_to_array(img)
        inputs = np.reshape(img, (1, 300, 300, 3))  # 'inputs' expects this size

        inputs = preprocess_input(np.array(inputs))
        preds = self.model.predict(inputs, batch_size=1, verbose=0)
        results = self.bbox_util.detection_out(preds)

        det_label = results[0][:, 0]
        det_conf = results[0][:, 1]

        # Get detections with confidence >= 0.8
        top_indices = [j for j, conf in enumerate(det_conf) if conf >= 0.8]
        top_conf = det_conf[top_indices]
        top_label_indices = det_label[top_indices].tolist()
        if top_label_indices == []:
            return TrafficLight.UNKNOWN, 0, 0, 0, 0, 0
        
        # assume only one signal detected
        label = int(top_label_indices[0])
        if label == 0:
            return TrafficLight.UNKNOWN
        elif label == 1:
            return TrafficLight.RED
        elif label == 2:
            return TrafficLight.YELLOW
        elif label == 3:
            return TrafficLight.GREEN
        else:
            return TrafficLight.UNKNOWN
