# ===============================================================================
# Copyright 2020 Hochschule Karlsruhe - Technik und Wirtschaft
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# ===============================================================================
# Authors:  Anna-Lena Marmein, Magdalena Kugler, Yannick Rauch, Patrick Rebling
# ===============================================================================

# ROS
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
# Utilities
import pathlib
import time
from threading import Lock
# Detectors
from models.mobile_net.detector import DetectorAPI
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog, DatasetCatalog

class DetectionDriver(Node):

    def __init__(self):

        # Initialize class as node
        super().__init__('detection_driver')
        # Setting error flag to false
        self.error = False

        # ROS parameter initialization
        self.declare_parameter('frequency')
        self.declare_parameter('image_topic')
        self.declare_parameter('det_topic')
        self.declare_parameter('flag_topic')
        self.declare_parameter('detection_threshold')
        self.declare_parameter('visualize_detection')
        self.declare_parameter('detector')
        self.declare_parameter('mn_model_path')
        self.declare_parameter('d2_model_path')
        self.declare_parameter('device')

        # ROS node initialization
        self.timer_period = 1 / self.get_parameter('frequency').get_parameter_value().double_value
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.image_subscriber = self.create_subscription(Image, 
                    self.get_parameter('image_topic').get_parameter_value().string_value,
                    self.image_callback, 1)
        self.det_img_publisher = self.create_publisher(Image, 
                    self.get_parameter('det_topic').get_parameter_value().string_value, 10)
        self.det_flag_publisher = self.create_publisher(Bool, 
                    self.get_parameter('flag_topic').get_parameter_value().string_value, 10)

        self.get_logger().info('Node Initialized.')

        # CV Bridge for image converting and publishing
        self.bridge = CvBridge()

        # Instantiate neural network and members
        self.in_detection = False
        self.image = None
        self.mutex = Lock()
        self.detector = self.get_parameter('detector').get_parameter_value().string_value
        self.threshold = self.get_parameter('detection_threshold').get_parameter_value().double_value
        self.visualization = self.get_parameter('visualize_detection').get_parameter_value().bool_value
        device = self.get_parameter('device').get_parameter_value().string_value

        # MobileNet
        if self.detector == 'MN':
            modelpath = str(pathlib.Path(__file__).parent.parent.absolute()) + '/' \
                        + self.get_parameter('mn_model_path').get_parameter_value().string_value
            self.nn = DetectorAPI(modelpath, self.threshold, self.visualization)
        # Detectron2
        elif self.detector == 'D2':
            self.cfg = get_cfg()
            # Set model
            modelpath = self.get_parameter('d2_model_path').get_parameter_value().string_value
            self.cfg.merge_from_file(model_zoo.get_config_file(modelpath))
            # set threshold for this model
            self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = self.threshold
            # Set model weights
            self.cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url(modelpath)
            # Set device
            if device == 'cpu':
                self.cfg.MODEL.DEVICE = device
            self.nn = DefaultPredictor(self.cfg)
            self.get_logger().info('Detector \"Detectron2\" Loaded.')
        # Invalid detector code
        else:
            self.get_logger().error('\"' + str(self.detector) \
                + '\" is No Valid Detector. Please Check Config File.')
            self.get_logger().error('Shutdown of Detection Driver!')
            self.error = True

    def image_callback(self, msg):

        # safe incoming image with mutex
        with self.mutex:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image = img

    def timer_callback(self):

        # Get local copy of image with mutex
        with self.mutex:
            img = self.image
            self.image = None

        if img is None:
            return

        msg_flag = Bool()
        if self.detector == 'MN':
            image, boxes, scores, num, duration = self.nn.single_image_det(img)
            msg_flag.data = (sum(i > self.threshold for i in scores) > 0)
        elif self.detector == 'D2':
            start_time = time.time()
            outputs = self.nn(img)
            duration = time.time() - start_time
            msg_flag.data = (0 in outputs['instances'].pred_classes.tolist())
        else:
            return
        self.get_logger().info('Duration of Single Image Inference: ' + str(duration))
        self.det_flag_publisher.publish(msg_flag)

        # Visualization
        if self.get_parameter('visualize_detection').get_parameter_value().bool_value:
            msg_out = Image()
            if self.detector == 'MN':
                msg_out = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            elif self.detector == 'D2':
                v = Visualizer(img[:, :, ::-1], MetadataCatalog.get(self.cfg.DATASETS.TRAIN[0]), scale=1.2)
                out = v.draw_instance_predictions(outputs['instances'].to('cpu'))
                image = out.get_image()
                msg_out = self.bridge.cv2_to_imgmsg(image, encoding='rgb8')
            self.det_img_publisher.publish(msg_out)

def main():

    rclpy.init()
    detection_driver = DetectionDriver()
    if not detection_driver.error:
        rclpy.spin(detection_driver)
    detection_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
