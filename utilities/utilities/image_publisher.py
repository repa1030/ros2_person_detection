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

from os import listdir, path

# Third Party
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImagePublisher(Node):

    def __init__(self):

        # Initialize class as node
        super().__init__('image_publisher')
        self.error = False

        # ROS parameter initialization
        self.declare_parameter('frequency')
        self.declare_parameter('image_topic')
        self.declare_parameter('test_data')
        self.declare_parameter('data_type')
        self.declare_parameter('image_resolution')

        # ROS node initialization
        timer_period = 1 / self.get_parameter('frequency').get_parameter_value().double_value
        self.img_publisher = self.create_publisher(Image,
                                                   self.get_parameter('image_topic').
                                                   get_parameter_value().string_value, 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('Node Initialized.')

        # CV Bridge for image converting and publishing
        self.bridge = CvBridge()

        # Image resize factor
        self.rf = self.get_parameter('image_resolution').get_parameter_value().double_value

        # Check data type
        self.data_type = self.get_parameter('data_type').get_parameter_value().string_value
        if self.data_type != 'image' and self.data_type != 'video':
            self.get_logger().error('\"' + str(self.data_type) + '\" is No Valid Data Type. \
                                Please Check Config File.')
            self.error = True

        # Check if directory exists
        self.data_dir = self.get_parameter('test_data').get_parameter_value().string_value
        if not path.exists(self.data_dir):
            self.get_logger().error('Given Data Directory Does Not Exist.')
            self.error = True

        # Generate list of files
        if path.isfile(self.data_dir) and not self.error:
            self.files = [self.data_dir]
        elif not self.error:
            if self.data_dir[-1] != '/':
                self.data_dir += '/'
            self.files = [(self.data_dir + f) for f in listdir(self.data_dir) if
                          path.isfile(path.join(self.data_dir, f))]
        self.file_counter = 0
        self.video_opened = False
        self.video = None

        # Everything is fine
        if not self.error:
            self.get_logger().info('Directory is Valid. Starting with Publishing.')

    def timer_callback(self):

        # Get first image
        data = self.files[self.file_counter]
        ret = False
        image = None

        # Capture image
        if self.data_type == 'image':
            image = cv2.imread(data)
            # Point to next file (overflow -> start from first file)
            self.file_counter += 1
            if self.file_counter >= len(self.files):
                self.file_counter = 0
        # Capture video
        elif self.data_type == 'video':
            if not self.video_opened:
                self.video = cv2.VideoCapture(data)
                self.video_opened = True
            # Get next frame
            if self.video.isOpened():
                ret, image = self.video.read()
            # Video is finished -> next video
            if not ret:
                self.file_counter += 1
                if self.file_counter >= len(self.files):
                    self.file_counter = 0
                self.video_opened = False
                self.video.release()

        # No valid image
        if image is None:
            return

        # Resize
        if self.rf < 1.0:
            width = int(image.shape[1] * self.rf)
            height = int(image.shape[0] * self.rf)
            image = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)

        # Convert to ROS message and publish
        msg = Image()
        msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        self.img_publisher.publish(msg)


def main():

    rclpy.init()
    image_publisher = ImagePublisher()
    if not image_publisher.error:
        rclpy.spin(image_publisher)
    image_publisher.get_logger().error('Shutdown of Image Publisher!')
    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
