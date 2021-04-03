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

from ftplib import FTP
from io import BytesIO
import telnetlib

# Third Party
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CognexDriver(Node):

    def __init__(self):

        # Initialize class as node
        super().__init__('cognex_driver')

        # ROS parameter initialization
        self.declare_parameter('frequency')
        self.declare_parameter('image_topic')
        self.declare_parameter('telnet_port')
        self.declare_parameter('cognex_ip')
        self.declare_parameter('cognex_user')
        self.declare_parameter('cognex_password')
        self.declare_parameter('cognex_file_name')
        self.declare_parameter('image_resolution')

        # ROS node initialization
        timer_period = 1 / self.get_parameter('frequency').get_parameter_value().double_value
        self.cognex_publisher = self.create_publisher(Image,
                                                      self.get_parameter('image_topic').
                                                      get_parameter_value().string_value, 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Node Initialized.')

        # CV Bridge for image converting and publishing
        self.bridge = CvBridge()

        # Image resize factor
        self.rf = self.get_parameter('image_resolution').get_parameter_value().double_value

        # Telnet connection initialization and login
        telnet_user = self.get_parameter('cognex_user').get_parameter_value().string_value \
            + '\r\n'
        telnet_pw = self.get_parameter('cognex_password').get_parameter_value().string_value \
            + '\r\n'
        self.get_logger().info('Waiting for Cognex Camera ...')
        self.tn = telnetlib.Telnet(
            self.get_parameter('cognex_ip').get_parameter_value().string_value,
            self.get_parameter('telnet_port').get_parameter_value().integer_value)
        self.tn.write(telnet_user.encode('ascii'))
        self.tn.write(telnet_pw.encode('ascii'))

        # FTP connection
        self.file = self.get_parameter('cognex_file_name').get_parameter_value().string_value
        self.ftp = FTP(self.get_parameter('cognex_ip').get_parameter_value().string_value)
        self.ftp.login(self.get_parameter('cognex_user').get_parameter_value().string_value,
                       self.get_parameter('cognex_password').get_parameter_value().string_value)

        # Connected to cognex camera
        self.get_logger().info('Connetion to Cognex Established.')

    def timer_callback(self):

        # Request image capturing via Telnet
        self.tn.write(b'SE8\r\n')

        # Load empty buffer
        buf = BytesIO()

        # Download image from camera to PC via FTP
        self.ftp.retrbinary('RETR ' + self.file, buf.write)

        # Convert to numpy array and openCV image
        image = np.asarray(bytearray(buf.getvalue()), dtype='uint8')
        image = cv2.imdecode(image, cv2.IMREAD_COLOR)
        if self.rf < 1.0:
            width = int(image.shape[1] * self.rf)
            height = int(image.shape[0] * self.rf)
            image = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)

        # Convert to ROS message and publish
        msg = Image()
        msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        self.cognex_publisher.publish(msg)


def main():

    rclpy.init()
    cognex_driver = CognexDriver()
    rclpy.spin(cognex_driver)
    cognex_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
