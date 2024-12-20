#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from flask import Flask, Response
from flask_cors import CORS
import threading
import json

app = Flask(__name__)
CORS(app)

class WebStreamNode(Node):
    def __init__(self):
        super().__init__('web_stream_node')
        self.bridge = CvBridge()
        self.latest_frame = None
        
        # ROS2 subscriber
        self.subscription = self.create_subscription(
            Image,
            '/detected_objects',
            self.image_callback,
            10)
        
        self.get_logger().info('Web Stream Node Started')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Görüntü boyutunu küçült
            cv_image = cv2.resize(cv_image, (640, 480))
            # JPEG kalitesini düşür ama çok da düşük olmasın
            _, jpeg = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            self.latest_frame = jpeg.tobytes()
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {str(e)}')

def generate_frames(node):
    while True:
        if node.latest_frame is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + node.latest_frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(web_stream_node),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return """
    <html>
    <head>
        <title>ROS2 Object Detection Stream</title>
        <style>
            body { 
                margin: 0; 
                padding: 20px;
                font-family: Arial, sans-serif;
                background: #f0f0f0;
            }
            .container {
                max-width: 800px;
                margin: 0 auto;
                text-align: center;
            }
            img {
                max-width: 100%;
                border: 2px solid #333;
                border-radius: 8px;
            }
            h1 {
                color: #333;
                margin-bottom: 20px;
            }
        </style>
    </head>
    <body>
        <div class="container">
            <h1>ROS2 Object Detection Stream</h1>
            <img src="/video_feed" />
        </div>
    </body>
    </html>
    """

def main(args=None):
    rclpy.init(args=args)
    global web_stream_node
    web_stream_node = WebStreamNode()
    
    # ROS2 spin in a separate thread
    thread = threading.Thread(target=lambda: rclpy.spin(web_stream_node))
    thread.daemon = True
    thread.start()
    
    # Run Flask app
    app.run(host='0.0.0.0', port=5000)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()