#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import os

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        
        # Model dosyaları
        package_path = os.path.dirname(os.path.realpath(__file__))
        self.weights = os.path.join(package_path, 'models', 'yolov4-tiny.weights')
        self.config = os.path.join(package_path, 'models', 'yolov4-tiny.cfg')
        
        # YOLO modelini yükle
        self.net = cv2.dnn.readNet(self.weights, self.config)
        
        # CPU kullanımını zorla
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        
        # COCO sınıf isimleri
        self.classes = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck']
        
        # Hedef sınıflar (indeksler)
        self.target_classes = {
            0: 'person',  # person
            2: 'car',     # car
            3: 'motorcycle', # motorcycle
            5: 'bus',     # bus
            7: 'truck'    # truck
        }
        
        # CV bridge
        self.bridge = CvBridge()
        
        # Kamera ayarları
        self.camera_id = 0
        self.cap = cv2.VideoCapture(self.camera_id)
        
        # Görüntü boyutu
        self.input_size = (320, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # FPS hesaplama
        self.prev_frame_time = 0
        self.new_frame_time = 0
        
        # Timer (5 FPS)
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        # Publisher
        self.publisher = self.create_publisher(Image, '/detected_objects', 10)
        
        # Frame counter
        self.frame_counter = 0
        self.process_every_n_frames = 1
        
        self.get_logger().info('CPU Object Detection Node Started with YOLOv4-tiny')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # FPS hesapla
            self.new_frame_time = time.time()
            fps = 1/(self.new_frame_time-self.prev_frame_time)
            self.prev_frame_time = self.new_frame_time
            
            if self.frame_counter % self.process_every_n_frames == 0:
                detected_frame = self.detect_objects(frame)
            else:
                detected_frame = frame
            
            # FPS göster
            cv2.putText(detected_frame, f'FPS: {int(fps)}', (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (100, 255, 0), 2)
            
            try:
                ros_image = self.bridge.cv2_to_imgmsg(detected_frame, encoding='bgr8')
                self.publisher.publish(ros_image)
            except Exception as e:
                self.get_logger().error(f'Error publishing image: {str(e)}')
            
            self.frame_counter += 1

    def detect_objects(self, frame):
        height, width = frame.shape[:2]
        
        # Görüntüyü hazırla
        blob = cv2.dnn.blobFromImage(frame, 1/255.0, self.input_size, swapRB=True, crop=False)
        self.net.setInput(blob)
        
        # Çıktı katmanlarının isimlerini al
        layer_names = self.net.getLayerNames()
        output_layers = [layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]
        
        # Tespit yap
        outputs = self.net.forward(output_layers)
        
        # Tespitleri işle
        boxes = []
        confidences = []
        class_ids = []
        
        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                
                if class_id in self.target_classes.keys() and confidence > 0.6:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    
                    x = int(center_x - w/2)
                    y = int(center_y - h/2)
                    
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
        
        # Non-maximum suppression uygula
        indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.6, 0.3)
        
        if len(indices) > 0:
            for i in indices.flatten():
                x, y, w, h = boxes[i]
                label = self.target_classes[class_ids[i]]
                confidence = confidences[i]
                
                # Renk seç
                color = (0, 255, 0) if label == 'person' else (255, 0, 0)
                
                # Kutu çiz
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                
                # Etiket yaz
                text = f"{label}: {confidence:.2f}"
                cv2.putText(frame, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        return frame

    def __del__(self):
        if hasattr(self, 'cap'):
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = ObjectDetectionNode()
        rclpy.spin(node)
    except Exception as e:
        print(f'Error in main: {str(e)}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()