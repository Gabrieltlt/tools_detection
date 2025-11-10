#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import rclpy
from rclpy.node import Node
from ultralytics import YOLO
from cv_bridge import CvBridge
from std_srvs.srv import Empty
from std_msgs.msg import String
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory

class ObjectTracking(Node):
    def __init__(self) -> None:
        super().__init__('yolov8_recognition')

        self.model = None
        self.run = False
        self.bridge = CvBridge()
        
        self.declareParameters()
        self.readParameters()
        self.initRosComm()
        
        if self.start_on_init:
            self._startRecognition()

    def __del__(self):
        """Destructor to ensure clean shutdown"""
        try:
            if hasattr(self, 'run') and self.run:
                self._stopRecognition()
        except:
            pass

    def destroy_node(self):
        """Override destroy_node to ensure clean shutdown"""
        try:
            if hasattr(self, 'run') and self.run:
                self._stopRecognition()
        except:
            pass
        super().destroy_node()

    def initRosComm(self) -> None:
        """Initialize ROS2 communication"""
        # Publishers
        self.imagePublisher = self.create_publisher(
            Image, self.imagePublisherTopic, qos_profile=self.imageQosProfile)
        self.trackingPublisher = self.create_publisher(
            Image, self.trackingPublisherTopic, qos_profile=self.trackingQosProfile)
        self.debugPublisher = self.create_publisher(
            String, self.debugPublisherTopic, qos_profile=self.debugQosProfile)
        
        # Services
        self.recognitionStartService = self.create_service(
            Empty, self.startRecognitionTopic, self.startRecognition)
        self.recognitionStopService = self.create_service(
            Empty, self.stopRecognitionTopic, self.stopRecognition)
        
        # Subscription
        self.imageSubscription = self.create_subscription(
            Image, self.imageSubscriberTopic, self.imageCallback, 
            qos_profile=self.imageSubscriberQosProfile)

    def loadModel(self) -> None:
        """Load YOLO model"""
        self.get_logger().info("=> Loading model")
        try:
            self.model = YOLO(self.modelFile)
            self.model.conf = self.threshold
            self.get_logger().info("=> Model loaded successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            self.model = None

    def unloadModel(self) -> None:
        """Unload YOLO model and free memory"""
        if self.model is not None:
            del self.model
            self.model = None
        self.get_logger().info("=> Model unloaded")

    def _startRecognition(self):
        """Internal method to start recognition"""
        self.loadModel()
        if self.model is not None:
            self.run = True
            self.get_logger().info("Starting Object Recognition!!!")
            if self.target_class:
                self.get_logger().info(f"=> Detection filter: {self.target_class.upper()} ONLY")
            else:
                self.get_logger().info("=> Detection filter: ALL CLASSES")
        else:
            self.get_logger().error("Cannot start recognition - model loading failed")

    def _stopRecognition(self):
        """Internal method to stop recognition"""
        self.run = False
        self.unloadModel()
        self.get_logger().info("Stopping Object Recognition!!!")

    def startRecognition(self, req: Empty.Request, resp: Empty.Response):
        """Service callback to start recognition"""
        self._startRecognition()
        return resp

    def stopRecognition(self, req: Empty.Request, resp: Empty.Response):
        """Service callback to stop recognition"""
        self._stopRecognition()
        return resp

    def getClassName(self, classId: int) -> str:
        """Get human-readable class name from class ID using model's class names"""
        if self.model is not None and classId in self.model.names:
            return self.model.names[classId]
        return f"class_{classId}"

    def _drawTrackingArrow(self, image, detectionCenters):
        """Draw tracking arrow pointing to the closest or most relevant object"""
        if not detectionCenters:
            return
            
        # Get image dimensions
        img_height, img_width = image.shape[:2]
        screen_center_x = img_width // 2
        screen_center_y = img_height // 2
        
        # Find the closest object to screen center for primary tracking
        closest_object = None
        min_distance = float('inf')
        
        for obj_center in detectionCenters:
            obj_x, obj_y = obj_center
            distance = ((obj_x - screen_center_x) ** 2 + (obj_y - screen_center_y) ** 2) ** 0.5
            if distance < min_distance:
                min_distance = distance
                closest_object = obj_center
        
        if closest_object is None:
            return
            
        obj_x, obj_y = closest_object
        
        # Calculate arrow parameters
        arrow_length = 60  # Tamanho da seta
        arrow_thickness = 3
        arrow_color = (0, 0, 255)  # Vermelho
        
        # Calculate direction vector from center to object
        dx = obj_x - screen_center_x
        dy = obj_y - screen_center_y
        
        # Normalize direction vector
        distance = (dx ** 2 + dy ** 2) ** 0.5
        if distance == 0:
            return  # Object is at center
            
        # Normalize direction
        dx_norm = dx / distance
        dy_norm = dy / distance
        
        # Calculate arrow start and end points
        arrow_start_x = int(screen_center_x + dx_norm * 20)  # Start a bit away from center
        arrow_start_y = int(screen_center_y + dy_norm * 20)
        arrow_end_x = int(screen_center_x + dx_norm * arrow_length)
        arrow_end_y = int(screen_center_y + dy_norm * arrow_length)
        
        # Draw arrow line
        cv2.arrowedLine(image, (arrow_start_x, arrow_start_y), (arrow_end_x, arrow_end_y), 
                       arrow_color, arrow_thickness, tipLength=0.3)
        
        # Draw center crosshair
        crosshair_size = 15
        cv2.line(image, (screen_center_x - crosshair_size, screen_center_y), 
                (screen_center_x + crosshair_size, screen_center_y), (255, 255, 255), 2)
        cv2.line(image, (screen_center_x, screen_center_y - crosshair_size), 
                (screen_center_x, screen_center_y + crosshair_size), (255, 255, 255), 2)
        
        # Add distance text
        distance_text = f"Dist: {int(min_distance)}px"
        cv2.putText(image, distance_text, (screen_center_x - 50, screen_center_y + 40),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    def imageCallback(self, imageMsg: Image) -> None:
        """Main callback for processing images"""
        if not self.run:
            return
        
        if self.model is None:
            self.get_logger().error("Model is not loaded.")
            return

        if imageMsg is None:
            self.get_logger().error("Input image message is invalid.")
            return
        
        try:
            # Convert ROS image to OpenCV format
            cvImage = self.bridge.imgmsg_to_cv2(imageMsg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        try:
            # Run YOLO inference
            results = self.model(cvImage, verbose=False)
            if not results or len(results) == 0:
                return
                
            result = results[0]
        except Exception as e:
            self.get_logger().error(f"YOLO inference failed: {e}")
            return

        # Process detections
        detectedClasses = []
        detectionCenters = []  # Para tracking das posições dos objetos
        
        if result.boxes is not None and len(result.boxes) > 0:
            for box in result.boxes:
                if box is None:
                    continue
                
                conf = float(box.conf)
                cls = int(box.cls)

                if conf < self.threshold:
                    continue

                # Get class name and apply class filter
                className = self.getClassName(cls)
                
                # Filter: Only detect specified target class (if set)
                if self.target_class and className.lower() != self.target_class:
                    continue

                # Get bounding box coordinates
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                # Calculate object center for tracking
                obj_center_x = (x1 + x2) // 2
                obj_center_y = (y1 + y2) // 2
                detectionCenters.append((obj_center_x, obj_center_y))
                
                # Get percentage
                percentage = conf * 100
                detectedClasses.append(f"{className}: {percentage:.1f}%")

                # Draw bounding box and label on image
                cv2.rectangle(cvImage, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(cvImage, f"{className} {percentage:.1f}%", (x1, y1 - 5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Create a copy for tracking visualization
        cvImageTracking = cvImage.copy()

        # Object Tracking - Draw arrow pointing to detected objects (only on tracking image)
        if detectionCenters:
            self._drawTrackingArrow(cvImageTracking, detectionCenters)

        # Publish normal detection image (without tracking elements)
        try:
            imageMsg_out = self.bridge.cv2_to_imgmsg(cvImage, "bgr8")
            imageMsg_out.header = imageMsg.header
            self.imagePublisher.publish(imageMsg_out)
        except Exception as e:
            self.get_logger().error(f"Failed to publish detection image: {e}")

        # Publish tracking image (with arrow and crosshair)
        try:
            trackingMsg_out = self.bridge.cv2_to_imgmsg(cvImageTracking, "bgr8")
            trackingMsg_out.header = imageMsg.header
            self.trackingPublisher.publish(trackingMsg_out)
        except Exception as e:
            self.get_logger().error(f"Failed to publish tracking image: {e}")

        # Publish detected classes as text
        classesMsg = String()
        if detectedClasses:
            classesMsg.data = " | ".join(detectedClasses)
        else:
            if self.target_class:
                classesMsg.data = f"No {self.target_class} detected"
            else:
                classesMsg.data = "No objects detected"
        self.debugPublisher.publish(classesMsg)

    def declareParameters(self) -> None:
        """Declare all ROS2 parameters"""
        # Publisher parameters
        self.declare_parameter("publishers.image.topic", "/micky_vision/tools_recognition")
        self.declare_parameter("publishers.image.qos_profile", 10)
        self.declare_parameter("publishers.tracking.topic", "/micky_vision/object_tracking")
        self.declare_parameter("publishers.tracking.qos_profile", 10)
        self.declare_parameter("publishers.debug.topic", "/micky_vision/tools_debug")
        self.declare_parameter("publishers.debug.qos_profile", 10)
        
        # Subscriber parameters
        self.declare_parameter("subscribers.image_rgb.topic", "/micky_vision/camera/color/image_raw")
        self.declare_parameter("subscribers.image_rgb.qos_profile", 10)
        
        # Service parameters
        self.declare_parameter("services.tracking.start", "/micky_vision/tracking_start")
        self.declare_parameter("services.tracking.stop", "/micky_vision/tracking_stop")
        
        # Algorithm parameters (values come from YAML config file)
        self.declare_parameter("threshold", 0.5)  # Default fallback - YAML will override
        self.declare_parameter("model_file", "yolov8n.pt")  # Default fallback - YAML will override  
        self.declare_parameter("start_on_init", True)  # Default fallback - YAML will override
        self.declare_parameter("target_class", "")  # Default fallback - YAML will override (empty = all classes)

    def readParameters(self) -> None:
        """Read all ROS2 parameters"""
        # Publisher parameters
        self.imagePublisherTopic = self.get_parameter("publishers.image.topic").value
        self.imageQosProfile = self.get_parameter("publishers.image.qos_profile").value
        self.trackingPublisherTopic = self.get_parameter("publishers.tracking.topic").value
        self.trackingQosProfile = self.get_parameter("publishers.tracking.qos_profile").value
        self.debugPublisherTopic = self.get_parameter("publishers.debug.topic").value
        self.debugQosProfile = self.get_parameter("publishers.debug.qos_profile").value
        
        # Subscriber parameters
        self.imageSubscriberTopic = self.get_parameter("subscribers.image_rgb.topic").value
        self.imageSubscriberQosProfile = self.get_parameter("subscribers.image_rgb.qos_profile").value
        
        # Service parameters
        self.startRecognitionTopic = self.get_parameter("services.tracking.start").value
        self.stopRecognitionTopic = self.get_parameter("services.tracking.stop").value
        
        # Algorithm parameters
        self.threshold = self.get_parameter("threshold").value
        self.start_on_init = self.get_parameter("start_on_init").value
        self.target_class = self.get_parameter("target_class").value.strip().lower()
        
        # Model file path
        modelFileName = self.get_parameter("model_file").value
        try:
            packageShareDirectory = get_package_share_directory('tools_recognition')
            self.modelFile = f"{packageShareDirectory}/weights/{modelFileName}"
        except Exception:
            self.modelFile = f"weights/{modelFileName}"
            self.get_logger().warn(f"Using relative path for model: {self.modelFile}")
        
        # Log important parameters
        self.get_logger().info(f"Threshold: {self.threshold}")
        self.get_logger().info(f"Model file: {self.modelFile}")
        self.get_logger().info(f"Start on init: {self.start_on_init}")
        self.get_logger().info(f"Target class filter: {'ALL CLASSES' if not self.target_class else self.target_class}")
        self.get_logger().info(f"Image topic: {self.imageSubscriberTopic}")
        self.get_logger().info(f"Output image topic: {self.imagePublisherTopic}")
        self.get_logger().info(f"Tracking topic: {self.trackingPublisherTopic}")
        self.get_logger().info(f"Debug topic: {self.debugPublisherTopic}")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTracking()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()