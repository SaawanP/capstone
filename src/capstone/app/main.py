import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
import cv2
import numpy as np

from PySide6.QtCore import Qt, QPoint, Signal, QObject, Slot
from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtGui import QImage, QPixmap

# You'll need to adjust these imports based on your structure
from ui.new_interface import Ui_MainWindow
from src.gui_functionality import GuiFunctions
from Custom_Widgets import *
from Custom_Widgets.QAppSettings import QAppSettings
import _icons_rc

from src.backend import Backend

# # Signal class to communicate between ROS2 thread and Qt GUI
# class CameraSignals(QObject):
#     image_ready = Signal(np.ndarray)

# class CameraSubscriber(Node):
#     def __init__(self):
#         super().__init__('camera_subscriber')
#         self.bridge = CvBridge()
#         self.signals = CameraSignals()
        
#         # Subscribe to camera topic
#         self.subscription = self.create_subscription(
#             Image,
#             '/camera/image_raw',  # Adjust this topic to match your camera node
#             self.image_callback,
#             10)
    
#     def image_callback(self, msg):
#         # Convert ROS Image message to OpenCV image
#         # try:
#         #     cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#         #     self.signals.image_ready.emit(cv_image)
#         # except Exception as e:
#         #     self.get_logger().error(f'Error processing image: {e}')

class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        
        # Apply JSON styling
        json_file_path = os.path.join(os.path.dirname(__file__), "json-styles", "style.json")
        loadJsonStyle(self, self.ui, jsonFiles={json_file_path})

        # Update app settings if necessary
        QAppSettings.updateAppSettings(self)

        self.app_functions = GuiFunctions(self)
        
        # Initialize camera feed display
        # Assuming you have a QLabel named 'cameraFeed' in your UI
        # If not, you'll need to modify this part
        # if hasattr(self.ui, 'cameraFeed'):
        #     self.camera_subscriber = None
        #     self.setup_camera_connection()
        
        # Show the main window
        self.show()

    # def setup_camera_connection(self):
    #     # This will be called once ROS2 is initialized
    #     if hasattr(self, 'camera_subscriber') and self.camera_subscriber is not None:
    #         self.camera_subscriber.signals.image_ready.connect(self.update_camera_feed)
    
    @Slot(np.ndarray)
    def update_camera_feed(self, frame_image):
        if hasattr(self.ui, 'cameraFeed'):
            # Convert CV image to Qt format
            height, width, channel = frame_image.shape
            bytes_per_line = 3 * width
            q_image = QImage(frame_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(q_image)
            
            # Update the QLabel
            self.ui.cameraFeed.setPixmap(pixmap)
            self.ui.cameraFeed.setScaledContents(True)

    @Slot(np.ndarray)
    def update_defects(self, defect_image):
        if hasattr(self.ui, 'capturedDefects'):
            # Convert CV image to Qt format
            height, width, channel = defect_image.shape
            bytes_per_line = 3 * width
            q_image = QImage(defect_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(q_image)

            # Update the QLabel
            self.ui.capturedDefects.setPixmap(pixmap)
            self.ui.capturedDefects.setScaledContents(True)

def ros_thread_function(backend_node):
    # Spin the node in a separate thread
    executor = MultiThreadedExecutor()
    executor.add_node(backend_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        # camera_subscriber.destroy_node()
        rclpy.shutdown()

def main(args=None):
    # Start ROS2 in a separate thread
    rclpy.init(args=args)
    backend_node = Backend()
    ros_thread = threading.Thread(target=ros_thread_function, args=(backend_node,))
    ros_thread.daemon = True  # Allow the thread to exit when the main process exits
    ros_thread.start()
    
    # Start the Qt application
    app = QApplication(sys.argv)
    window = MainWindow()

    # Wait a bit for ROS2 to initialize
    import time
    time.sleep(1)
    
    # Connect signals between window and node
    backend_node.signals.image_ready.connect(window.update_camera_feed)
    backend_node.signals.defect_detect.connect(window.update_defects)
    window.app_functions.startSignal.connect(backend_node.send_report_details)
    
    # Run the application
    return_code = app.exec()
    
    # Cleanup
    if ros_thread.is_alive():
        ros_thread.join(timeout=1.0)
    
    sys.exit(return_code)

if __name__ == "__main__":
    main()