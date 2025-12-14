import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import subprocess
import signal
import os
import time

class MissionRecorder(Node):
    def __init__(self):
        super().__init__('mission_recorder')
        
        self.is_recording = False
        self.bag_process = None
        self.video_writer = None
        self.bridge = CvBridge()
        
        # Subs
        self.status_sub = self.create_subscription(String, '/mission_status', self.status_callback, 10)
        self.img_sub = self.create_subscription(CompressedImage, '/camera_node/image_raw/compressed', self.img_callback, 10)
        
        self.get_logger().info("Mission Recorder Standing By...")

    def status_callback(self, msg):
        status = msg.data
        
        if status == "RUNNING" and not self.is_recording:
            self.start_recording()
        
        elif status == "FINISHED" and self.is_recording:
            self.stop_recording()

    def start_recording(self):
        self.get_logger().info("STARTING RECORDING (Video + Bag)...")
        self.is_recording = True
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        
        # 1. Start Bag Recording (Subprocess)
        bag_name = f"mission_bag_{timestamp}"
        self.bag_process = subprocess.Popen(
            ['ros2', 'bag', 'record', '-a', '-o', bag_name],
            preexec_fn=os.setsid # Create new session group for clean kill
        )
        
        # 2. Init Video Writer
        self.video_filename = f"mission_video_{timestamp}.mp4"
        # We initialize writer in the first image callback to ensure correct size

    def stop_recording(self):
        self.get_logger().info("STOPPING RECORDING...")
        self.is_recording = False
        
        # 1. Stop Bag
        if self.bag_process:
            os.killpg(os.getpgid(self.bag_process.pid), signal.SIGINT)
            self.bag_process = None
            self.get_logger().info("Bag Saved.")

        # 2. Stop Video
        if self.video_writer:
            self.video_writer.release()
            self.video_writer = None
            self.get_logger().info(f"Video Saved: {self.video_filename}")

    def img_callback(self, msg):
        if not self.is_recording:
            return
            
        # Decode Compressed Image
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        if self.video_writer is None:
            height, width, _ = frame.shape
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(self.video_filename, fourcc, 20.0, (width, height))
            
        self.video_writer.write(frame)

def main(args=None):
    rclpy.init(args=args)
    node = MissionRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_recording()
    node.destroy_node()
    rclpy.shutdown()