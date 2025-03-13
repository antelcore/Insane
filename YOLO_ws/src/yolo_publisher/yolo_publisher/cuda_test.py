import rclpy
from rclpy.node import Node
import torch
import sys

class CheckDeviceNode(Node):
    def __init__(self):
        super().__init__('check_device_node')
        
        # Python 실행 경로 확인
        python_executable = sys.executable
        self.get_logger().info(f"Python Executable: {python_executable}")

        # CUDA 및 PyTorch 정보 확인
        cuda_available = torch.cuda.is_available()
        device = "cuda" if cuda_available else "cpu"
        torch_version = torch.__version__

        self.get_logger().info(f"Using device: {device}")
        self.get_logger().info(f"CUDA Available: {cuda_available}")
        self.get_logger().info(f"Torch Version: {torch_version}")

def main(args=None):
    rclpy.init(args=args)
    node = CheckDeviceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
