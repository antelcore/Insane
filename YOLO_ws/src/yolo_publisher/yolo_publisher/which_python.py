import rclpy
from rclpy.node import Node
import sys
import os

class CheckPythonPathNode(Node):
    def __init__(self):
        super().__init__('check_python_path_node')

        # 현재 사용 중인 Python 실행 경로
        python_exec = sys.executable
        which_python = os.popen("which python").read().strip()
        site_packages = sys.path

        self.get_logger().info(f"Python Executable (sys.executable): {python_exec}")
        self.get_logger().info(f"Python Executable (which python): {which_python}")
        self.get_logger().info("Python sys.path:")

        for path in site_packages:
            self.get_logger().info(f"  - {path}")

def main(args=None):
    rclpy.init(args=args)
    node = CheckPythonPathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
