import torch

# CUDA 가용 여부 확인 후 모델 로드
device = 'cuda' if torch.cuda.is_available() else 'cpu'
# get_logger().info(f"Using device: {device}")
print(device)
# Load your YOLO model
# model = YOLO('weights/YOLO_0216.pt').to(device)
