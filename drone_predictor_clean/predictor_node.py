import os
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

import torch
import torch.nn as nn
import numpy as np
from collections import deque

from ament_index_python.packages import get_package_share_directory


# ===============================
# Parameters (must match training)
# ===============================
T_OBS = 10
T_PRED = 10


class MLPredictor(Node):
    def __init__(self):
        super().__init__('ml_predictor')

        # -----------------------------
        # ROS interfaces
        # -----------------------------
        self.sub = self.create_subscription(
            PoseStamped,
            '/drone/pose_stamped',
            self.pose_cb,
            10
        )

        self.pub = self.create_publisher(
            Path,
            '/drone/predicted_path',
            10
        )

        # -----------------------------
        # Load trained ML model
        # -----------------------------
        pkg_path = get_package_share_directory('drone_predictor_clean')
        model_path = os.path.join(pkg_path, 'models', 'position_ml_model.pt')

        checkpoint = torch.load(model_path, map_location='cpu', weights_only=False)


        self.mean = checkpoint['mean']
        self.std = checkpoint['std']

        self.model = nn.Sequential(
            nn.Linear(20, 64),
            nn.ReLU(),
            nn.Linear(64, 20)
        )

        self.model.load_state_dict(checkpoint['model_state'])
        self.model.eval()

        # -----------------------------
        # Rolling buffer for poses
        # -----------------------------
        self.buffer = deque(maxlen=T_OBS)

        self.get_logger().info('ML Predictor Node started successfully')

    # ===================================
    # Callback for incoming drone poses
    # ===================================
    def pose_cb(self, msg: PoseStamped):

        pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y
        ], dtype=np.float32)

        self.buffer.append(pos)

        # Wait until buffer is full
        if len(self.buffer) < T_OBS:
            return

        # -----------------------------
        # Prepare ML input
        # -----------------------------
        past = np.array(self.buffer)
        past_norm = (past - self.mean) / self.std
        inp = past_norm.reshape(1, -1)

        # -----------------------------
        # ML inference
        # -----------------------------
        with torch.no_grad():
            out = self.model(torch.tensor(inp, dtype=torch.float32))

        preds_norm = out.numpy().reshape(T_PRED, 2)
        preds = preds_norm * self.std + self.mean

        # -----------------------------
        # Publish Path
        # -----------------------------
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        for p in preds:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = float(p[0])
            ps.pose.position.y = float(p[1])
            ps.pose.position.z = msg.pose.position.z
            path.poses.append(ps)

        self.pub.publish(path)


def main():
    rclpy.init()
    node = MLPredictor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
