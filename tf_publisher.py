#!/usr/bin/env python3
"""
静的座標変換発行ノード
RPLiDARのための正しいTF変換を発行
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import math

class TFPublisher(Node):
    def __init__(self):
        super().__init__('tf_publisher')
        
        # 静的座標変換ブロードキャスター
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        # 座標変換を発行
        self.publish_transforms()
        
        self.get_logger().info('TF Publisher initialized')
    
    def publish_transforms(self):
        """必要な座標変換を発行"""
        
        transforms = []
        current_time = self.get_clock().now().to_msg()
        
        # map → base_link フレーム変換（固定位置）
        map_transform = TransformStamped()
        map_transform.header.stamp = current_time
        map_transform.header.frame_id = 'map'
        map_transform.child_frame_id = 'base_link'
        
        # ワールド座標での位置
        map_transform.transform.translation.x = 0.0
        map_transform.transform.translation.y = 0.0
        map_transform.transform.translation.z = 0.0
        
        # 回転なし
        map_transform.transform.rotation.x = 0.0
        map_transform.transform.rotation.y = 0.0
        map_transform.transform.rotation.z = 0.0
        map_transform.transform.rotation.w = 1.0
        
        transforms.append(map_transform)
        
        # base_link → laser フレーム変換
        # RPLiDARの位置・向きを正しく設定
        laser_transform = TransformStamped()
        laser_transform.header.stamp = current_time
        laser_transform.header.frame_id = 'base_link'
        laser_transform.child_frame_id = 'laser'
        
        # 位置設定（base_linkからの相対位置）
        laser_transform.transform.translation.x = 0.0  # 前後位置
        laser_transform.transform.translation.y = 0.0  # 左右位置
        laser_transform.transform.translation.z = 0.0  # 上下位置
        
        # 回転設定（RPLiDARの向き）
        # デフォルトは前方向（0度回転）
        laser_transform.transform.rotation.x = 0.0
        laser_transform.transform.rotation.y = 0.0
        laser_transform.transform.rotation.z = 0.0
        laser_transform.transform.rotation.w = 1.0
        
        transforms.append(laser_transform)
        
        # 複数の変換を一度に発行
        self.tf_broadcaster.sendTransform(transforms)
        
        self.get_logger().info('Published TF transforms: map -> base_link -> laser')

def main(args=None):
    rclpy.init(args=args)
    
    tf_publisher = TFPublisher()
    
    try:
        # ノードを維持（静的変換は一度発行すれば十分）
        rclpy.spin(tf_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        tf_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()