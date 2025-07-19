#!/usr/bin/env python3
"""
シンプルLiDARテストツール
最小構成でLiDARデータを確認
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class SimpleLidarTest(Node):
    def __init__(self):
        super().__init__('simple_lidar_test')
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            1)
        
        self.frame_count = 0
        self.get_logger().info('シンプルLiDARテスト開始')
    
    def scan_callback(self, msg):
        """最小限のスキャンデータ確認"""
        self.frame_count += 1
        
        if self.frame_count <= 5:
            print(f"\n=== フレーム {self.frame_count} ===")
            print(f"フレームID: {msg.header.frame_id}")
            print(f"測定点数: {len(msg.ranges)}")
            
            # 有効データ数
            ranges = np.array(msg.ranges)
            valid_mask = (ranges >= msg.range_min) & (ranges <= msg.range_max) & np.isfinite(ranges)
            valid_count = np.sum(valid_mask)
            
            print(f"有効測定点: {valid_count}/{len(msg.ranges)} ({valid_count/len(msg.ranges)*100:.1f}%)")
            
            if valid_count > 0:
                valid_ranges = ranges[valid_mask]
                print(f"距離範囲: {valid_ranges.min():.2f}m ~ {valid_ranges.max():.2f}m")
                print(f"平均距離: {valid_ranges.mean():.2f}m")
                
                # 前方10度の測定値
                angles = np.arange(len(msg.ranges)) * msg.angle_increment + msg.angle_min
                front_mask = (angles >= math.radians(-5)) & (angles <= math.radians(5)) & valid_mask
                
                if np.any(front_mask):
                    front_ranges = ranges[front_mask]
                    print(f"前方5度範囲: {front_ranges.min():.2f}m ~ {front_ranges.max():.2f}m")
                else:
                    print("前方にデータなし")
                
                print("✅ データ受信OK - RViz2で確認してください")
            else:
                print("❌ 有効データなし")
        
        elif self.frame_count == 10:
            print("\n✅ 10フレーム受信完了")
            print("RViz2でLaserScanが表示されているか確認してください")
            print("Ctrl+Cで終了")

def main(args=None):
    rclpy.init(args=args)
    
    print("🔍 === シンプルLiDARテスト ===")
    print("最小構成でデータ確認します")
    print()
    
    test_node = SimpleLidarTest()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        print("\n👋 テスト終了")
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()