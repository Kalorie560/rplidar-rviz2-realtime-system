#!/usr/bin/env python3
"""
計測と描写の同期テストツール
RPLiDARのスキャン同期とタイミングを検証
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import time
import math

class SyncTester(Node):
    def __init__(self):
        super().__init__('sync_tester')
        
        # サブスクリプション
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        self.frame_count = 0
        self.last_scan_time = None
        self.timing_errors = []
        
        self.get_logger().info('同期テスト開始')
    
    def scan_callback(self, msg):
        """スキャンの同期とタイミングを検証"""
        current_time = time.time()
        self.frame_count += 1
        
        # タイミング分析
        if self.last_scan_time is not None:
            interval = current_time - self.last_scan_time
            expected_interval = 0.1  # 10Hz想定
            timing_error = abs(interval - expected_interval)
            self.timing_errors.append(timing_error)
            
            if timing_error > 0.02:  # 20ms以上の誤差
                self.get_logger().warn(f"タイミング誤差: {timing_error:.3f}秒")
        
        self.last_scan_time = current_time
        
        # 10フレーム毎に詳細分析
        if self.frame_count % 10 == 0:
            self.analyze_sync(msg)
    
    def analyze_sync(self, msg):
        """同期の詳細分析"""
        print(f"\n=== 同期分析 (フレーム {self.frame_count}) ===")
        
        # タイムスタンプ分析
        scan_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        current_time = time.time()
        timestamp_delay = current_time - scan_timestamp
        
        print(f"⏱️  タイムスタンプ遅延: {timestamp_delay:.3f}秒")
        
        # スキャンタイミング
        print(f"📊 スキャン情報:")
        print(f"   スキャン時間: {msg.scan_time:.4f}秒")
        print(f"   時間増分: {msg.time_increment:.6f}秒")
        print(f"   測定点数: {len(msg.ranges)}")
        
        # 角度設定の確認
        angle_range = msg.angle_max - msg.angle_min
        expected_points = int(angle_range / msg.angle_increment)
        actual_points = len(msg.ranges)
        
        print(f"📐 角度設定:")
        print(f"   角度範囲: {math.degrees(angle_range):.1f}°")
        print(f"   期待点数: {expected_points}")
        print(f"   実際点数: {actual_points}")
        print(f"   点数誤差: {abs(expected_points - actual_points)}")
        
        if abs(expected_points - actual_points) > 5:
            print("⚠️  WARNING: 点数に大きな誤差があります")
        
        # タイミング統計
        if len(self.timing_errors) > 0:
            avg_error = np.mean(self.timing_errors)
            max_error = np.max(self.timing_errors)
            print(f"⏰ タイミング統計:")
            print(f"   平均誤差: {avg_error:.3f}秒")
            print(f"   最大誤差: {max_error:.3f}秒")
            
            if avg_error > 0.01:
                print("⚠️  WARNING: タイミング誤差が大きいです")
        
        # データ品質チェック
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[(ranges >= msg.range_min) & (ranges <= msg.range_max) & np.isfinite(ranges)]
        data_quality = len(valid_ranges) / len(ranges) * 100
        
        print(f"📈 データ品質: {data_quality:.1f}%")
        
        if data_quality < 50:
            print("⚠️  WARNING: データ品質が低いです")
        
        # 測定値の安定性チェック（前方1mの点を想定）
        front_indices = self.get_front_indices(msg)
        if len(front_indices) > 0:
            front_ranges = ranges[front_indices]
            front_valid = front_ranges[(front_ranges >= msg.range_min) & (front_ranges <= msg.range_max)]
            
            if len(front_valid) > 3:
                front_std = np.std(front_valid)
                print(f"🎯 前方測定安定性: {front_std:.3f}m")
                
                if front_std > 0.05:
                    print("⚠️  WARNING: 前方測定が不安定です")
    
    def get_front_indices(self, msg):
        """前方±15度の範囲のインデックスを取得"""
        angles = np.arange(len(msg.ranges)) * msg.angle_increment + msg.angle_min
        front_mask = (angles >= math.radians(-15)) & (angles <= math.radians(15))
        return np.where(front_mask)[0]

def main(args=None):
    rclpy.init(args=args)
    
    print("🔄 === 計測と描写の同期テスト ===")
    print("スキャンタイミングと同期を検証します")
    print("Ctrl+Cで終了")
    print()
    
    tester = SyncTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        print("\n👋 同期テスト終了")
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()