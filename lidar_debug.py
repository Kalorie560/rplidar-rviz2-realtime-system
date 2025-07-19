#!/usr/bin/env python3
"""
RPLiDAR データ検証ツール
角度情報とスキャンデータの正確性を検証
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class LidarDebugger(Node):
    def __init__(self):
        super().__init__('lidar_debugger')
        
        # サブスクリプション
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        self.frame_count = 0
        self.get_logger().info('LiDAR Debugger started')
    
    def scan_callback(self, msg):
        """スキャンデータを詳細分析"""
        self.frame_count += 1
        
        # 基本情報表示（最初の10フレームのみ）
        if self.frame_count <= 10:
            self.print_scan_info(msg)
        
        # 定期的に詳細分析（10秒毎）
        if self.frame_count % 100 == 0:
            self.analyze_scan_data(msg)
    
    def print_scan_info(self, msg):
        """基本スキャン情報を表示"""
        print(f"\n=== フレーム {self.frame_count} ===")
        print(f"📐 角度範囲:")
        print(f"   最小角度: {math.degrees(msg.angle_min):.1f}°")
        print(f"   最大角度: {math.degrees(msg.angle_max):.1f}°")
        print(f"   角度増分: {math.degrees(msg.angle_increment):.3f}°")
        print(f"   測定点数: {len(msg.ranges)}")
        print(f"   角度範囲: {math.degrees(msg.angle_max - msg.angle_min):.1f}°")
        
        print(f"📏 距離範囲:")
        print(f"   最小距離: {msg.range_min:.2f}m")
        print(f"   最大距離: {msg.range_max:.2f}m")
        
        print(f"⏱️ タイミング:")
        print(f"   スキャン時間: {msg.scan_time:.4f}秒")
        print(f"   時間増分: {msg.time_increment:.6f}秒")
        
        # 有効データ数
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        print(f"📊 データ:")
        print(f"   有効点数: {len(valid_ranges)}/{len(msg.ranges)}")
        print(f"   有効率: {len(valid_ranges)/len(msg.ranges)*100:.1f}%")
    
    def analyze_scan_data(self, msg):
        """詳細なスキャンデータ分析"""
        print(f"\n🔍 === 詳細分析 (フレーム {self.frame_count}) ===")
        
        # 角度配列を計算
        angles = np.arange(len(msg.ranges)) * msg.angle_increment + msg.angle_min
        ranges = np.array(msg.ranges)
        
        # 有効データのみ抽出
        valid_mask = (ranges >= msg.range_min) & (ranges <= msg.range_max) & np.isfinite(ranges)
        valid_angles = angles[valid_mask]
        valid_ranges = ranges[valid_mask]
        
        if len(valid_ranges) == 0:
            print("❌ 有効なデータがありません")
            return
        
        print(f"📊 統計情報:")
        print(f"   有効点数: {len(valid_ranges)}")
        print(f"   距離統計:")
        print(f"     最小: {valid_ranges.min():.2f}m")
        print(f"     最大: {valid_ranges.max():.2f}m")
        print(f"     平均: {valid_ranges.mean():.2f}m")
        print(f"     標準偏差: {valid_ranges.std():.2f}m")
        
        # 角度セクター分析（8方向）
        print(f"📐 方向別距離分析:")
        sectors = [
            ("前方    (±22.5°)", -22.5, 22.5),
            ("右前方  (22.5-67.5°)", 22.5, 67.5),
            ("右方    (67.5-112.5°)", 67.5, 112.5),
            ("右後方  (112.5-157.5°)", 112.5, 157.5),
            ("後方    (157.5-202.5°)", 157.5, 202.5),
            ("左後方  (202.5-247.5°)", 202.5, 247.5),
            ("左方    (247.5-292.5°)", 247.5, 292.5),
            ("左前方  (292.5-337.5°)", 292.5, 337.5)
        ]
        
        for name, start_deg, end_deg in sectors:
            start_rad = math.radians(start_deg)
            end_rad = math.radians(end_deg)
            
            # 角度範囲を正規化（-π to π → 0 to 2π）
            sector_mask = self.angle_in_range(valid_angles, start_rad, end_rad)
            sector_ranges = valid_ranges[sector_mask]
            
            if len(sector_ranges) > 0:
                min_dist = sector_ranges.min()
                avg_dist = sector_ranges.mean()
                print(f"   {name}: {min_dist:.2f}m (最小), {avg_dist:.2f}m (平均), {len(sector_ranges)}点")
            else:
                print(f"   {name}: データなし")
        
        # デカルト座標変換の確認
        print(f"🗺️  デカルト座標サンプル:")
        sample_indices = np.linspace(0, len(valid_angles)-1, min(5, len(valid_angles)), dtype=int)
        for i in sample_indices:
            angle = valid_angles[i]
            distance = valid_ranges[i]
            x = distance * np.cos(angle)
            y = distance * np.sin(angle)
            print(f"   角度{math.degrees(angle):6.1f}° 距離{distance:5.2f}m → X:{x:6.2f}m Y:{y:6.2f}m")
    
    def angle_in_range(self, angles, start_rad, end_rad):
        """角度が指定範囲内にあるかチェック（-π to π 対応）"""
        # 角度を0-2πに正規化
        norm_angles = np.where(angles < 0, angles + 2*math.pi, angles)
        norm_start = start_rad if start_rad >= 0 else start_rad + 2*math.pi
        norm_end = end_rad if end_rad >= 0 else end_rad + 2*math.pi
        
        # 範囲チェック
        if norm_start <= norm_end:
            return (norm_angles >= math.radians(norm_start)) & (norm_angles < math.radians(norm_end))
        else:
            # 360度を跨ぐ場合
            return (norm_angles >= math.radians(norm_start)) | (norm_angles < math.radians(norm_end))

def main(args=None):
    rclpy.init(args=args)
    
    print("🔍 === RPLiDAR データ検証ツール ===")
    print("角度情報と測定データの正確性を検証します")
    print("Ctrl+Cで終了")
    print()
    
    debugger = LidarDebugger()
    
    try:
        rclpy.spin(debugger)
    except KeyboardInterrupt:
        print("\n👋 デバッガー終了")
    finally:
        debugger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()