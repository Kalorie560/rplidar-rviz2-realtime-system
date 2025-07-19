#!/usr/bin/env python3
"""
手動RPLiDARテスト - 最小構成
"""
import serial
import time
import struct

def test_rplidar_direct():
    """RPLiDARとの直接通信テスト"""
    try:
        # シリアルポート接続
        ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        print("✅ RPLiDAR接続成功")
        
        # デバイス情報取得コマンド
        ser.write(b'\xA5\x50\x00\x00\x00\x01')
        time.sleep(0.1)
        
        response = ser.read(20)
        if len(response) > 0:
            print(f"✅ デバイス応答: {len(response)} バイト")
            print(f"データ: {response.hex()}")
        else:
            print("❌ デバイス応答なし")
        
        ser.close()
        return True
        
    except Exception as e:
        print(f"❌ RPLiDAR接続エラー: {e}")
        return False

def test_ros2_rplidar():
    """ROS2経由でのRPLiDARテスト"""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    
    class TestNode(Node):
        def __init__(self):
            super().__init__('test_node')
            self.received_data = False
            
            self.subscription = self.create_subscription(
                LaserScan, '/scan', self.callback, 10)
            
            # 5秒待機
            timer = self.create_timer(5.0, self.timeout_callback)
            
        def callback(self, msg):
            print(f"✅ ROS2データ受信: {len(msg.ranges)} 測定点")
            self.received_data = True
            
        def timeout_callback(self):
            if not self.received_data:
                print("❌ ROS2データ受信なし")
            rclpy.shutdown()
    
    try:
        rclpy.init()
        node = TestNode()
        rclpy.spin(node)
        return node.received_data
    except Exception as e:
        print(f"❌ ROS2テストエラー: {e}")
        return False

if __name__ == '__main__':
    print("🔍 === RPLiDAR診断テスト ===")
    print()
    
    print("1. 直接通信テスト")
    direct_ok = test_rplidar_direct()
    print()
    
    if direct_ok:
        print("2. ROS2通信テスト")
        ros2_ok = test_ros2_rplidar()
        print()
        
        if ros2_ok:
            print("✅ 全テスト成功 - システム正常")
        else:
            print("⚠️  ROS2ドライバーに問題")
    else:
        print("❌ ハードウェア接続に問題")