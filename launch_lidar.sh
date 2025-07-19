#!/bin/bash

# RPLiDAR リアルタイム可視化システム
# RViz2ベース - 高性能3D表示 & rosbag録画対応
# Robosense A1plus事前検証用

echo "🚁 === RPLiDAR Real-time Visualization System ==="
echo "📊 RViz2ベース - 高性能リアルタイム3D表示"
echo "🎥 rosbag録画機能付き"
echo ""

# 色設定
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# ROS2環境設定
echo -e "${BLUE}🔧 ROS2環境設定中...${NC}"
source /opt/ros/humble/setup.bash

# 必要パッケージの確認
echo -e "${BLUE}📦 必要パッケージを確認中...${NC}"

packages=("rviz2" "rplidar-ros")
for package in "${packages[@]}"; do
    if ! dpkg -l | grep -q $package; then
        echo -e "${YELLOW}⚠️  $package が見つかりません。インストール中...${NC}"
        sudo apt update && sudo apt install -y ros-humble-${package}
    else
        echo -e "${GREEN}✅ $package はインストール済み${NC}"
    fi
done

# USB権限設定
echo -e "${BLUE}🔌 USB権限設定中...${NC}"
USB_DEVICE=""
for device in /dev/ttyUSB*; do
    if [ -e "$device" ]; then
        USB_DEVICE="$device"
        sudo chmod 666 "$device"
        echo -e "${GREEN}✅ USB権限設定完了: $device${NC}"
        break
    fi
done

if [ -z "$USB_DEVICE" ]; then
    echo -e "${YELLOW}⚠️  RPLiDAR未検出${NC}"
    echo "   USBデバイスを接続してから再実行してください"
    exit 1
fi

# ワークスペース設定
WORKSPACE_DIR="/home/yuki/claude/rplidar"
cd "$WORKSPACE_DIR"

# 使用方法表示
echo ""
echo -e "${GREEN}🎮 === 使用方法 ===${NC}"
echo "1️⃣  基本表示    : Enter を押してRViz2起動"
echo "2️⃣  録画モード  : 'r' + Enter で録画付きで起動"
echo "3️⃣  再生モード  : 'p' + Enter でrosbag再生"
echo "4️⃣  終了        : Ctrl+C"
echo ""

# ユーザー選択
echo -n "選択してください (Enter/r/p): "
read -r choice

case "$choice" in
    "r"|"R")
        echo -e "${YELLOW}🎥 録画モード選択${NC}"
        
        # 録画ファイル名設定
        TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
        BAG_FILE="lidar_recording_$TIMESTAMP"
        
        echo "📁 録画ファイル: $BAG_FILE.db3"
        echo ""
        
        # TF Publisher起動（バックグラウンド）
        echo -e "${BLUE}🔗 座標変換システム起動中...${NC}"
        gnome-terminal --title="TF Publisher" -- bash -c "
            source /opt/ros/humble/setup.bash
            cd '$WORKSPACE_DIR'
            python3 tf_publisher.py
            read -p 'Press Enter to close...'
        " &
        
        sleep 1
        
        # RPLiDAR起動（バックグラウンド）
        echo -e "${BLUE}🚁 RPLiDARドライバー起動中...${NC}"
        gnome-terminal --title="RPLiDAR Driver" -- bash -c "
            source /opt/ros/humble/setup.bash
            ros2 run rplidar_ros rplidar_composition \
                --ros-args \
                -p serial_port:=/dev/ttyUSB1 \
                -p serial_baudrate:=115200 \
                -p frame_id:=laser \
                -p inverted:=false \
                -p angle_compensate:=true \
                -p scan_mode:=Standard
            read -p 'Press Enter to close...'
        " &
        
        sleep 3
        
        # rosbag録画開始（バックグラウンド）
        echo -e "${RED}🔴 録画開始: $BAG_FILE${NC}"
        gnome-terminal --title="rosbag Recording" -- bash -c "
            source /opt/ros/humble/setup.bash
            ros2 bag record -o $BAG_FILE /scan /tf /tf_static
            read -p 'Press Enter to close...'
        " &
        
        sleep 2
        
        # RViz2起動
        echo -e "${GREEN}📊 RViz2起動中...${NC}"
        rviz2 -d "$WORKSPACE_DIR/lidar_config.rviz"
        ;;
        
    "p"|"P")
        echo -e "${BLUE}📼 再生モード選択${NC}"
        
        # 利用可能なbagファイル一覧
        echo "利用可能な録画ファイル:"
        ls -la *.db3 2>/dev/null | grep -E "lidar_recording_.*\.db3$" | while read -r line; do
            filename=$(echo "$line" | awk '{print $9}')
            size=$(echo "$line" | awk '{print $5}')
            date=$(echo "$line" | awk '{print $6, $7, $8}')
            echo "  📁 $filename ($size bytes, $date)"
        done
        
        echo ""
        echo -n "再生するbagファイル名を入力 (拡張子なし): "
        read -r bag_name
        
        if [ -f "${bag_name}" ]; then
            # RViz2起動（バックグラウンド）
            echo -e "${GREEN}📊 RViz2起動中...${NC}"
            rviz2 -d "$WORKSPACE_DIR/lidar_config.rviz" &
            
            sleep 3
            
            # rosbag再生
            echo -e "${BLUE}▶️  再生開始: $bag_name${NC}"
            ros2 bag play "$bag_name" --loop
        else
            echo -e "${RED}❌ ファイルが見つかりません: $bag_name${NC}"
        fi
        ;;
        
    *)
        echo -e "${GREEN}📊 基本表示モード${NC}"
        
        # TF Publisher起動（バックグラウンド）
        echo -e "${BLUE}🔗 座標変換システム起動中...${NC}"
        gnome-terminal --title="TF Publisher" -- bash -c "
            source /opt/ros/humble/setup.bash
            cd '$WORKSPACE_DIR'
            python3 tf_publisher.py
            read -p 'Press Enter to close...'
        " &
        
        sleep 1
        
        # RPLiDAR起動（バックグラウンド）
        echo -e "${BLUE}🚁 RPLiDARドライバー起動中...${NC}"
        gnome-terminal --title="RPLiDAR Driver" -- bash -c "
            source /opt/ros/humble/setup.bash
            ros2 run rplidar_ros rplidar_composition \
                --ros-args \
                -p serial_port:=/dev/ttyUSB1 \
                -p serial_baudrate:=115200 \
                -p frame_id:=laser \
                -p inverted:=false \
                -p angle_compensate:=true \
                -p scan_mode:=Standard
            read -p 'Press Enter to close...'
        " &
        
        sleep 3
        
        # RViz2起動
        echo -e "${GREEN}📊 RViz2起動中...${NC}"
        rviz2 -d "$WORKSPACE_DIR/lidar_config.rviz"
        ;;
esac

echo ""
echo -e "${GREEN}✅ システム終了${NC}"