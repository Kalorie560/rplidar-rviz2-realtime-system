#!/bin/bash

# rosbag録画管理ツール
# 録画の開始・停止・一覧・再生を統合管理

echo "🎥 === LiDAR Recording Manager ==="

# 色設定
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# ROS2環境設定
source /opt/ros/humble/setup.bash

WORKSPACE_DIR="/home/yuki/claude/rplidar"
cd "$WORKSPACE_DIR"

# 機能一覧表示
show_menu() {
    echo ""
    echo -e "${GREEN}📋 機能一覧${NC}"
    echo "1️⃣  録画開始"
    echo "2️⃣  録画一覧"
    echo "3️⃣  録画再生"
    echo "4️⃣  録画削除"
    echo "5️⃣  録画統計"
    echo "6️⃣  終了"
    echo ""
}

# 録画開始
start_recording() {
    TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
    BAG_FILE="lidar_recording_$TIMESTAMP"
    
    echo -e "${YELLOW}🎥 録画開始準備${NC}"
    echo "📁 ファイル名: $BAG_FILE"
    echo -n "録画するトピックを選択 (1:スキャンのみ, 2:全て): "
    read -r topic_choice
    
    case "$topic_choice" in
        "1")
            TOPICS="/scan"
            echo "📊 スキャンデータのみ録画"
            ;;
        "2")
            TOPICS="/scan /tf /tf_static"
            echo "📊 全データ録画（スキャン + 座標変換）"
            ;;
        *)
            TOPICS="/scan"
            echo "📊 デフォルト：スキャンデータのみ録画"
            ;;
    esac
    
    echo ""
    echo -e "${RED}🔴 録画開始: $BAG_FILE${NC}"
    echo "停止するには Ctrl+C を押してください"
    echo ""
    
    ros2 bag record -o "$BAG_FILE" $TOPICS
    
    echo ""
    echo -e "${GREEN}✅ 録画完了: $BAG_FILE${NC}"
}

# 録画一覧
list_recordings() {
    echo -e "${BLUE}📂 録画ファイル一覧${NC}"
    echo ""
    
    if ls lidar_recording_*.db3 1> /dev/null 2>&1; then
        echo -e "${GREEN}利用可能な録画:${NC}"
        echo ""
        
        counter=1
        for bag_dir in lidar_recording_*; do
            if [ -d "$bag_dir" ]; then
                # db3ファイルサイズ取得
                db3_file="$bag_dir/$(basename "$bag_dir")_0.db3"
                if [ -f "$db3_file" ]; then
                    size=$(du -h "$db3_file" | cut -f1)
                    date=$(date -r "$bag_dir" '+%Y-%m-%d %H:%M:%S')
                    
                    echo -e "  ${counter}. 📁 $bag_dir"
                    echo -e "     💾 サイズ: $size"
                    echo -e "     📅 作成日: $date"
                    echo ""
                    
                    ((counter++))
                fi
            fi
        done
    else
        echo -e "${YELLOW}📭 録画ファイルが見つかりません${NC}"
    fi
}

# 録画再生
play_recording() {
    echo -e "${BLUE}▶️  録画再生${NC}"
    
    # 利用可能なbagファイル一覧（ディレクトリのみ）
    recordings=()
    for item in lidar_recording_*; do
        if [ -d "$item" ]; then
            recordings+=("$item")
        fi
    done
    
    if [ ${#recordings[@]} -eq 0 ]; then
        echo -e "${YELLOW}📭 再生可能な録画がありません${NC}"
        return
    fi
    
    echo "利用可能な録画:"
    counter=1
    for bag_dir in "${recordings[@]}"; do
        if [ -d "$bag_dir" ]; then
            # ディレクトリサイズと作成日時を表示
            size=$(du -sh "$bag_dir" 2>/dev/null | cut -f1)
            date=$(stat -c %y "$bag_dir" 2>/dev/null | cut -d' ' -f1,2 | cut -d'.' -f1)
            echo "  $counter. $bag_dir (${size:-不明}, ${date:-不明})"
            
            # .db3ファイルの詳細も表示
            if ls "$bag_dir"/*.db3 >/dev/null 2>&1; then
                db3_count=$(ls "$bag_dir"/*.db3 2>/dev/null | wc -l)
                echo "     📁 ${db3_count}個のデータファイル"
            fi
            ((counter++))
        fi
    done
    
    echo ""
    echo -n "再生する録画番号を入力: "
    read -r choice
    
    # 選択肢の検証
    if [[ "$choice" =~ ^[0-9]+$ ]] && [ "$choice" -ge 1 ] && [ "$choice" -lt "$counter" ]; then
        selected_bag="${recordings[$((choice-1))]}"
        
        echo -e "${GREEN}▶️  再生開始: $selected_bag${NC}"
        echo "RViz2を起動してから再生します..."
        
        # RViz2をバックグラウンドで起動
        rviz2 -d "$WORKSPACE_DIR/lidar_config.rviz" &
        rviz_pid=$!
        
        sleep 3
        
        echo "再生オプション:"
        echo "1. 通常再生"
        echo "2. ループ再生"
        echo "3. 0.5倍速再生"
        echo "4. 2倍速再生"
        echo -n "選択: "
        read -r play_option
        
        # ディレクトリ内のdb3ファイルを直接指定
        db3_file=$(find "$selected_bag" -name "*.db3" -type f | head -1)
        if [ -z "$db3_file" ]; then
            echo -e "${RED}❌ データファイルが見つかりません${NC}"
            return
        fi
        
        case "$play_option" in
            "2")
                echo "🔄 ループ再生中..."
                ros2 bag play "$db3_file" --loop
                ;;
            "3")
                echo "🐌 0.5倍速再生中..."
                ros2 bag play "$db3_file" --rate 0.5
                ;;
            "4")
                echo "🚀 2倍速再生中..."
                ros2 bag play "$db3_file" --rate 2.0
                ;;
            *)
                echo "▶️  通常再生中..."
                ros2 bag play "$db3_file"
                ;;
        esac
        
        # RViz2終了
        kill $rviz_pid 2>/dev/null
        
    else
        echo -e "${RED}❌ 無効な選択です${NC}"
    fi
}

# 録画削除
delete_recording() {
    echo -e "${RED}🗑️  録画削除${NC}"
    
    recordings=(lidar_recording_*)
    
    if [ ! -d "${recordings[0]}" ]; then
        echo -e "${YELLOW}📭 削除可能な録画がありません${NC}"
        return
    fi
    
    echo "削除可能な録画:"
    counter=1
    for bag_dir in "${recordings[@]}"; do
        if [ -d "$bag_dir" ]; then
            size=$(du -h "$bag_dir" | cut -f1)
            echo "  $counter. $bag_dir ($size)"
            ((counter++))
        fi
    done
    
    echo ""
    echo -n "削除する録画番号を入力 (0で全削除): "
    read -r choice
    
    if [ "$choice" = "0" ]; then
        echo -n -e "${RED}⚠️  全ての録画を削除しますか？ (y/N): ${NC}"
        read -r confirm
        if [[ "$confirm" =~ ^[Yy]$ ]]; then
            rm -rf lidar_recording_*
            echo -e "${GREEN}✅ 全ての録画を削除しました${NC}"
        else
            echo "キャンセルしました"
        fi
    elif [[ "$choice" =~ ^[0-9]+$ ]] && [ "$choice" -ge 1 ] && [ "$choice" -lt "$counter" ]; then
        selected_bag="${recordings[$((choice-1))]}"
        
        echo -n -e "${YELLOW}📁 $selected_bag を削除しますか？ (y/N): ${NC}"
        read -r confirm
        if [[ "$confirm" =~ ^[Yy]$ ]]; then
            rm -rf "$selected_bag"
            echo -e "${GREEN}✅ 削除完了: $selected_bag${NC}"
        else
            echo "キャンセルしました"
        fi
    else
        echo -e "${RED}❌ 無効な選択です${NC}"
    fi
}

# 録画統計
show_statistics() {
    echo -e "${BLUE}📊 録画統計${NC}"
    echo ""
    
    total_size=0
    total_count=0
    
    for bag_dir in lidar_recording_*; do
        if [ -d "$bag_dir" ]; then
            # フォルダサイズ計算（KB単位）
            size_kb=$(du -sk "$bag_dir" | cut -f1)
            total_size=$((total_size + size_kb))
            total_count=$((total_count + 1))
            
            # 人間が読みやすい形式に変換
            size_human=$(du -sh "$bag_dir" | cut -f1)
            
            echo -e "📁 $bag_dir: $size_human"
        fi
    done
    
    if [ $total_count -gt 0 ]; then
        # 合計サイズを人間が読みやすい形式に変換
        if [ $total_size -gt 1048576 ]; then
            total_human=$(echo "$total_size" | awk '{printf "%.1fGB", $1/1048576}')
        elif [ $total_size -gt 1024 ]; then
            total_human=$(echo "$total_size" | awk '{printf "%.1fMB", $1/1024}')
        else
            total_human="${total_size}KB"
        fi
        
        echo ""
        echo -e "${GREEN}📈 統計情報:${NC}"
        echo -e "   📊 録画数: $total_count 個"
        echo -e "   💾 合計サイズ: $total_human"
        echo -e "   📦 平均サイズ: $(echo "$total_size $total_count" | awk '{printf "%.1fMB", $1/$2/1024}')"
    else
        echo -e "${YELLOW}📭 録画ファイルが見つかりません${NC}"
    fi
}

# メインループ
while true; do
    show_menu
    echo -n "選択してください (1-6): "
    read -r choice
    
    case "$choice" in
        "1")
            start_recording
            ;;
        "2")
            list_recordings
            ;;
        "3")
            play_recording
            ;;
        "4")
            delete_recording
            ;;
        "5")
            show_statistics
            ;;
        "6")
            echo -e "${GREEN}👋 終了します${NC}"
            exit 0
            ;;
        *)
            echo -e "${RED}❌ 無効な選択です。1-6を入力してください。${NC}"
            ;;
    esac
    
    echo ""
    echo -n "Enterキーで続行..."
    read -r
done