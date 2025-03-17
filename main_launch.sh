#!/bin/bash

# 定义信号处理函数
cleanup() {
    echo "收到中断信号，正在终止所有后台进程..."
    kill $pid1 $pid2 $pid3 2>/dev/null
    wait $pid1 $pid2 $pid3 2>/dev/null
    echo "所有后台进程已终止"
    exit 1
}

# 捕获 SIGINT 信号（Ctrl+C）
trap cleanup SIGINT

# 启动第一个脚本并在后台运行
./lidar_launch.sh &
pid1=$!

# 启动第二个脚本并在后台运行
./lidar_topic_launch.sh &
pid2=$!

# 启动第三个脚本并在后台运行
./camera_launch.sh &
pid3=$!

# 等待所有后台进程完成
wait $pid1 $pid2 $pid3

echo "所有脚本已完成"