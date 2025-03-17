import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import open3d as o3d
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import os
from scipy.spatial import cKDTree
import time
import socket
import pickle
from matplotlib import pyplot as plt
from geometry_msgs.msg import Point

class SaveBackgroundPointCloud(Node):
    def __init__(self):
        super().__init__('save_background_pointcloud')

        # 设置 QoS 策略
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 创建订阅者
        self.subscription = self.create_subscription(
            PointCloud2,
            '/mid70clp',
            self.topic_callback,
            qos_profile
        )

        # 初始化点云存储
        self.point_clouds = []
        self.num_samples = 0
        self.target_samples = 100  # 设置目标样本数

    def topic_callback(self, msg):
        try:
            # 解析点云数据
            pcl = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            if not pcl:
                self.get_logger().warn('Received an empty point cloud')
                return

            # 转换为 NumPy 数组
            points = np.array(pcl, dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])
            points = points.view(dtype=np.float32).reshape(-1, 3)  # 转换为普通数组

            # 添加到点云存储列表
            self.point_clouds.append(points)
            self.num_samples += 1
            self.get_logger().info(f"Collected {self.num_samples} point clouds")

            # 当收集到目标样本数时，计算平均点云
            if self.num_samples >= self.target_samples:
                self.calculate_average_point_cloud()
                # 取消订阅，停止收集
                self.get_logger().info("Unsubscribing from topic...")
                self.destroy_subscription(self.subscription)

        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')

    def calculate_average_point_cloud(self):
        # 合并所有点云数据
        merged_points = np.concatenate(self.point_clouds, axis=0)
        self.get_logger().info(f"Total points merged: {merged_points.shape[0]}")

        # 计算点云中心
        center = np.mean(merged_points, axis=0)
        self.get_logger().info(f"Center of merged point cloud: {center}")

        # 创建平均点云对象
        average_pcd = o3d.geometry.PointCloud()
        average_pcd.points = o3d.utility.Vector3dVector(merged_points)
        average_pcd = average_pcd.voxel_down_sample(voxel_size=0.05)  # 可选：下采样以减少点数

        # 保存为 PLY 文件
        #output_file = "background_avg.ply"
        #o3d.io.write_point_cloud(output_file, average_pcd)
        #self.get_logger().info(f"Average background point cloud saved to {output_file}")

        # 可视化平均点云
        #o3d.visualization.draw_geometries([average_pcd])

        # 销毁节点
        self.get_logger().info("Destroying node...")
        self.destroy_node()
        rclpy.shutdown()

class Mid70PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('mid70_pointcloud_subscriber')

        # 设置 QoS 策略
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 创建订阅者
        self.subscription = self.create_subscription(
            PointCloud2,
            '/mid70clp',
            self.topic_callback,
            qos_profile
        )

        # 创建聚类中心发布者
        self.cluster_center_publisher = self.create_publisher(
            Point,
            'lidar_topic',
            qos_profile
        )

        # 加载背景点云（假设背景点云存储为 .ply 文件）
        self.get_logger().info("Loading background point cloud...")
        self.background = o3d.io.read_point_cloud("background_avg.ply")
        self.background = self.background.voxel_down_sample(voxel_size=0.05)
        self.background_points = np.asarray(self.background.points)
        self.kdtree = cKDTree(self.background_points)

    def topic_callback(self, msg):
        try:
            # 解析点云数据
            pcl = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            if not pcl:
                self.get_logger().warn('Received an empty point cloud')
                return

            # 转换为 NumPy 数组
            points = np.array(pcl, dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])
            points = points.view(dtype=np.float32).reshape(-1, 3)  # 转换为普通数组

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            
            points_array = np.asarray(pcd.points)
            distances, _ = self.kdtree.query(points_array, k=1)
            mean_distance = np.mean(distances)
            filter_threshold = 0.1  # 静态阈值
            filter_mask = distances >= filter_threshold
            filtered_points = points_array[filter_mask]

            # 场地滤波处理 
            #print("->场地滤波中...")
            #points_array = cKDTree(np.asarray(down_pcd.points)) 
            #distances, _ = points_array.query(np.asarray(points_array),  k=1)
            #filter_mask = distances >= np.mean(distances)  * 1.5  # 动态阈值 
            #filtered_points = points.select_by_index(np.where(filter_mask)[0]) 

            # 创建滤波后的点云
            filtered_pcd = o3d.geometry.PointCloud()
            filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)

            # DBSCAN 聚类
            eps = 0.30  # 聚类半径
            min_points = 100  # 有效聚类最小点数

            with o3d.utility.VerbosityContextManager(
                    o3d.utility.VerbosityLevel.Info) as cm:
                labels = np.array(filtered_pcd.cluster_dbscan(eps, min_points, print_progress=True))

            max_label = labels.max()
            self.get_logger().info(f"DBSCAN 聚类标签数目: {max_label + 1}")

            # 提取各个簇的中心点
            centers = []
            for label in range(max_label + 1):
                indices = np.where(labels == label)[0]
                cluster_points = filtered_points[indices]
                if len(cluster_points) > 0:  # 跳过无效聚类
                    center = cluster_points.mean(axis=0)
                    centers.append(center)
                    self.get_logger().info(f"类别 {label + 1} 的中心坐标: {center}")

                    
                    #udp通讯
                    #my_list = [[float(center[0]),float(center[1]),float(center[2])]]
                    #print(f"x:{my_list[0][0]}, y{my_list[0][1]}:, z:{my_list[0][2]}")
                    #udp_send_list("192.168.1.113", 9999, my_list)
                    

                    # 创建 Point 消息并发布
                    point_msg = Point()
                    point_msg.x = float(center[0])
                    point_msg.y = float(center[1])
                    point_msg.z = float(center[2])
                    self.cluster_center_publisher.publish(point_msg)

            # 可视化
            colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
            colors[labels < 0] = 0  # 噪声点设为黑色
            filtered_pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])

        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')


def udp_send_list(target_ip, target_port, data_list):
    # 创建UDP套接字 
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    try:
        # 序列化数据 
        serialized_data = pickle.dumps(data_list)
        
        # 发送数据（注意：Python默认缓冲区大小通常为8192）
        sock.sendto(serialized_data, (target_ip, target_port))
        print(f"已发送 {len(serialized_data)} 字节数据")
        
    except Exception as e:
        print(f"发送失败: {str(e)}")
    finally:
        sock.close()

def main(args=None):
    # 首先检查背景点云文件是否存在
    background_file = "background_avg.ply"
    background_exists = os.path.exists(background_file)
    
    # 如果背景点云不存在，先运行第一部分节点保存背景点云
    if not background_exists:
        rclpy.init(args=args)
        saver = SaveBackgroundPointCloud()
        executor = MultiThreadedExecutor()
        executor.add_node(saver)
        executor.spin()  # 使用 spin 方法
        saver.destroy_node()
        rclpy.shutdown()

    # 如果背景点云存在，则直接运行第二部分节点
    if background_exists or os.path.exists(background_file):
        rclpy.init(args=args)
        subscriber = Mid70PointCloudSubscriber()
        executor = MultiThreadedExecutor()
        executor.add_node(subscriber)
        executor.spin()  # 使用 spin 方法
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()