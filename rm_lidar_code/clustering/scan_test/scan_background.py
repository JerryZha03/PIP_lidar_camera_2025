import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import open3d as o3d
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class SaveBackgroundPointCloud(Node):
    def __init__(self):
        super().__init__('save_background_pointcloud')

        # 设置 QoS 策略
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
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
            if self.num_samples == self.target_samples:
                self.calculate_average_point_cloud()
                # 取消订阅，停止收集
                self.subscription.destroy()

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
        output_file = "background_league.ply"
        o3d.io.write_point_cloud(output_file, average_pcd)
        self.get_logger().info(f"Average background point cloud saved to {output_file}")

        # 可视化平均点云
        o3d.visualization.draw_geometries_with_vertex_selection([average_pcd])
        #o3d.visualization.draw_geometries([average_pcd])




def main(args=None):
    rclpy.init(args=args)
    saver = SaveBackgroundPointCloud()
    rclpy.spin(saver)
    saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()