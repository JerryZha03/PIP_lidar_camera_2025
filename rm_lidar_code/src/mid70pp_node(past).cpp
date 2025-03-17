//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
#include <vector>
#include <cstdint>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif
#include <string.h>
#include "livox_sdk.h"

#include <memory>
#include <mutex>
#include <chrono>
#include <condition_variable>

#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/msg/string.hpp"

#define TARGET_POINTS 10000

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
//point_cloud2_iterator

using namespace std::chrono;
std::mutex mtx;
std::condition_variable lidar_arrive_condition;
std::mutex mtx_lidardata;
std::condition_variable lidar_data_arrive_condition;
// LivoxExtendRawPoint point_data;
LivoxExtendRawPoint *p_point_data;              //=&point_data;
//std::vector<LivoxExtendRawPoint> points_buffer; // 全局点云数据缓冲区

// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <sensor_msgs/point_field.h>
#include <stdint.h>

// 点结构体
struct Point {
    float x, y, z;
};

// 解析点云数据
std::vector<Point> parsePointCloud(const std::vector<uint8_t>& data, size_t point_step) {
    std::vector<Point> points;
    size_t num_points = data.size() / point_step;

    for (size_t i = 0; i < num_points; ++i) {
        const uint8_t* point_data = data.data() + i * point_step;
        Point point;
        point.x = *reinterpret_cast<const float*>(point_data + 0);  // x 坐标
        point.y = *reinterpret_cast<const float*>(point_data + 4);  // y 坐标
        point.z = *reinterpret_cast<const float*>(point_data + 8);  // z 坐标
        points.push_back(point);
    }

    return points;
}


// 环形缓冲区实现
template <typename T, size_t Capacity>
class RingBuffer {
private:
    T data[Capacity];      // 固定容量数组
    size_t head = 0;        // 最旧数据的位置
    size_t tail = 0;        // 下一个写入的位置
    size_t count = 0;       // 当前元素数量

public:
    void push(const T& value) {
        data[tail] = value;          // 写入新数据
        tail = (tail + 1) % Capacity; // 尾指针循环移动
        
        if (count < Capacity) {
            count++;                  // 未满时增加计数
        } else {
            head = (head + 1) % Capacity; // 已满时头指针后移（覆盖旧数据）
        }
    }

    T& operator[](size_t index) {
              if (index >= count) {
                  throw std::out_of_range("Index out of range");
              }
              return data[(head + index) % Capacity];
          }
      
          const T& operator[](size_t index) const {
              if (index >= count) {
                  throw std::out_of_range("Index out of range");
              }
              return data[(head + index) % Capacity];
          }

    // 示例：简单遍历（实际可扩展迭代器）
    void print() const {
        size_t index = head;
        for (size_t i = 0; i < count; ++i) {
            std::cout << data[index] << " ";
            index = (index + 1) % Capacity;
        }
        std::cout << "\n";
    }


     // 新增的 size() 方法，用于获取当前数组中的数据量
     size_t size() const {
      return count;
     }

};

//创建环形缓冲区对象
RingBuffer<LivoxExtendRawPoint, TARGET_POINTS> points_buffer;

class PointCloudSubscriber : public rclcpp::Node {
public:
    PointCloudSubscriber()
        : Node("point_cloud_subscriber") {
        // 创建订阅者，订阅 /mid70clp 话题
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/mid70clp", 10,
            std::bind(&PointCloudSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    // 订阅者回调函数
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // 解析点云数据
        std::vector<Point> points = parsePointCloud(msg->data, msg->point_step);

        // 将解析后的点云数据存储到环形缓冲区
        for (const auto& point : points) {
            points_buffer_.push(point);
        }

        // 打印当前缓冲区中的点云数量
        std::cout << "Points in buffer: " << points_buffer_.size() << std::endl;
    }

    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

    // 环形缓冲区
    RingBuffer<Point, 1000> points_buffer_;
};

typedef enum
{
  kDeviceStateDisconnect = 0,
  kDeviceStateConnect = 1,
  kDeviceStateSampling = 2,
} DeviceState;

typedef struct
{
  uint8_t handle;
  DeviceState device_state;
  DeviceInfo info;
} DeviceItem;

DeviceItem devices[kMaxLidarCount];
uint32_t data_recveive_count[kMaxLidarCount];

//连接所有的广播设备
int lidar_count = 0;
char broadcast_code_list[kMaxLidarCount][kBroadcastCodeSize];

/** Connect the broadcast device in list, please input the broadcast code and modify the BROADCAST_CODE_LIST_SIZE. */
/*#define BROADCAST_CODE_LIST_SIZE  3
int lidar_count = BROADCAST_CODE_LIST_SIZE;
char broadcast_code_list[kMaxLidarCount][kBroadcastCodeSize] = {
  "000000000000002",
  "000000000000003",
  "000000000000004"
};*/


void OnGetPointCloudReturnMode(livox_status status, uint8_t handle, LidarGetPointCloudReturnModeResponse* response, void* client_data) {
  // 处理响应
  if (response != NULL) {
      // 读取或者使用response中的数据
      printf("---------Return Mode: %d \n", response->mode);
  }
}



//等到两秒直到没有接收到任何设备
void WaitForDevicesReady()
{
  bool device_ready = false;
  seconds wait_time = seconds(2);
  steady_clock::time_point last_time = steady_clock::now();
  while (!device_ready)
  {
    std::unique_lock<std::mutex> lock(mtx);
    lidar_arrive_condition.wait_for(lock, wait_time);
    // printf("%d\n",(steady_clock::now() - last_time + milliseconds(50)));
    if ((steady_clock::now() - last_time + milliseconds(50)) >= wait_time)
    {
      device_ready = true;
    }
    else
    {
      last_time = steady_clock::now();
    }
  }
}

//从雷达接收错误信息
void OnLidarErrorStatusCallback(livox_status status, uint8_t handle, ErrorMessage *message)
{
  static uint32_t error_message_count = 0;
  if (message != NULL)
  {
    ++error_message_count;
    if (0 == (error_message_count % 100))
    {
      printf("handle: %u\n", handle);
      printf("temp_status : %u\n", message->lidar_error_code.temp_status);
      printf("volt_status : %u\n", message->lidar_error_code.volt_status);
      printf("motor_status : %u\n", message->lidar_error_code.motor_status);
      printf("dirty_warn : %u\n", message->lidar_error_code.dirty_warn);
      printf("firmware_err : %u\n", message->lidar_error_code.firmware_err);
      printf("pps_status : %u\n", message->lidar_error_code.device_status);
      printf("fan_status : %u\n", message->lidar_error_code.fan_status);
      printf("self_heating : %u\n", message->lidar_error_code.self_heating);
      printf("ptp_status : %u\n", message->lidar_error_code.ptp_status);
      printf("time_sync_status : %u\n", message->lidar_error_code.time_sync_status);
      printf("system_status : %u\n", message->lidar_error_code.system_status);
    }
  }
}

//从雷达接收点云
void GetLidarData(uint8_t handle, LivoxEthPacket *data, uint32_t data_num, void *client_data)
{
  std::lock_guard<std::mutex> lock(mtx_lidardata);
  //static int count=0;
  if (data)
  {
    data_recveive_count[handle]++;
    //count++;
    if (data_recveive_count[handle] % 100 == 0)
    {
      /** Parsing(解析) the timestamp and the point cloud data. */
      uint64_t cur_timestamp = *((uint64_t *)(data->timestamp));
      //printf("\n%ld\n",cur_timestamp);

      // 转换为ROS2时间
      //rclcpp::Time ros_time(cur_timestamp); // 直接接受纳秒级时间戳
      // 分解时间
      //int32_t sec = ros_time.seconds();
      //uint32_t nsec = ros_time.nanoseconds();
      // 格式化输出
      //printf( "ROS Time: %d.%09d", sec, nsec);



      if (data->data_type == kCartesian)
      {
        LivoxRawPoint *p_point_data = (LivoxRawPoint *)data->data;
        /*
        for (uint32_t i = 0; i < data_num; i++)
        {
          printf("Point %d: x = %d mm, y = %d mm, z = %d mm, reflectivity = %d\n",
                 i, p_point_data[i].x, p_point_data[i].y, p_point_data[i].z,
                 p_point_data[i].reflectivity);
        }
        */
      }
      else if (data->data_type == kSpherical)
      {
        LivoxSpherPoint *p_point_data = (LivoxSpherPoint *)data->data;
      }
      else if (data->data_type == kExtendCartesian)
      {
        p_point_data = (LivoxExtendRawPoint *)data->data;

      /*if(count==1)
      {
        // 清空缓冲区并保留内存空间以提高效率
        points_buffer.clear();
        //oints_buffer.reserve(NUM*96);
        //printf("0000000000000000000000000points_buffer的大小为：%ld\n", points_buffer.size());
        //getchar();
      } */


        // 清空缓冲区并保留内存空间以提高效率
        //points_buffer.clear();
        //points_buffer.reserve(data_num);

        // 将数据存入全局缓冲区
        for (uint32_t i = 0; i < data_num; i++) {
          points_buffer.push(p_point_data[i]);
        }


        // 每100包数据打印一次日志
        //printf("Received %u points\n", data_num);
        //printf("Received %u points\n", points_buffer.size());

        /*
        for (uint32_t i = 0; i < data_num; i++)
        {
          printf("Point %d: x = %d mm, y = %d mm, z = %d mm, reflectivity = %d\n",
                 i, p_point_data[i].x, p_point_data[i].y, p_point_data[i].z,
                 p_point_data[i].reflectivity);
        }
        */
        //printf("小飞象来咯\n");
      }
      else if (data->data_type == kExtendSpherical)
      {
        LivoxExtendSpherPoint *p_point_data = (LivoxExtendSpherPoint *)data->data;
      }
      else if (data->data_type == kDualExtendCartesian)
      {
        LivoxDualExtendRawPoint *p_point_data = (LivoxDualExtendRawPoint *)data->data;
      }
      else if (data->data_type == kDualExtendSpherical)
      {
        LivoxDualExtendSpherPoint *p_point_data = (LivoxDualExtendSpherPoint *)data->data;
      }
      else if (data->data_type == kImu)
      {
        LivoxImuPoint *p_point_data = (LivoxImuPoint *)data->data;
      }
      else if (data->data_type == kTripleExtendCartesian)
      {
        LivoxTripleExtendRawPoint *p_point_data = (LivoxTripleExtendRawPoint *)data->data;
      }
      else if (data->data_type == kTripleExtendSpherical)
      {
        LivoxTripleExtendSpherPoint *p_point_data = (LivoxTripleExtendSpherPoint *)data->data;
      }
      // printf("data_type %d packet num %d\n", data->data_type, data_recveive_count[handle]);
    }
    //printf("小飞象来咯\n");
    lidar_data_arrive_condition.notify_one();

  }
}

//开始采样的反馈信息
void OnSampleCallback(livox_status status, uint8_t handle, uint8_t response, void *data)
{
  printf("OnSampleCallback statue %d handle %d response %d \n", status, handle, response);
  if (status == kStatusSuccess)
  {
    if (response != 0)
    {
      devices[handle].device_state = kDeviceStateConnect;
    }
  }
  else if (status == kStatusTimeout)
  {
    devices[handle].device_state = kDeviceStateConnect;
  }
}

//停止采样的反馈功能
void OnStopSampleCallback(livox_status status, uint8_t handle, uint8_t response, void *data)
{
}

/** 查询Livox LiDAR框架的版本 */
void OnDeviceInformation(livox_status status, uint8_t handle, DeviceInformationResponse *ack, void *data)
{
  if (status != kStatusSuccess)
  {
    printf("Device Query Informations Failed %d\n", status);
  }
  if (ack)
  {
    printf("firm ver: %d.%d.%d.%d\n",
           ack->firmware_version[0],
           ack->firmware_version[1],
           ack->firmware_version[2],
           ack->firmware_version[3]);
  }
}

//雷达连接
void LidarConnect(const DeviceInfo *info)
{
  uint8_t handle = info->handle;
  QueryDeviceInformation(handle, OnDeviceInformation, NULL);
  if (devices[handle].device_state == kDeviceStateDisconnect)
  {
    devices[handle].device_state = kDeviceStateConnect;
    devices[handle].info = *info;
  }
}

//雷达没有连接
void LidarDisConnect(const DeviceInfo *info)
{
  uint8_t handle = info->handle;
  devices[handle].device_state = kDeviceStateDisconnect;
}

//雷达状态改变
void LidarStateChange(const DeviceInfo *info)
{
  uint8_t handle = info->handle;
  devices[handle].info = *info;
}

//设备状态改变的反馈功能
void OnDeviceInfoChange(const DeviceInfo *info, DeviceEvent type)
{
  if (info == NULL)
  {
    return;
  }

  uint8_t handle = info->handle;
  if (handle >= kMaxLidarCount)
  {
    return;
  }
  if (type == kEventConnect)
  {
    LidarConnect(info);
    printf("[WARNING] Lidar sn: [%s] Connect!!!\n", info->broadcast_code);
  }
  else if (type == kEventDisconnect)
  {
    LidarDisConnect(info);
    printf("[WARNING] Lidar sn: [%s] Disconnect!!!\n", info->broadcast_code);
  }
  else if (type == kEventStateChange)
  {
    LidarStateChange(info);
    printf("[WARNING] Lidar sn: [%s] StateChange!!!\n", info->broadcast_code);
  }

  if (devices[handle].device_state == kDeviceStateConnect)
  {
    printf("Device Working State %d\n", devices[handle].info.state);
    if (devices[handle].info.state == kLidarStateInit)
    {
      printf("Device State Change Progress %u\n", devices[handle].info.status.progress);
    }
    else
    {
      printf("Device State Error Code 0X%08x\n", devices[handle].info.status.status_code.error_code);
    }
    printf("Device feature %d\n", devices[handle].info.feature);
    SetErrorMessageCallback(handle, OnLidarErrorStatusCallback);
    if (devices[handle].info.state == kLidarStateNormal)
    {
      LidarStartSampling(handle, OnSampleCallback, NULL);
      devices[handle].device_state = kDeviceStateSampling;
    }
  }
}

//受到广播信息时的反馈
//You need to add listening device broadcast code and set the point cloud data callback in this function.
void OnDeviceBroadcast(const BroadcastDeviceInfo *info)
{
  if (info == NULL || info->dev_type == kDeviceTypeHub)
  {
    return;
  }

  printf("???Receive Broadcast Code %s\n", info->broadcast_code);

  if (lidar_count > 0)
  {
    bool found = false;
    int i = 0;
    for (i = 0; i < lidar_count; ++i)
    {
      if (strncmp(info->broadcast_code, broadcast_code_list[i], kBroadcastCodeSize) == 0)
      {
        found = true;
        break;
      }
    }
    if (!found)
    {
      lidar_arrive_condition.notify_one();
      return;
    }
  }

  bool result = false;
  uint8_t handle = 0;
  result = AddLidarToConnect(info->broadcast_code, &handle);
  printf("AddLidarToConnect result %d handle %d\n", result, handle);
  if (result == kStatusSuccess)
  {
    /** Set the point cloud data for a specific Livox LiDAR. */
    SetDataCallback(handle, GetLidarData, NULL);
    devices[handle].handle = handle;
    devices[handle].device_state = kDeviceStateDisconnect;
  }
}

class mid70pub : public rclcpp::Node
{
public:
  mid70pub() : Node("mid70pp")
  {
    //publisher_ = this->create_publisher<std_msgs::msg::String>("mid70clp", 10);
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mid70clp", rclcpp::SensorDataQoS());
    printf("ros节点初始化ing\n");
    //publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mid70clp",10);
    // 启动主循环，等待数据并发布
    printf("启动主循环\n");
    std::thread main_thread(&mid70pub::processData, this);
    printf("主循环启动成功\n");


    //printf("ros节点关闭\n");

    main_thread.join();
    printf("ros节点关闭\n");
    rclcpp::shutdown();
    printf("ros节点关闭成功\n");
  }

private:
  void processData()
  {
    //printf("processData");
    while (rclcpp::ok())
    {

      std::unique_lock<std::mutex> lock(mtx_lidardata);
      //printf("-------------------------------%d\n",points_buffer.empty());
      // 等待新数据到达，最多等待100毫秒
      auto status = lidar_data_arrive_condition.wait_for(lock, std::chrono::milliseconds(100));


      if (!rclcpp::ok())
      {
        printf("ROS 2 上下文已关闭！\n");
        break;
      }


      if (status == std::cv_status::timeout)
      {
        printf("Timeout: No new data received...\n");
        continue; // 跳过本次循环的后续代码，进入下一次循环
      }




      // 将lidar_data转换为ROS2消息并发布
      //printf("新数据要发布了！！！\n");

      // 创建ROS2点云消息
      sensor_msgs::msg::PointCloud2 msg;
      // ...

      // 设置消息头
      msg.header.stamp = this->now();
      msg.header.frame_id = "livox_frame"; // 建议使用专用雷达坐标系

      /* 设置点云结构 */
      msg.height = 1;                      // 无序点云设为1
      msg.width = points_buffer.size();//;    // 点云数量
      //printf("%ldiiiiiiiiiiiiiiiiiiiii\n", points_buffer.size());
      //getchar();

      // 定义字段（x, y, z）
      sensor_msgs::PointCloud2Modifier modifier(msg);
      //modifier.setPointCloud2FieldsByString(1, "xyz");
      msg.fields.resize(3);
      msg.fields[0].name = "x";
      msg.fields[0].offset = 0;
      msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
      msg.fields[0].count = 1;
      msg.fields[1].name = "y";
      msg.fields[1].offset = 4;
      msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
      msg.fields[1].count = 1;
      msg.fields[2].name = "z";
      msg.fields[2].offset = 8;
      msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
      msg.fields[2].count = 1;

      // 计算数据布局参数
      msg.point_step = 3 * sizeof(float); // 每个点的字节数（12: float32 x 3）
      msg.row_step = msg.point_step * msg.width;

      // 分配存储空间
      msg.data.resize(msg.row_step * msg.height);

      // 使用迭代器填充数据
      sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

      // 转换每个点的坐标系（毫米转米）
      for (uint32_t i = 0; i < points_buffer.size(); i++) {
          *iter_x = points_buffer[i].x / 1000.0f;
          *iter_y = points_buffer[i].z / 1000.0f;
          *iter_z = points_buffer[i].y/ 1000.0f;

          ++iter_x; ++iter_y; ++iter_z; // 移动迭代器到下一个点
      }

      

      /*
      for (uint32_t i = 0; i < 96; i++)
        {
          printf("Point %d: x = %d mm, y = %d mm, z = %d mm, reflectivity = %d\n",
                 i, points_buffer[i].x, points_buffer[i].y, points_buffer[i].z,
                 points_buffer[i].reflectivity);
        }
      */



        
      
      publisher_->publish(msg);

      //points_buffer.clear(); // 清空已处理数据
    }
  }
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, const char *argv[])
{
  printf("Livox SDK initializing.\n");
  /** Initialize Livox-SDK. */
  if (!Init())
  {
    return -1;
  }
  printf("Livox SDK has been initialized.\n");

  LivoxSdkVersion _sdkversion;
  GetLivoxSdkVersion(&_sdkversion);
  printf("Livox SDK version %d.%d.%d .\n", _sdkversion.major, _sdkversion.minor, _sdkversion.patch);

  memset(devices, 0, sizeof(devices));
  memset(data_recveive_count, 0, sizeof(data_recveive_count));

  /** Set the callback function receiving broadcast message from Livox LiDAR. */
  SetBroadcastCallback(OnDeviceBroadcast);

  /** Set the callback function called when device state change,
   * which means connection/disconnection and changing of LiDAR state.
   */
  SetDeviceStateUpdateCallback(OnDeviceInfoChange);
  printf("????????\n");


  //开始周期性寻找设备
  if (!Start())
  {
    printf("Start discovering device failed.\n");
    Uninit();
    return -1;
  }
  printf("Start discovering device.\n");


WaitForDevicesReady();


int huibomode = LidarSetPointCloudReturnMode(0, kStrongestReturn, NULL, NULL);
printf("set return mode:%x\n", huibomode);
LidarGetPointCloudReturnModeResponse return_mode_response;
LidarGetPointCloudReturnMode(0, OnGetPointCloudReturnMode, &return_mode_response);
printf("strong return mode:%x\n", kStrongestReturn);
printf("actual return mode:%x\n", return_mode_response.ret_code);



  // 等待雷达连接

  //linux设置静态ip命令


  // 初始化ros节点
  rclcpp::init(argc, argv);
  printf("ros节点初始化完成\n");

  // 开启节点
  rclcpp::spin(std::make_shared<mid70pub>());
  auto node = std::make_shared<PointCloudSubscriber>();
  printf("主程序继续\n");

  // 关闭节点
  rclcpp::shutdown();
  printf("ros节点关闭成功\n");

  #ifdef WIN32
    // printf("running on windows, sleep 30s\n");
    Sleep(30000);
  #else
    // printf("running on linux, sleep 30s\n");
    sleep(30000);
  #endif


  int i = 0;
  for (i = 0; i < kMaxLidarCount; ++i)
  {
    if (devices[i].device_state == kDeviceStateSampling)
    {
      /** Stop the sampling of Livox LiDAR. */
      LidarStopSampling(devices[i].handle, OnStopSampleCallback, NULL);
    }
  }

  /** Uninitialize Livox-SDK. */
  Uninit();
}