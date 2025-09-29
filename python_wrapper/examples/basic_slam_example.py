#!/usr/bin/env python3
"""
RTAB-Map Python 包装器基础 SLAM 示例

这个示例演示了如何使用 RTAB-Map Python 包装器进行基础的 RGB-D SLAM 处理。

SLAM (Simultaneous Localization and Mapping) 是同时定位与地图构建技术，
它能够让机器人在未知环境中同时估计自己的位置并构建环境地图。

RGB-D SLAM 使用彩色图像(RGB)和深度图像(D)来实现更准确的定位和建图：
- RGB 图像：提供丰富的视觉特征信息，用于场景识别和回环检测
- 深度图像：提供三维空间信息，用于精确的位置估计和地图构建

RTAB-Map (Real-Time Appearance-Based Mapping) 是一个开源的 SLAM 库，
特别适合于实时的视觉 SLAM 应用。
"""

import numpy as np
import cv2
import time
import os
import sys

# 将父目录添加到 Python 路径中，以便导入 rtabmap_python 模块
# 这是一个常见的技巧，用于在开发阶段导入本地模块
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rtabmap_python as rtab


def main():
    """
    主 SLAM 处理循环函数
    
    这个函数演示了完整的 SLAM 处理流程：
    1. 初始化 RTAB-Map 系统
    2. 配置 SLAM 参数
    3. 处理传感器数据
    4. 检测回环闭合
    5. 保存结果
    """
    print("RTAB-Map Python 包装器 - 基础 SLAM 示例")
    print("=" * 50)
    
    # 第一步：初始化 RTAB-Map 核心对象
    # RTABMap 是整个 SLAM 系统的核心，负责处理传感器数据并维护地图
    slam = rtab.RTABMap()
    
    # 第二步：配置 SLAM 参数
    # 参数配置是 SLAM 系统性能的关键，需要根据具体应用场景调整
    params = rtab.Parameters()
    
    # 启用 RGB-D 模式，表示我们使用彩色图像+深度图像的数据
    params.set('RGBD/Enabled', 'true')
    
    # 设置时间阈值为700毫秒，这是两帧之间的最小时间间隔
    # 较大的时间阈值有助于减少计算负担，但可能影响实时性
    params.set('Rtabmap/TimeThr', '700')
    
    # 设置回环检测阈值为0.11，这个值决定了回环检测的敏感度
    # 较低的阈值会检测到更多回环，但可能产生误检测
    params.set('Rtabmap/LoopThr', '0.11')
    
    # 设置最大特征点数量为400，特征点用于图像匹配和位置估计
    # 更多特征点提供更好的精度，但增加计算成本
    params.set('Kp/MaxFeatures', '400')
    
    # 设置特征检测策略为6 (GFTT/BRIEF)
    # GFTT: Good Features To Track，BRIEF: Binary Robust Independent Elementary Features
    # 这种组合在计算效率和特征质量之间取得良好平衡
    params.set('Kp/DetectorStrategy', '6')
    
    # 第三步：初始化 SLAM 系统并指定数据库文件
    # 数据库文件用于持久化存储地图数据，支持地图的保存和加载
    database_path = "example_map.db"
    if slam.init(params.get_all_parameters(), database_path):
        print(f"✓ RTAB-Map 系统初始化成功，数据库文件: {database_path}")
    else:
        print("✗ RTAB-Map 系统初始化失败")
        return
        
    # 第四步：创建相机模型
    # 相机模型描述了相机的内参，这些参数对于准确的3D重建至关重要
    camera_model = rtab.CameraModel(
        name="example_camera",           # 相机名称
        fx=525.0, fy=525.0,             # 焦距参数 (像素单位)
        cx=320.0, cy=240.0,             # 主点坐标 (图像中心点)
        image_size=(640, 480)           # 图像分辨率 (宽x高)
    )
    print(f"✓ 相机模型创建成功，图像尺寸: {camera_model.image_size()}")
    
    # 第五步：模拟 RGB-D 数据处理
    # 在实际应用中，这些数据来自真实的 RGB-D 相机（如 Kinect、RealSense 等）
    print("\n开始处理模拟的 RGB-D 数据...")
    print("-" * 30)
    
    num_frames = 50                    # 处理的帧数
    loop_closures_detected = 0         # 检测到的回环闭合数量
    
    # 主要的 SLAM 处理循环
    for i in range(num_frames):
        # 生成合成的 RGB 图像
        # 在实际应用中，这将是来自相机的真实彩色图像
        rgb_image = generate_synthetic_rgb_image(640, 480, i)
        
        # 生成合成的深度图像
        # 深度图像提供每个像素到相机的距离信息（通常以毫米为单位）
        depth_image = generate_synthetic_depth_image(640, 480, i)
        
        # 创建传感器数据对象
        # SensorData 封装了 SLAM 系统需要的所有传感器信息
        sensor_data = rtab.SensorData(
            image=rgb_image,              # RGB 彩色图像
            depth=depth_image,            # 深度图像
            camera_model=camera_model,    # 相机内参模型
            image_id=i+1,                # 图像唯一标识符
            timestamp=time.time()         # 时间戳（用于数据同步）
        )
        
        # 模拟里程计位姿（向前移动并轻微旋转）
        # 在实际机器人中，这些数据来自轮式编码器、IMU或视觉里程计
        x = i * 0.1          # 每帧向前移动10厘米
        y = 0.0              # Y轴位置保持不变
        z = 0.0              # Z轴位置保持不变（地面移动）
        yaw = i * 0.02       # 每帧轻微旋转（模拟转弯）
        
        # 创建变换矩阵表示机器人的位姿
        # Transform 对象描述了机器人在3D空间中的位置和方向
        odometry_pose = rtab.Transform(x, y, z, 0, 0, yaw)
        
        # 使用 RTAB-Map 处理当前帧
        # 这是 SLAM 的核心步骤：处理传感器数据并更新地图
        start_time = time.time()
        added_to_map = slam.process(sensor_data, odometry_pose)
        process_time = (time.time() - start_time) * 1000  # 转换为毫秒
        
        # 获取 SLAM 系统的统计信息
        # 统计信息包含内存使用、处理时间等性能指标
        stats = slam.get_statistics()
        
        # 检查是否检测到回环闭合
        # 回环闭合是 SLAM 中的重要概念，表示机器人回到了之前访问过的位置
        loop_id = slam.get_loop_closure_id()           # 回环闭合的目标节点ID
        loop_value = slam.get_loop_closure_value()     # 回环闭合的置信度
        
        # 根据是否检测到回环闭合显示不同的信息
        if loop_id > 0:
            loop_closures_detected += 1
            print(f"帧 {i+1:3d}: 处理时间: {process_time:6.1f}ms, "
                  f"工作内存: {slam.get_working_memory().__len__():2d}, "
                  f"*回环 {slam.get_last_location_id()}→{loop_id}* (置信度: {loop_value:.3f})")
        else:
            print(f"帧 {i+1:3d}: 处理时间: {process_time:6.1f}ms, "
                  f"工作内存: {slam.get_working_memory().__len__():2d}, "
                  f"添加到地图: {'是' if added_to_map else '否'}")
        
        # 小延迟以模拟实时处理
        # 在实际应用中，这个延迟由相机帧率决定
        time.sleep(0.01)
    
    # 第六步：显示最终统计结果
    print("\n" + "=" * 50)
    print("SLAM 处理完成！")
    print(f"总处理帧数: {num_frames}")
    print(f"检测到的回环闭合: {loop_closures_detected}")
    print(f"最终工作内存大小: {len(slam.get_working_memory())}")
    print(f"最终位姿数量: {len(slam.get_optimized_poses())}")
    
    # 获取最终的性能统计信息
    # 这些统计数据有助于评估 SLAM 系统的性能
    final_stats = slam.get_statistics()
    performance_summary = final_stats.get_performance_summary()
    
    print("\n性能摘要:")
    print(f"  平均处理时间: {performance_summary['process_time_ms']:.1f} ms")
    print(f"  总移动距离: {performance_summary['distance_travelled_m']:.2f} m")
    print(f"  内存使用量: {performance_summary['ram_usage_mb']:.1f} MB")
    
    # 第七步：生成 DOT 图形文件用于可视化
    # DOT 图形显示了 SLAM 图的结构，包括节点和边的关系
    graph_file = "slam_graph.dot"
    if slam.generate_dot_graph(graph_file):
        print(f"✓ 图形文件已保存到: {graph_file}")
        print("  使用以下命令查看: neato -Tpdf slam_graph.dot -o slam_graph.pdf")
    
    # 第八步：保存轨迹数据
    # 轨迹文件包含了机器人的完整路径信息
    poses = slam.get_optimized_poses()
    if poses:
        trajectory_file = "trajectory.txt"
        rtab.Graph.export_poses(trajectory_file, 0, poses)
        print(f"✓ 轨迹文件已保存到: {trajectory_file}")
    
    # 第九步：关闭 RTAB-Map 系统并保存数据库
    # 正确关闭系统确保所有数据都被保存
    slam.close(database_saved=True)
    print("✓ RTAB-Map 系统已关闭，数据库已保存")


def generate_synthetic_rgb_image(width: int, height: int, frame_id: int) -> np.ndarray:
    """
    生成用于测试的合成 RGB 图像
    
    参数:
        width: 图像宽度（像素）
        height: 图像高度（像素）
        frame_id: 帧序号，用于创建随时间变化的图像内容
    
    返回:
        numpy.ndarray: BGR 格式的彩色图像（OpenCV 标准格式）
        
    说明:
        这个函数创建一个包含多种几何图形的合成图像，用于模拟真实相机的输出。
        图像中包含静态和动态元素，这对于测试 SLAM 系统的特征检测和跟踪能力很重要。
    """
    # 创建一个黑色的空白图像作为背景
    # shape: (高度, 宽度, 通道数)，dtype=uint8 表示每个像素值范围是 0-255
    image = np.zeros((height, width, 3), dtype=np.uint8)
    
    # 计算图像中心点坐标，用于放置几何图形
    center_x, center_y = width // 2, height // 2
    
    # 添加一个移动的绿色圆形
    # 使用三角函数创建平滑的椭圆运动轨迹
    circle_x = int(center_x + 100 * np.sin(frame_id * 0.1))  # X 坐标随正弦函数变化
    circle_y = int(center_y + 50 * np.cos(frame_id * 0.1))   # Y 坐标随余弦函数变化
    cv2.circle(image, (circle_x, circle_y), 30, (0, 255, 0), -1)  # 绿色实心圆
    
    # 添加一个静态的蓝色矩形
    # 静态特征为 SLAM 系统提供稳定的参考点
    cv2.rectangle(image, (50, 50), (150, 150), (255, 0, 0), -1)  # 蓝色实心矩形
    
    # 添加一条从左到右移动的红色竖线
    # 移动特征有助于测试特征跟踪算法
    line_offset = int(frame_id * 2) % width  # 线条位置循环移动
    cv2.line(image, (line_offset, 0), (line_offset, height), (0, 0, 255), 3)  # 红色竖线
    
    # 添加随机噪声以模拟真实相机的图像噪声
    # 噪声测试系统对不完美图像数据的鲁棒性
    noise = np.random.randint(0, 50, (height, width, 3), dtype=np.uint8)
    image = cv2.add(image, noise)  # 将噪声添加到图像中
    
    return image


def generate_synthetic_depth_image(width: int, height: int, frame_id: int) -> np.ndarray:
    """
    生成用于测试的合成深度图像
    
    参数:
        width: 图像宽度（像素）
        height: 图像高度（像素） 
        frame_id: 帧序号，用于创建随时间变化的深度内容
    
    返回:
        numpy.ndarray: 16位无符号整数深度图像，单位为毫米
        
    说明:
        深度图像是 RGB-D SLAM 的核心组成部分，每个像素值表示该点到相机的距离。
        深度信息使得 SLAM 系统能够：
        1. 直接获得3D点云信息，无需三角测量
        2. 提供尺度信息，解决单目视觉的尺度模糊问题
        3. 在纹理稀少的环境中仍能工作
        
        常见深度值范围：
        - 近距离：500-1000mm（0.5-1米）
        - 中距离：1000-3000mm（1-3米） 
        - 远距离：3000-5000mm（3-5米）
    """
    # 创建基础深度图像，默认深度为2米（2000毫米）
    # dtype=uint16 可以表示 0-65535 的深度值，足够覆盖常见的深度范围
    depth = np.ones((height, width), dtype=np.uint16) * 2000
    
    # 计算图像中心点，用于放置深度物体
    center_x, center_y = width // 2, height // 2
    
    # 添加一个移动的近距离物体（对应 RGB 图像中的绿色圆形）
    # 这个物体的深度为1米，比背景更近
    circle_x = int(center_x + 100 * np.sin(frame_id * 0.1))
    circle_y = int(center_y + 50 * np.cos(frame_id * 0.1))
    cv2.circle(depth, (circle_x, circle_y), 30, 1000, -1)  # 1000mm = 1m 深度
    
    # 添加一个静态的近距离矩形物体（对应 RGB 图像中的蓝色矩形）
    # 深度为1.5米，介于移动圆形和背景之间
    cv2.rectangle(depth, (50, 50), (150, 150), 1500, -1)  # 1500mm = 1.5m 深度
    
    # 添加深度渐变效果，模拟地面或倾斜表面
    # 图像下方的深度值逐渐增加，模拟相机向下倾斜观察地面的情况
    for y in range(height):
        depth[y, :] += int(y * 0.5)  # 每行深度增加0.5mm
    
    # 添加深度噪声以模拟真实深度传感器的测量误差
    # 深度传感器通常有±几毫米到几厘米的测量误差
    noise = np.random.randint(-100, 100, (height, width), dtype=np.int16)
    # 使用 clip 确保深度值在合理范围内（0.5米到5米）
    depth = np.clip(depth.astype(np.int32) + noise, 500, 5000).astype(np.uint16)
    
    return depth


if __name__ == "__main__":
    # 程序入口点：当脚本被直接运行时执行主函数
    # 这是 Python 的标准做法，确保代码只在直接运行时执行，而不在被导入时执行
    main()
