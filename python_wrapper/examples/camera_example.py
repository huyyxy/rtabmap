#!/usr/bin/env python3
"""
RTAB-Map Python 包装器 - 相机接口示例

本示例详细演示了如何使用 RTAB-Map Python 包装器中的不同相机接口类型。
包含以下相机类型的使用方法：
1. RGB 相机 - 普通彩色相机
2. RGB-D 相机 - 彩色+深度相机（如 Kinect、RealSense）
3. 双目立体相机 - 通过两个相机计算深度
4. 图像序列 - 从文件夹中读取图像序列

学习目标：
- 理解不同相机类型的特点和应用场景
- 掌握相机初始化、数据获取和资源释放的标准流程
- 学会处理图像数据和深度信息
- 了解传感器数据的验证和错误处理方法
"""

import numpy as np      # 用于数值计算和数组操作
import cv2             # OpenCV 图像处理库
import time            # 用于添加延时和时间控制
import os              # 操作系统接口，用于文件和目录操作
import sys             # 系统相关参数和函数

# 将父目录添加到 Python 路径中，以便导入 rtabmap_python 模块
# 这是一种常见的技巧，用于导入相对路径中的自定义模块
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rtabmap_python as rtab


def rgb_camera_example():
    """
    RGB 相机使用示例
    
    演示如何使用普通的彩色（RGB）相机进行图像采集。
    RGB 相机是最常见的相机类型，只提供彩色图像信息，不包含深度数据。
    
    主要学习内容：
    1. 相机对象的创建和初始化
    2. 图像数据的获取和验证
    3. 传感器数据的基本属性访问
    4. 资源的正确释放
    """
    print("RGB 相机示例")
    print("-" * 20)
    
    # 步骤1：创建 RGB 相机对象
    # device_id=0 表示使用系统默认的第一个摄像头（通常是内置摄像头）
    # frame_rate=10.0 设置期望的帧率为每秒10帧
    camera = rtab.CameraRGB(device_id=0, frame_rate=10.0)
    
    # 步骤2：初始化相机
    # init() 方法会尝试连接相机硬件并进行配置
    # 返回 True 表示初始化成功，False 表示失败
    if not camera.init():
        print("❌ RGB 相机初始化失败")
        print("   可能原因：相机被其他程序占用，或者设备不存在")
        return
        
    print(f"✅ RGB 相机初始化成功")
    print(f"   相机型号: {camera.get_camera_model()}")
    
    # 步骤3：连续采集图像帧
    # 这里采集5帧图像来演示数据获取过程
    print("   开始采集图像...")
    for i in range(5):
        # take_image() 方法获取一帧图像数据
        # 返回 SensorData 对象，包含图像和相关元数据
        sensor_data = camera.take_image()
        
        # 步骤4：验证数据有效性
        # 在处理传感器数据前，必须检查数据是否有效
        if sensor_data.is_valid():
            # 获取原始图像数据（BGR格式的numpy数组）
            image = sensor_data.image_raw()
            print(f"   第 {i+1} 帧: 图像尺寸 {image.shape}, 数据ID: {sensor_data.id()}")
            
            # 步骤5：保存示例图像（仅保存第一帧）
            # 在实际应用中，可以在这里进行图像处理或分析
            if i == 0:
                cv2.imwrite("rgb_sample.jpg", image)
                print("       已保存示例图像: rgb_sample.jpg")
        else:
            print(f"   第 {i+1} 帧: 数据无效")
            print("       可能原因：相机断开连接或硬件故障")
            
        # 添加延时，控制采集频率
        time.sleep(0.1)
    
    # 步骤6：释放相机资源
    # 使用完毕后必须调用 close() 方法释放相机资源
    # 这样其他程序才能使用该相机
    camera.close()
    print("✅ RGB 相机已关闭\n")


def rgbd_camera_example():
    """
    RGB-D 相机使用示例
    
    演示如何使用 RGB-D（彩色+深度）相机进行数据采集。
    RGB-D 相机能够同时提供彩色图像和深度信息，常见设备包括：
    - Microsoft Kinect 系列
    - Intel RealSense 系列
    - Apple iPhone/iPad 的 TrueDepth 相机
    
    主要学习内容：
    1. RGB-D 相机的特殊配置参数（深度缩放因子）
    2. 同时获取彩色图像和深度图像
    3. 深度数据的分析和统计
    4. 深度图像的保存和可视化
    """
    print("RGB-D 相机示例")
    print("-" * 22)
    
    # 步骤1：创建 RGB-D 相机对象
    # device_id=0: 使用第一个 RGB-D 设备
    # frame_rate=10.0: 设置帧率为每秒10帧
    # depth_scale=1000.0: 深度缩放因子，将深度值转换为毫米单位
    #   例如：如果原始深度值是1.5，乘以1000后得到1500毫米（1.5米）
    camera = rtab.CameraRGBD(device_id=0, frame_rate=10.0, depth_scale=1000.0)
    
    # 步骤2：初始化相机
    if not camera.init():
        print("❌ RGB-D 相机初始化失败")
        print("   可能原因：")
        print("   - RGB-D 设备未连接或驱动未安装")
        print("   - 设备被其他程序占用")
        print("   - 系统不支持该类型的相机")
        return
        
    print(f"✅ RGB-D 相机初始化成功")
    print(f"   相机型号: {camera.get_camera_model()}")
    print(f"   深度缩放因子: {camera.get_depth_scale()}")
    
    # 步骤3：连续采集 RGB-D 数据
    print("   开始采集 RGB-D 数据...")
    for i in range(5):
        # 获取一帧 RGB-D 数据
        sensor_data = camera.take_image()
        
        if sensor_data.is_valid():
            # 步骤4：分别获取彩色图像和深度图像
            rgb = sensor_data.image_raw()      # 彩色图像（BGR格式）
            depth = sensor_data.depth_raw()    # 深度图像（单通道，深度值以毫米为单位）
            
            print(f"   第 {i+1} 帧:")
            print(f"      RGB 图像尺寸: {rgb.shape}")
            print(f"      深度图像尺寸: {depth.shape}")
            print(f"      数据ID: {sensor_data.id()}")
            
            # 步骤5：保存和分析第一帧数据
            if i == 0:
                # 保存彩色图像和深度图像
                cv2.imwrite("rgbd_rgb_sample.jpg", rgb)
                cv2.imwrite("rgbd_depth_sample.png", depth)
                print("      已保存示例图像:")
                print("        - rgbd_rgb_sample.jpg (彩色图像)")
                print("        - rgbd_depth_sample.png (深度图像)")
                
                # 步骤6：深度数据统计分析
                # 过滤掉无效的深度值（通常深度值为0表示无效）
                valid_depth = depth[depth > 0]
                if len(valid_depth) > 0:
                    print(f"      深度数据统计:")
                    print(f"        - 有效像素数量: {len(valid_depth)}")
                    print(f"        - 深度范围: {valid_depth.min():.0f} - {valid_depth.max():.0f} 毫米")
                    print(f"        - 平均深度: {valid_depth.mean():.1f} 毫米")
                    print(f"        - 最近物体距离: {valid_depth.min():.0f} 毫米")
                    print(f"        - 最远物体距离: {valid_depth.max():.0f} 毫米")
                else:
                    print("      警告: 未检测到有效的深度数据")
        else:
            print(f"   第 {i+1} 帧: 数据无效")
            print("       可能原因：传感器故障或数据传输错误")
            
        # 控制采集频率
        time.sleep(0.1)
    
    # 步骤7：释放资源
    camera.close()
    print("✅ RGB-D 相机已关闭\n")


def stereo_camera_example():
    """
    双目立体相机使用示例
    
    演示如何使用双目立体相机系统进行深度计算。
    双目立体相机通过两个相机的视差（disparity）来计算深度信息，
    模拟人眼的立体视觉原理。
    
    核心概念：
    - 基线距离（Baseline）：两个相机之间的距离，影响深度计算精度
    - 视差（Disparity）：同一物体在左右图像中的位置差异
    - 深度计算公式：depth = (focal_length × baseline) / disparity
    
    主要学习内容：
    1. 双目相机系统的配置参数
    2. 立体匹配和深度计算的原理
    3. 左右图像的同步获取
    4. 计算深度图与原始图像对的区别
    """
    print("双目立体相机示例")
    print("-" * 21)
    
    # 步骤1：创建双目立体相机对象
    camera = rtab.CameraStereo(
        device_id_left=0,      # 左相机设备ID
        device_id_right=1,     # 右相机设备ID
        frame_rate=10.0,       # 帧率设置
        baseline=0.12,         # 基线距离：12厘米（0.12米）
        compute_depth=True     # 是否自动计算深度图
    )
    
    print("   双目相机配置参数：")
    print(f"     - 左相机ID: 0")
    print(f"     - 右相机ID: 1") 
    print(f"     - 基线距离: 12 厘米")
    print(f"     - 自动深度计算: 启用")
    
    # 步骤2：初始化双目相机系统
    if not camera.init():
        print("❌ 双目立体相机初始化失败")
        print("   可能原因：")
        print("   - 左右相机设备未正确连接")
        print("   - 设备ID配置错误")
        print("   - 相机标定参数缺失")
        print("   - 系统资源不足")
        return
        
    print(f"✅ 双目立体相机初始化成功")
    print(f"   相机型号: {camera.get_camera_model()}")
    print(f"   实际基线距离: {camera.get_baseline():.3f} 米")
    print(f"   深度计算模式: {'开启' if camera._compute_depth else '关闭'}")
    
    # 步骤3：采集立体图像对
    print("   开始采集立体图像...")
    for i in range(3):  # 采集3帧数据进行演示
        sensor_data = camera.take_image()
        
        if sensor_data.is_valid():
            # 获取左相机图像（主图像）
            left = sensor_data.image_raw()
            
            # 步骤4：根据配置模式处理数据
            if camera._compute_depth:
                # 模式A：自动深度计算模式
                # 系统自动进行立体匹配并计算深度图
                depth = sensor_data.depth_raw()
                print(f"   第 {i+1} 帧 [深度计算模式]:")
                print(f"      左图像尺寸: {left.shape}")
                print(f"      计算深度图尺寸: {depth.shape}")
                print(f"      数据ID: {sensor_data.id()}")
                
                # 保存第一帧的图像和深度数据
                if i == 0:
                    cv2.imwrite("stereo_left_sample.jpg", left)
                    cv2.imwrite("stereo_depth_sample.png", depth)
                    print("      已保存文件:")
                    print("        - stereo_left_sample.jpg (左相机图像)")
                    print("        - stereo_depth_sample.png (计算的深度图)")
                    
                    # 分析深度数据质量
                    valid_depth = depth[depth > 0]
                    if len(valid_depth) > 0:
                        print(f"      深度计算结果:")
                        print(f"        - 有效深度像素: {len(valid_depth)}")
                        print(f"        - 深度范围: {valid_depth.min():.1f} - {valid_depth.max():.1f} 毫米")
                        print(f"        - 立体匹配成功率: {len(valid_depth)/depth.size*100:.1f}%")
            else:
                # 模式B：原始图像对模式
                # 获取左右两个原始图像，用户可以自行进行立体匹配
                right = sensor_data.depth_raw()  # 右图像存储在深度字段中
                print(f"   第 {i+1} 帧 [原始图像模式]:")
                print(f"      左图像尺寸: {left.shape}")
                print(f"      右图像尺寸: {right.shape}")
                print(f"      数据ID: {sensor_data.id()}")
                
                if i == 0:
                    cv2.imwrite("stereo_left_sample.jpg", left)
                    cv2.imwrite("stereo_right_sample.jpg", right)
                    print("      已保存立体图像对:")
                    print("        - stereo_left_sample.jpg (左相机图像)")
                    print("        - stereo_right_sample.jpg (右相机图像)")
        else:
            print(f"   第 {i+1} 帧: 数据无效")
            print("       可能原因：左右相机不同步或硬件故障")
            
        # 控制采集频率
        time.sleep(0.1)
    
    # 步骤5：释放双目相机资源
    camera.close()
    print("✅ 双目立体相机已关闭\n")


def image_sequence_example():
    """
    图像序列处理示例
    
    演示如何从文件夹中的图像序列创建虚拟相机。
    这种方法常用于：
    - 处理预录制的图像数据集
    - 离线测试和算法验证
    - 重现特定的场景和条件
    - 批量处理历史图像数据
    
    主要学习内容：
    1. 从图像文件夹创建相机对象
    2. 使用迭代器模式遍历图像序列
    3. 动态生成测试图像数据
    4. 资源管理和清理操作
    """
    print("图像序列处理示例")
    print("-" * 22)
    
    # 步骤1：创建测试图像数据
    sample_dir = "sample_images"
    print(f"   正在创建测试图像目录: {sample_dir}")
    create_sample_images(sample_dir)
    
    # 步骤2：从图像序列创建虚拟 RGB 相机
    # image_path: 指定包含图像文件的目录路径
    # frame_rate: 设置虚拟播放帧率（控制处理速度）
    camera = rtab.CameraRGB(image_path=sample_dir, frame_rate=5.0)
    
    # 步骤3：初始化图像序列相机
    if not camera.init():
        print("❌ 图像序列相机初始化失败")
        print("   可能原因：")
        print("   - 指定目录不存在或无访问权限")
        print("   - 目录中没有有效的图像文件")
        print("   - 图像格式不被支持")
        return
        
    print(f"✅ 图像序列相机初始化成功")
    print(f"   图像总数: {camera.get_image_count()} 张")
    print(f"   相机模型: {camera.get_camera_model()}")
    print(f"   虚拟帧率: 5.0 FPS")
    
    # 步骤4：使用迭代器遍历图像序列
    # 这种方法比手动循环更加优雅和高效
    print("   开始处理图像序列...")
    frame_count = 0
    
    # 使用 for 循环自动迭代所有图像
    # camera 对象实现了迭代器协议，可以直接用于 for 循环
    for sensor_data in camera:
        frame_count += 1
        
        # 获取当前图像数据
        image = sensor_data.image_raw()
        print(f"   处理第 {frame_count} 张图像:")
        print(f"      图像尺寸: {image.shape}")
        print(f"      数据ID: {sensor_data.id()}")
        print(f"      时间戳: {sensor_data.stamp()}")
        
        # 在实际应用中，这里可以进行图像处理、特征提取等操作
        # 例如：
        # - 目标检测
        # - 特征点提取
        # - SLAM 算法处理
        # - 图像增强
        
        # 限制输出数量以避免过多日志
        if frame_count >= 10:
            print(f"   ... (为简化输出，仅显示前10张图像)")
            break
    
    # 继续处理剩余图像（不显示详细信息）
    remaining_count = 0
    for sensor_data in camera:
        remaining_count += 1
        # 这里可以继续处理图像，但不输出详细信息
    
    total_processed = frame_count + remaining_count
    print(f"✅ 图像序列处理完成")
    print(f"   总计处理: {total_processed} 张图像")
    print(f"   详细显示: {frame_count} 张图像")
    
    # 步骤5：资源清理
    camera.close()
    cleanup_sample_images(sample_dir)
    print("✅ 已清理测试图像和相机资源\n")


def create_sample_images(directory: str, count: int = 20):
    """
    创建测试图像序列
    
    生成一系列用于演示的测试图像，每张图像包含：
    - 帧编号文字标识
    - 运动的绿色圆形（模拟动态场景）
    - 固定的图像尺寸和格式
    
    参数:
        directory (str): 保存图像的目录路径
        count (int): 要生成的图像数量，默认20张
        
    生成的图像特点：
    - 尺寸：240x320 像素（QVGA 分辨率）
    - 格式：BGR 彩色图像
    - 背景：黑色
    - 动态元素：圆形按正弦轨迹运动
    """
    # 创建目录（如果不存在的话）
    os.makedirs(directory, exist_ok=True)
    
    print(f"      正在生成 {count} 张测试图像...")
    
    for i in range(count):
        # 创建空白的黑色图像 (高度=240, 宽度=320, 通道数=3)
        # dtype=np.uint8 表示每个像素值范围为 0-255
        image = np.zeros((240, 320, 3), dtype=np.uint8)
        
        # 添加帧编号文字标识
        # 位置：(10, 30) 左上角
        # 字体：OpenCV 内置的 Hershey Simplex 字体
        # 大小：1.0 倍
        # 颜色：白色 (255, 255, 255)
        # 粗细：2 像素
        cv2.putText(image, f"Frame {i+1}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # 添加运动的绿色圆形
        # 使用三角函数创建椭圆形运动轨迹
        # 水平运动：中心160 + 振幅80 * sin(相位)
        # 垂直运动：中心120 + 振幅40 * cos(相位)
        # 相位随帧数递增，创建连续的运动效果
        center_x = int(160 + 80 * np.sin(i * 0.3))  # 水平位置
        center_y = int(120 + 40 * np.cos(i * 0.3))  # 垂直位置
        
        # 绘制实心圆形
        # 中心：(center_x, center_y)
        # 半径：20 像素
        # 颜色：绿色 (0, 255, 0) - 注意 OpenCV 使用 BGR 格式
        # 厚度：-1 表示填充整个圆形
        cv2.circle(image, (center_x, center_y), 20, (0, 255, 0), -1)
        
        # 保存图像文件
        # 文件名格式：image_001.jpg, image_002.jpg, ...
        # 使用 03d 格式确保文件名按字典序正确排序
        filename = os.path.join(directory, f"image_{i+1:03d}.jpg")
        cv2.imwrite(filename, image)
    
    print(f"      ✅ 成功生成 {count} 张测试图像")


def cleanup_sample_images(directory: str):
    """
    清理测试图像目录
    
    删除之前创建的测试图像目录及其所有内容。
    这是一个重要的清理步骤，避免在系统中留下临时文件。
    
    参数:
        directory (str): 要删除的目录路径
        
    安全特性：
    - 在删除前检查目录是否存在
    - 使用 shutil.rmtree() 安全删除整个目录树
    - 不会因为目录不存在而报错
    """
    if os.path.exists(directory):
        import shutil  # 导入文件操作工具库
        print(f"      正在清理目录: {directory}")
        
        # shutil.rmtree() 递归删除目录及其所有内容
        # 相当于 Linux/Mac 下的 "rm -rf" 命令
        shutil.rmtree(directory)
        print(f"      ✅ 已删除目录: {directory}")
    else:
        print(f"      ℹ️  目录不存在，无需清理: {directory}")


def main():
    """Main function to run all camera examples."""
    print("RTAB-Map Python Wrapper - Camera Examples")
    print("=" * 45)
    print()
    
    try:
        # Run examples
        rgb_camera_example()
        rgbd_camera_example() 
        stereo_camera_example()
        image_sequence_example()
        
        print("All camera examples completed successfully!")
        
    except KeyboardInterrupt:
        print("\nExamples interrupted by user")
    except Exception as e:
        print(f"Error running examples: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
