#!/usr/bin/env python3
"""
测试RTAB-Map版本号获取功能
"""

import sys
import os

# 添加当前目录到Python路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import rtabmap_python as rtab
    
    print("=== RTAB-Map Python绑定版本测试 ===")
    print(f"模块版本: {rtab.__version__}")
    print(f"主版本号: {rtab.RTABMAP_MAJOR_VERSION}")
    print(f"次版本号: {rtab.RTABMAP_MINOR_VERSION}")
    print(f"补丁版本号: {rtab.RTABMAP_PATCH_VERSION}")
    
    # 测试Rtabmap类的getVersion方法
    try:
        slam = rtab.Rtabmap()
        print(f"Rtabmap.getVersion(): {slam.getVersion()}")
    except Exception as e:
        print(f"Rtabmap.getVersion() 测试失败: {e}")
    
    print("\n✅ 版本号获取测试完成！")
    
except ImportError as e:
    print(f"❌ 导入rtabmap_python失败: {e}")
    print("请先编译项目: python3 setup.py build_ext --inplace")
except Exception as e:
    print(f"❌ 测试失败: {e}")
