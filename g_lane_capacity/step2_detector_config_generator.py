#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import xml.etree.ElementTree as ET
from xml.dom import minidom
import os
from typing import Dict, List, Set, Optional
from dataclasses import dataclass
from datetime import datetime
from copy import deepcopy

# 导入Step1的数据结构
from step1_busline_intersection_analyzer import (
    BuslineIntersectionInfo,
    IntersectionApproachInfo,
    load_busline_intersection_info
)


# ============================================================================
# 常量定义
# ============================================================================

# 【V2.0关键更新】检测器默认位置
# 从 -1.0 改为 -3.5，确保能检测到排队车辆
# 物理依据：
#   - 小汽车长度：约 5 米
#   - 停车间距（jam distance）：约 2 米
#   - 停止线位置：pos = 0（车道末端）
#   - 第一辆排队车辆车头位置：约 -0.5 到 -1.0 米
#   - 第一辆排队车辆车身中部：约 -3.0 到 -3.5 米
# 因此 pos = -3.5 可确保检测器位于排队第一辆车的车身上
DEFAULT_DETECTOR_POSITION = -3.5


# ============================================================================
# 数据结构定义
# ============================================================================

@dataclass
class DetectorInfo:
    """检测器信息"""
    detector_id: str        # 检测器ID
    lane_id: str            # 车道ID
    edge_id: str            # 路段ID
    position: float         # 位置 (负数表示距离车道末端的距离)
    direction: str          # 流向
    buslines: List[str]     # 涉及的公交线路


# ============================================================================
# 检测器配置生成器
# ============================================================================

class DetectorConfigGenerator:
    """
    Instant E1检测器配置生成器 (V2.0)
    
    基于公交线路交叉口流向信息，生成Instant E1检测器配置
    
    【V2.0更新】
    - 默认检测器位置从 -1.0 改为 -3.5 米
    - 新增位置选择说明文档
    """
    
    def __init__(
        self,
        busline_info: Dict[str, BuslineIntersectionInfo],
        detector_position: float = DEFAULT_DETECTOR_POSITION,
        output_filename: str = "E1_instant_out.xml"
    ):
        """
        初始化检测器配置生成器
        
        Args:
            busline_info: 公交线路交叉口流向信息
            detector_position: 检测器位置 (默认-3.5米，停止线上游3.5米处)
            output_filename: 检测器数据输出文件名
        """
        self.busline_info = busline_info
        self.detector_position = detector_position
        self.output_filename = output_filename
        
        # 存储生成的检测器
        self.detectors: Dict[str, DetectorInfo] = {}  # lane_id -> DetectorInfo
        
        self._generate_detectors()
    
    def _generate_detectors(self):
        """生成所有需要的检测器"""
        print("\n[Step2] 生成检测器配置...")
        print(f"  【V2.0】检测器位置: pos={self.detector_position}m (停止线上游{-self.detector_position}米)")
        
        # 收集所有需要布设检测器的车道
        lane_buslines: Dict[str, Set[str]] = {}  # lane_id -> set of busline_ids
        lane_directions: Dict[str, str] = {}     # lane_id -> direction_group
        lane_edges: Dict[str, str] = {}          # lane_id -> edge_id
        
        for busline_id, info in self.busline_info.items():
            for inter in info.intersections:
                for lane_id, lane_info in inter.lane_all_directions.items():
                    if lane_id not in lane_buslines:
                        lane_buslines[lane_id] = set()
                    lane_buslines[lane_id].add(busline_id)
                    
                    # 保存流向和路段信息
                    lane_directions[lane_id] = lane_info.direction_group
                    lane_edges[lane_id] = inter.from_edge
        
        # 为每个车道生成检测器
        for lane_id in sorted(lane_buslines.keys()):
            # 生成检测器ID（使用车道ID作为基础）
            detector_id = f"det_{lane_id}"
            
            detector = DetectorInfo(
                detector_id=detector_id,
                lane_id=lane_id,
                edge_id=lane_edges[lane_id],
                position=self.detector_position,
                direction=lane_directions[lane_id],
                buslines=sorted(lane_buslines[lane_id])
            )
            
            self.detectors[lane_id] = detector
        
        print(f"  生成了 {len(self.detectors)} 个检测器")
    
    def save_to_xml(
        self,
        output_path: str,
        output_dir: str = None
    ) -> str:
        """
        保存检测器配置到XML文件
        
        Args:
            output_path: 输出文件路径
            output_dir: 检测器数据输出目录（可选）
            
        Returns:
            输出文件路径
        """
        root = ET.Element('additional')
        root.set('xmlns:xsi', 'http://www.w3.org/2001/XMLSchema-instance')
        root.set('xsi:noNamespaceSchemaLocation', 
                 'http://sumo.dlr.de/xsd/additional_file.xsd')
        
        # 添加注释
        comment = ET.Comment(f' Instant E1检测器配置文件 - 自动生成 (V2.0) ')
        root.append(comment)
        comment2 = ET.Comment(f' 生成时间: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")} ')
        root.append(comment2)
        comment3 = ET.Comment(f' 检测器位置: pos="{self.detector_position}" (停止线上游{-self.detector_position}米) ')
        root.append(comment3)
        comment4 = ET.Comment(f' 检测器数量: {len(self.detectors)} ')
        root.append(comment4)
        comment5 = ET.Comment(f' 【V2.0说明】位置-3.5米确保能检测到红灯排队车辆 ')
        root.append(comment5)
        
        # 确定输出文件路径
        if output_dir:
            file_path = os.path.join(output_dir, self.output_filename)
        else:
            # 使用与配置文件相同的目录
            config_dir = os.path.dirname(output_path)
            if config_dir:
                file_path = os.path.join(config_dir, self.output_filename)
            else:
                file_path = self.output_filename
        
        # 添加所有检测器
        for lane_id in sorted(self.detectors.keys()):
            detector = self.detectors[lane_id]
            
            elem = ET.Element('instantInductionLoop')
            elem.set('id', detector.detector_id)
            elem.set('lane', detector.lane_id)
            elem.set('pos', str(self.detector_position))
            elem.set('file', file_path)
            elem.set('friendlyPos', 'true')
            
            root.append(elem)
        
        # 格式化输出
        xml_str = ET.tostring(root, encoding='unicode')
        
        try:
            dom = minidom.parseString(xml_str)
            pretty_xml = dom.toprettyxml(indent="    ")
            lines = pretty_xml.split('\n')
            cleaned_lines = [line for line in lines if line.strip()]
            xml_str = '\n'.join(cleaned_lines)
        except Exception:
            pass
        
        # 确保输出目录存在
        output_dir_path = os.path.dirname(output_path)
        if output_dir_path:
            os.makedirs(output_dir_path, exist_ok=True)
        
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(xml_str)
        
        print(f"[Step2] 检测器配置已保存: {output_path}")
        return output_path
    
    def get_detector_count(self) -> int:
        """获取检测器数量"""
        return len(self.detectors)
    
    def get_detector_lanes(self) -> List[str]:
        """获取所有检测器对应的车道ID"""
        return list(self.detectors.keys())
    
    def get_detectors_for_edge(self, edge_id: str) -> List[DetectorInfo]:
        """获取某路段的所有检测器"""
        return [d for d in self.detectors.values() if d.edge_id == edge_id]


# ============================================================================
# 检测器配置处理器（用于修改输出文件名）
# ============================================================================

class DetectorConfigProcessor:
    """
    检测器配置处理器
    
    用于为不同实验修改检测器输出文件名，避免结果被覆盖
    """
    
    def __init__(self, detector_config_path: str):
        """
        初始化处理器
        
        Args:
            detector_config_path: 原始检测器配置文件路径
        """
        self.detector_config_path = detector_config_path
        self.tree = None
        self.root = None
        
        self._parse_config()
    
    def _parse_config(self):
        """解析检测器配置文件"""
        if not os.path.exists(self.detector_config_path):
            raise FileNotFoundError(f"检测器配置文件不存在: {self.detector_config_path}")
        
        self.tree = ET.parse(self.detector_config_path)
        self.root = self.tree.getroot()
    
    def create_modified_config(
        self,
        run_id: str,
        output_config_path: str,
        output_data_dir: str
    ) -> str:
        """
        创建修改后的检测器配置文件
        
        修改输出文件名，添加运行编号前缀
        
        Args:
            run_id: 运行编号 (如 "7X_run_001")
            output_config_path: 输出配置文件路径
            output_data_dir: 检测器数据输出目录
            
        Returns:
            输出配置文件路径
        """
        # 深拷贝根元素
        new_root = deepcopy(self.root)
        
        # 修改所有检测器的输出文件
        for elem in new_root.iter():
            if elem.tag == 'instantInductionLoop':
                original_file = elem.get('file', '')
                
                if original_file:
                    # 提取原始文件名
                    original_filename = os.path.basename(original_file)
                    
                    # 生成新的文件路径（添加run_id前缀）
                    new_filename = f"{run_id}_{original_filename}"
                    new_file_path = os.path.join(output_data_dir, new_filename)
                    
                    elem.set('file', new_file_path)
        
        # 格式化输出
        xml_str = ET.tostring(new_root, encoding='unicode')
        
        try:
            dom = minidom.parseString(xml_str)
            pretty_xml = dom.toprettyxml(indent="    ")
            lines = pretty_xml.split('\n')
            cleaned_lines = [line for line in lines if line.strip()]
            xml_str = '\n'.join(cleaned_lines)
        except Exception:
            pass
        
        # 确保输出目录存在
        output_dir = os.path.dirname(output_config_path)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)
        
        with open(output_config_path, 'w', encoding='utf-8') as f:
            f.write(xml_str)
        
        return output_config_path
    
    def get_output_data_filename(self) -> str:
        """获取检测器数据输出文件名"""
        for elem in self.root.iter():
            if elem.tag == 'instantInductionLoop':
                file_path = elem.get('file', '')
                if file_path:
                    return os.path.basename(file_path)
        return "E1_instant_out.xml"


# ============================================================================
# 便捷函数
# ============================================================================

def generate_detector_config(
    busline_intersection_xml: str,
    output_path: str,
    detector_position: float = DEFAULT_DETECTOR_POSITION,
    output_filename: str = "E1_instant_out.xml"
) -> str:
    """
    便捷函数：生成检测器配置文件
    
    Args:
        busline_intersection_xml: Step1生成的公交线路交叉口流向信息XML
        output_path: 输出文件路径
        detector_position: 检测器位置 (默认-3.5米)
        output_filename: 检测器数据输出文件名
        
    Returns:
        输出文件路径
    """
    # 加载公交线路交叉口流向信息
    busline_info = load_busline_intersection_info(busline_intersection_xml)
    
    # 生成检测器配置
    generator = DetectorConfigGenerator(
        busline_info=busline_info,
        detector_position=detector_position,
        output_filename=output_filename
    )
    
    return generator.save_to_xml(output_path)


def create_detector_config_for_run(
    original_config_path: str,
    run_id: str,
    output_config_path: str,
    output_data_dir: str
) -> str:
    """
    便捷函数：为特定实验创建检测器配置
    
    Args:
        original_config_path: 原始检测器配置文件路径
        run_id: 运行编号
        output_config_path: 输出配置文件路径
        output_data_dir: 检测器数据输出目录
        
    Returns:
        输出配置文件路径
    """
    processor = DetectorConfigProcessor(original_config_path)
    return processor.create_modified_config(run_id, output_config_path, output_data_dir)


# ============================================================================
# 主程序入口
# ============================================================================

if __name__ == "__main__":
    import sys
    
    print("=" * 70)
    print("Step 2: Instant E1检测器配置生成器 (V2.0 - 优化检测器位置)")
    print("=" * 70)
    print(f"\n【V2.0更新】检测器位置从 -1.0 改为 {DEFAULT_DETECTOR_POSITION} 米")
    print("  原因：确保能检测到红灯排队时停在检测器上的车辆")
    print("  物理依据：小汽车长度约5米，停车间距约2米")
    print(f"           -3.5米位于排队第一辆车的车身中部")
    
    if len(sys.argv) >= 3:
        busline_intersection_xml = sys.argv[1]
        output_path = sys.argv[2]
        
        result = generate_detector_config(
            busline_intersection_xml,
            output_path
        )
        
        print(f"\n生成完成！")
        print(f"输出文件: {result}")
    else:
        print("""
用法:
    python step2_detector_config_generator.py <busline_intersection_xml> <output_path>

参数:
    busline_intersection_xml: Step1生成的公交线路交叉口流向信息XML
    output_path: 输出文件路径

示例:
    python step2_detector_config_generator.py step1_busline_intersection_info.xml detectors.add.xml

编程调用:
    from step2_detector_config_generator import generate_detector_config
    
    output_path = generate_detector_config(
        'step1_busline_intersection_info.xml',
        'detectors.add.xml'
    )
""")
