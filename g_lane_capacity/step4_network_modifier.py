#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import xml.etree.ElementTree as ET
from xml.dom import minidom
import os
import copy
from typing import Dict, List, Set, Tuple, Optional
from dataclasses import dataclass, field
from datetime import datetime
from collections import defaultdict

# 导入Step1的数据结构和函数
from step1_busline_intersection_analyzer import (
    BuslineIntersectionInfo,
    IntersectionApproachInfo,
    load_busline_intersection_info,
    normalize_direction
)


# ============================================================================
# 数据结构定义
# ============================================================================

@dataclass
class SignalCycleConfig:
    """
    信号灯周期配置
    
    控制全绿-全红周期的参数
    """
    num_cycles: int = 4              # 周期数量
    cycle_duration: float = 120.0    # 单个周期时长(秒)
    green_ratio: float = 0.6         # 全绿阶段占比
    
    @property
    def green_duration(self) -> float:
        """全绿阶段时长"""
        return self.cycle_duration * self.green_ratio
    
    @property
    def red_duration(self) -> float:
        """全红阶段时长"""
        return self.cycle_duration * (1 - self.green_ratio)
    
    @property
    def total_duration(self) -> float:
        """总时长"""
        return self.num_cycles * self.cycle_duration


@dataclass
class SignalModification:
    """
    单个信号灯的修改记录
    """
    tl_id: str                          # 信号灯ID
    original_phases: List[str]          # 原始相位状态列表
    green_state: str                    # 新的全绿状态
    red_state: str                      # 新的全红状态
    green_link_indices: List[int]       # 设为绿灯的linkIndex
    reason: str                         # 修改原因（目标交叉口/路径上游等）


@dataclass
class NetworkModificationResult:
    """
    路网修改结果
    """
    output_path: str                                    # 输出文件路径
    modifications: List[SignalModification]             # 所有信号灯的修改记录
    warnings: List[str] = field(default_factory=list)   # 警告信息
    
    def get_modified_tl_count(self) -> int:
        """获取修改的信号灯数量"""
        return len(self.modifications)


# ============================================================================
# 修改后路网生成器 (V2.1)
# ============================================================================

class ModifiedNetworkGeneratorV2:
    """
    修改后路网生成器 V2.1
    
    【核心类】生成修改后的路网文件，将路径上所有信号灯设置为全绿-全红周期
    
    V2.1改动：
    - 接收路径上所有信号灯的列表（从Step3获取）
    - 修改所有相关信号灯，而不只是目标交叉口的信号灯
    - 确保整条路径畅通无阻
    """
    
    def __init__(
        self, 
        net_file: str,
        cycle_config: SignalCycleConfig = None
    ):
        """
        初始化生成器
        
        Args:
            net_file: SUMO路网文件路径
            cycle_config: 信号灯周期配置
        """
        self.net_file = net_file
        self.cycle_config = cycle_config or SignalCycleConfig()
        self.tree = None
        self.root = None
        
        # 存储解析结果
        self.tl_phases: Dict[str, List[Tuple[int, str]]] = {}  # tl_id -> [(duration, state)]
        self.tl_state_length: Dict[str, int] = {}              # tl_id -> 状态字符串长度
        self.tl_elements: Dict[str, ET.Element] = {}           # tl_id -> XML元素
        
        self._parse_network()
    
    def _parse_network(self):
        """解析路网文件，提取信号灯配时信息"""
        if not os.path.exists(self.net_file):
            raise FileNotFoundError(f"路网文件不存在: {self.net_file}")
        
        self.tree = ET.parse(self.net_file)
        self.root = self.tree.getroot()
        
        # 解析所有信号灯配时
        for tl_logic in self.root.findall('tlLogic'):
            tl_id = tl_logic.get('id')
            phases = []
            
            for phase in tl_logic.findall('phase'):
                duration = int(float(phase.get('duration', 0)))
                state = phase.get('state', '')
                phases.append((duration, state))
            
            if phases:
                self.tl_phases[tl_id] = phases
                self.tl_state_length[tl_id] = len(phases[0][1])
                self.tl_elements[tl_id] = tl_logic
    
    def _generate_states(
        self,
        tl_id: str,
        green_link_indices: List[int]
    ) -> Tuple[Optional[str], Optional[str]]:
        """
        生成全绿和全红状态字符串
        
        Args:
            tl_id: 信号灯ID
            green_link_indices: 需要设为绿灯的linkIndex列表
            
        Returns:
            (全绿状态, 全红状态) 或 (None, None)
        """
        if tl_id not in self.tl_state_length:
            return None, None
        
        num_links = self.tl_state_length[tl_id]
        
        # 全红状态：所有位置都是红灯
        red_state = 'r' * num_links
        
        # 全绿状态：默认全红，指定位置设为绿灯
        green_list = ['r'] * num_links
        for idx in green_link_indices:
            if 0 <= idx < num_links:
                green_list[idx] = 'G'
        green_state = ''.join(green_list)
        
        return green_state, red_state
    
    def _modify_single_traffic_light(
        self,
        tl_logic: ET.Element,
        green_link_indices: List[int],
        reason: str
    ) -> Optional[SignalModification]:
        """
        修改单个信号灯的配时
        
        Args:
            tl_logic: 信号灯的XML元素
            green_link_indices: 需要设为绿灯的linkIndex列表
            reason: 修改原因
            
        Returns:
            SignalModification 或 None
        """
        tl_id = tl_logic.get('id')
        
        # 获取原始相位
        original_phases = [p.get('state', '') for p in tl_logic.findall('phase')]
        
        # 生成全绿和全红状态
        green_state, red_state = self._generate_states(tl_id, green_link_indices)
        
        if not green_state or not red_state:
            return None
        
        # 删除原有相位
        for phase in tl_logic.findall('phase'):
            tl_logic.remove(phase)
        
        # 添加全绿-全红周期相位
        green_duration = self.cycle_config.green_duration
        red_duration = self.cycle_config.red_duration
        
        for _ in range(self.cycle_config.num_cycles):
            # 全绿相位
            green_phase = ET.SubElement(tl_logic, 'phase')
            green_phase.set('duration', str(int(green_duration)))
            green_phase.set('state', green_state)
            
            # 全红相位
            red_phase = ET.SubElement(tl_logic, 'phase')
            red_phase.set('duration', str(int(red_duration)))
            red_phase.set('state', red_state)
        
        # 返回修改记录
        return SignalModification(
            tl_id=tl_id,
            original_phases=original_phases,
            green_state=green_state,
            red_state=red_state,
            green_link_indices=sorted(green_link_indices),
            reason=reason
        )
    
    def generate_modified_network_for_route(
        self,
        target_edge: str,
        target_direction: str,
        all_traffic_lights: List[Tuple[str, List[int]]],
        output_path: str
    ) -> NetworkModificationResult:
        """
        为单个路由生成修改后的路网文件
        
        【V2.1核心函数】修改路径上所有涉及的信号灯
        
        Args:
            target_edge: 目标进口道路段ID
            target_direction: 目标流向
            all_traffic_lights: 路径上所有信号灯及其全绿linkIndex
                格式: [(traffic_light_id, [green_link_indices]), ...]
            output_path: 输出文件路径
            
        Returns:
            NetworkModificationResult
        """
        print(f"\n[Step4] 为路由 {target_edge}->{target_direction} 生成修改后的路网...")
        print(f"  路径涉及 {len(all_traffic_lights)} 个信号灯")
        
        # 深拷贝路网
        new_root = copy.deepcopy(self.root)
        
        # 存储修改记录和警告
        modifications: List[SignalModification] = []
        warnings: List[str] = []
        
        # 创建信号灯ID到XML元素的映射（在新的根元素中）
        tl_elements_new = {}
        for tl_logic in new_root.findall('tlLogic'):
            tl_elements_new[tl_logic.get('id')] = tl_logic
        
        # 去重：同一个信号灯可能在列表中出现多次，合并其green_link_indices
        merged_tl: Dict[str, Set[int]] = defaultdict(set)
        for tl_id, green_indices in all_traffic_lights:
            if tl_id:
                merged_tl[tl_id].update(green_indices)
        
        # 修改路径上所有信号灯
        for i, (tl_id, green_indices_set) in enumerate(merged_tl.items()):
            if tl_id not in tl_elements_new:
                warnings.append(f"[警告] 信号灯 {tl_id} 不存在于路网中")
                continue
            
            green_indices = sorted(green_indices_set)
            reason = f"路径第{i+1}个信号灯"
            
            # 修改信号灯
            tl_logic = tl_elements_new[tl_id]
            mod = self._modify_single_traffic_light(tl_logic, green_indices, reason)
            
            if mod:
                modifications.append(mod)
                print(f"    修改信号灯 {tl_id}: "
                      f"{self.cycle_config.green_duration:.0f}s绿+{self.cycle_config.red_duration:.0f}s红, "
                      f"linkIndex={green_indices}")
            else:
                warnings.append(f"[警告] 无法为信号灯 {tl_id} 生成状态")
        
        # 保存修改后的路网
        xml_str = ET.tostring(new_root, encoding='unicode')
        
        # 确保输出目录存在
        output_dir = os.path.dirname(output_path)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)
        
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
            f.write(xml_str)
        
        print(f"  输出文件: {output_path}")
        print(f"  共修改 {len(modifications)} 个信号灯")
        
        if warnings:
            for w in warnings:
                print(f"  {w}")
        
        return NetworkModificationResult(
            output_path=output_path,
            modifications=modifications,
            warnings=warnings
        )
    
    def generate_modified_network_for_single_route(
        self,
        edge_id: str,
        direction: str,
        traffic_light: str,
        green_link_indices: List[int],
        output_path: str,
        additional_traffic_lights: List[Tuple[str, List[int]]] = None
    ) -> NetworkModificationResult:
        """
        为单个流向生成修改后的路网文件（兼容旧接口）
        
        Args:
            edge_id: 进口道路段ID
            direction: 流向
            traffic_light: 目标信号灯ID
            green_link_indices: 目标全绿linkIndex列表
            output_path: 输出文件路径
            additional_traffic_lights: 额外需要修改的信号灯列表
            
        Returns:
            NetworkModificationResult
        """
        # 构建完整的信号灯列表
        all_traffic_lights = []
        
        # 添加额外的信号灯（路径上游的）
        if additional_traffic_lights:
            all_traffic_lights.extend(additional_traffic_lights)
        
        # 添加目标信号灯
        if traffic_light:
            all_traffic_lights.append((traffic_light, green_link_indices))
        
        return self.generate_modified_network_for_route(
            target_edge=edge_id,
            target_direction=direction,
            all_traffic_lights=all_traffic_lights,
            output_path=output_path
        )


# 为了向后兼容，保留原来的类名
ModifiedNetworkGenerator = ModifiedNetworkGeneratorV2


# ============================================================================
# 便捷函数
# ============================================================================

def generate_modified_network_for_single_route(
    net_file: str,
    edge_id: str,
    direction: str,
    traffic_light: str,
    green_link_indices: List[int],
    output_path: str,
    signal_num_cycles: int = 4,
    signal_cycle_duration: float = 120.0,
    signal_green_ratio: float = 0.6,
    additional_traffic_lights: List[Tuple[str, List[int]]] = None
) -> NetworkModificationResult:
    """
    便捷函数：为单个流向生成修改后的路网文件
    
    Args:
        net_file: 原始SUMO路网文件路径
        edge_id: 进口道路段ID
        direction: 流向
        traffic_light: 目标信号灯ID
        green_link_indices: 目标全绿linkIndex列表
        output_path: 输出文件路径
        signal_num_cycles: 信号灯周期数量
        signal_cycle_duration: 单个周期时长
        signal_green_ratio: 全绿阶段占比
        additional_traffic_lights: 路径上额外需要修改的信号灯
        
    Returns:
        NetworkModificationResult
    """
    cycle_config = SignalCycleConfig(
        num_cycles=signal_num_cycles,
        cycle_duration=signal_cycle_duration,
        green_ratio=signal_green_ratio
    )
    
    generator = ModifiedNetworkGeneratorV2(net_file, cycle_config)
    return generator.generate_modified_network_for_single_route(
        edge_id=edge_id,
        direction=direction,
        traffic_light=traffic_light,
        green_link_indices=green_link_indices,
        output_path=output_path,
        additional_traffic_lights=additional_traffic_lights
    )


def generate_modified_network_for_busline(
    net_file: str,
    busline_intersection_xml: str,
    busline_id: str,
    output_path: str,
    signal_num_cycles: int = 4,
    signal_cycle_duration: float = 120.0,
    signal_green_ratio: float = 0.6
) -> NetworkModificationResult:
    """
    便捷函数：为指定公交线路生成修改后的路网文件（向后兼容）
    
    注意：此函数用于向后兼容，V2.1建议使用generate_modified_network_for_single_route
    
    Args:
        net_file: 原始SUMO路网文件路径
        busline_intersection_xml: Step1生成的公交线路交叉口流向信息XML
        busline_id: 公交线路ID
        output_path: 输出文件路径
        signal_num_cycles: 信号灯周期数量
        signal_cycle_duration: 单个周期时长
        signal_green_ratio: 全绿阶段占比
        
    Returns:
        NetworkModificationResult
    """
    # 加载公交线路交叉口流向信息
    all_busline_info = load_busline_intersection_info(busline_intersection_xml)
    
    if busline_id not in all_busline_info:
        raise ValueError(f"找不到公交线路 {busline_id} 的信息")
    
    busline_info = all_busline_info[busline_id]
    
    # 收集所有信号灯
    all_traffic_lights = []
    for inter in busline_info.intersections:
        if inter.traffic_light:
            all_traffic_lights.append((inter.traffic_light, inter.green_link_indices))
    
    # 配置
    cycle_config = SignalCycleConfig(
        num_cycles=signal_num_cycles,
        cycle_duration=signal_cycle_duration,
        green_ratio=signal_green_ratio
    )
    
    # 生成修改后的路网
    generator = ModifiedNetworkGeneratorV2(net_file, cycle_config)
    return generator.generate_modified_network_for_route(
        target_edge="busline_" + busline_id,
        target_direction="ALL",
        all_traffic_lights=all_traffic_lights,
        output_path=output_path
    )


# ============================================================================
# 主程序入口
# ============================================================================

if __name__ == "__main__":
    import sys
    
    print("=" * 70)
    print("Step 4: 修改后路网生成器 (V2.1)")
    print("=" * 70)
    print("""
V2.1版本更新:
    - 修改路径上【所有】涉及的信号灯
    - 所有信号灯都设置为"全绿-全红"周期变化
    - 确保车辆从上游到下游不会被红灯阻塞

用法:
    请通过main.py运行，或在代码中调用相关函数
""")
