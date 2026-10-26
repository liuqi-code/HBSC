#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import xml.etree.ElementTree as ET
from xml.dom import minidom
import os
import random
from typing import Dict, List, Set, Tuple, Optional
from dataclasses import dataclass, field
from collections import defaultdict
from datetime import datetime

# 导入Step1的数据结构
from step1_busline_intersection_analyzer import (
    BuslineIntersectionInfo,
    IntersectionApproachInfo,
    LaneDirectionInfo,
    load_busline_intersection_info,
    normalize_direction,
    NetworkParser,
    BuslineParser
)


# ============================================================================
# 数据结构定义
# ============================================================================

@dataclass
class FlowPattern:
    """流量模式配置"""
    num_cycles: int = 2                 # 稀疏-拥堵循环次数
    cycle_duration: float = 1800.0      # 每个循环时长(秒)
    congestion_ratio: float = 0.5       # 拥堵阶段占比
    sparse_flow: float = 200.0          # 稀疏阶段流量 (veh/h/lane)
    congestion_flow: float = 2500.0     # 拥堵阶段流量 (veh/h/lane)
    transition_duration: float = 120.0  # 过渡时间(秒)
    warmup_time: float = 300.0          # 预热时间(秒)
    
    def get_total_duration(self) -> float:
        """获取总仿真时长"""
        return self.warmup_time + self.num_cycles * self.cycle_duration
    
    def generate_flow_profile(
        self, 
        lane_multiplier: int = 1
    ) -> List[Tuple[float, float, float]]:
        """
        生成流量剖面
        
        稀疏 -> 过渡 -> 拥堵 -> 过渡 -> 稀疏 ...
        
        Args:
            lane_multiplier: 车道数倍增因子
            
        Returns:
            [(begin, end, flow_rate), ...]
        """
        profile = []
        current_time = self.warmup_time
        
        actual_sparse = self.sparse_flow * lane_multiplier
        actual_congestion = self.congestion_flow * lane_multiplier
        
        for cycle in range(self.num_cycles):
            # 计算各阶段时长
            congestion_duration = self.cycle_duration * self.congestion_ratio
            sparse_duration = (self.cycle_duration - congestion_duration 
                              - 2 * self.transition_duration)
            
            # 1. 稀疏阶段前半段
            sparse_part1 = sparse_duration / 2
            profile.append((current_time, current_time + sparse_part1, actual_sparse))
            current_time += sparse_part1
            
            # 2. 过渡阶段（稀疏->拥堵）
            transition_steps = 5
            step_duration = self.transition_duration / transition_steps
            for i in range(transition_steps):
                ratio = (i + 1) / transition_steps
                flow = actual_sparse + (actual_congestion - actual_sparse) * ratio
                profile.append((current_time, current_time + step_duration, flow))
                current_time += step_duration
            
            # 3. 拥堵阶段
            profile.append((current_time, current_time + congestion_duration, 
                           actual_congestion))
            current_time += congestion_duration
            
            # 4. 过渡阶段（拥堵->稀疏）
            for i in range(transition_steps):
                ratio = 1 - (i + 1) / transition_steps
                flow = actual_sparse + (actual_congestion - actual_sparse) * ratio
                profile.append((current_time, current_time + step_duration, flow))
                current_time += step_duration
            
            # 5. 稀疏阶段后半段
            sparse_part2 = sparse_duration / 2
            profile.append((current_time, current_time + sparse_part2, actual_sparse))
            current_time += sparse_part2
        
        return profile


@dataclass
class VehicleTypeConfig:
    """车辆类型配置"""
    min_gap: float = 2.5
    length: float = 5.0
    tau: float = 1.3
    accel: float = 2.6
    decel: float = 4.5
    sigma: float = 0.5
    max_speed: float = 16.67
    speed_factor: float = 1.0
    
    # 大车参数调整因子
    heavy_length_factor: float = 2.4
    heavy_tau_factor: float = 1.5
    heavy_max_speed_factor: float = 0.8
    heavy_accel_factor: float = 0.7


@dataclass
class RouteGeneratorConfig:
    """路由生成器配置"""
    random_seed: int = 42
    heavy_ratio_min: float = 0.05
    heavy_ratio_max: float = 0.15
    friction_min: float = 0.8
    friction_max: float = 1.0
    flow_pattern: FlowPattern = field(default_factory=FlowPattern)
    vehicle_config: VehicleTypeConfig = field(default_factory=VehicleTypeConfig)
    
    # 上下游路段数量
    num_upstream_edges: int = 3     # 沿公交线路往前查找的路段数量
    num_downstream_edges: int = 2   # 沿公交线路往后查找的路段数量


@dataclass
class PathEdgeInfo:
    """
    路径上单个路段的信息
    
    记录路径上每个路段是否涉及信号灯，以及需要设置全绿的linkIndex
    """
    edge_id: str                    # 路段ID
    is_intersection_approach: bool  # 是否是交叉口进口道（有信号灯）
    traffic_light: str = ""         # 信号灯ID（如果有）
    direction: str = ""             # 流向（如果是交叉口进口道）
    green_link_indices: List[int] = field(default_factory=list)  # 需要全绿的linkIndex


@dataclass
class DirectionRouteInfo:
    """
    【V3.0新增】单个流向的路由信息
    
    用于在混合流向路由中存储每个流向的独立路由
    """
    direction: str                  # 流向 (L/T/R)
    route_id: str                   # 路由ID
    edges: List[str]                # 路段序列
    lane_count: int                 # 该流向可用的车道数
    to_edge: str                    # 目标下游路段
    green_link_indices: List[int]   # 该流向需要全绿的linkIndex
    # 【V3.1新增】该流向可用的车道索引列表
    lane_indices: List[int] = field(default_factory=list)


@dataclass
class MixedRouteInfo:
    """
    【V3.0核心数据结构】混合流向路由信息
    
    包含主流向和所有伴随流向的路由信息
    """
    # 基本信息
    group_id: str                   # 流向组ID (如 "E_J297_J321_130_T_with_L_R")
    target_edge: str                # 目标进口道路段（检测器所在的路段）
    primary_direction: str          # 公交线路的主流向
    
    # 流向组成
    all_directions: Set[str]        # 所有涉及的流向 (包括主流向和伴随流向)
    is_mixed: bool                  # 是否是混合流向（有伴随流向）
    
    # 各流向的路由信息
    direction_routes: Dict[str, DirectionRouteInfo] = field(default_factory=dict)
    
    # 路径信息（用于信号灯设置）
    path_intersections: List[PathEdgeInfo] = field(default_factory=list)
    
    # 汇总的全绿linkIndex（所有流向的合并）
    all_green_link_indices: List[int] = field(default_factory=list)
    
    def get_all_traffic_lights(self) -> List[Tuple[str, List[int]]]:
        """
        获取路径上所有涉及的信号灯及其需要全绿的linkIndex
        
        Returns:
            [(traffic_light_id, [green_link_indices]), ...]
        """
        result = []
        for edge_info in self.path_intersections:
            if edge_info.traffic_light and edge_info.green_link_indices:
                result.append((edge_info.traffic_light, edge_info.green_link_indices))
        return result
    
    def get_total_lane_count(self) -> int:
        """获取所有流向的总车道数"""
        return sum(dr.lane_count for dr in self.direction_routes.values())
    
    def get_direction_description(self) -> str:
        """获取流向描述字符串"""
        if self.is_mixed:
            companion = self.all_directions - {self.primary_direction}
            return f"{self.primary_direction}(+{'+'.join(sorted(companion))})"
        else:
            return self.primary_direction


@dataclass
class SingleRouteFile:
    """
    【V3.0更新】单个路由文件信息
    
    现在支持混合流向
    """
    route_id: str                   # 路由ID / 流向组ID
    busline_id: str                 # 公交线路ID
    edge_id: str                    # 目标进口道路段ID
    primary_direction: str          # 公交线路主流向
    traffic_light: str              # 目标信号灯ID
    green_link_indices: List[int]   # 所有需要全绿的linkIndex（合并后）
    file_path: str                  # 文件路径
    
    # V3.0新增：混合流向信息
    is_mixed: bool = False                              # 是否是混合流向
    all_directions: Set[str] = field(default_factory=set)  # 所有涉及的流向
    mixed_route_info: Optional[MixedRouteInfo] = None   # 混合路由详细信息
    
    # 路径上所有需要设置的信号灯
    all_traffic_lights: List[Tuple[str, List[int]]] = field(default_factory=list)


# ============================================================================
# 路由文件生成器 (V3.0)
# ============================================================================

class RouteFileGeneratorV3:
    """
    路由文件生成器 V3.0 - 混合流向版
    
    【核心类】根据车道共享情况生成单流向或混合流向路由文件
    
    V3.0核心改动：
    - 分析公交线路流向所在车道的共享情况
    - 专用车道：单独生成该流向的路由
    - 共享车道：同一路由文件中生成所有相关流向的车流
    """
    
    def __init__(
        self,
        net_file: str,
        busline_info: Dict[str, BuslineIntersectionInfo],
        busline_parser: BuslineParser,
        config: RouteGeneratorConfig = None
    ):
        """
        初始化路由生成器
        
        Args:
            net_file: SUMO路网文件路径
            busline_info: 公交线路交叉口流向信息（来自Step1）
            busline_parser: 公交线路解析器（用于获取线路路段序列）
            config: 路由生成器配置
        """
        self.net_file = net_file
        self.busline_info = busline_info
        self.busline_parser = busline_parser
        self.config = config or RouteGeneratorConfig()
        
        # 解析路网（用于获取车道数、连接关系等信息）
        self.network = NetworkParser(net_file)
        
        # 构建公交线路的路段序列索引
        self._build_busline_edge_sequences()
        
        # 构建路段到交叉口信息的映射
        self._build_edge_intersection_map()
        
        # 构建连接关系映射（用于查找各流向的下游路段）
        self._build_connection_map()
        
        # 实际使用的随机参数
        self.actual_heavy_ratio: float = 0.1
        self.actual_friction: float = 0.9
    
    def _build_busline_edge_sequences(self):
        """
        构建公交线路的路段序列
        
        从BuslineParser中提取每条公交线路的完整路段序列
        """
        self.busline_edge_sequences: Dict[str, List[str]] = {}
        
        for busline_id in self.busline_parser.get_busline_ids():
            elements = self.busline_parser.get_busline_elements(busline_id)
            # 提取所有路段ID（按顺序）
            edge_sequence = [elem['id'] for elem in elements if elem['id']]
            self.busline_edge_sequences[busline_id] = edge_sequence
    
    def _build_edge_intersection_map(self):
        """
        构建路段到交叉口信息的映射
        
        用于快速查找某个路段是否是交叉口进口道，以及相关的信号灯信息
        """
        self.edge_intersection_map: Dict[Tuple[str, str], IntersectionApproachInfo] = {}
        
        for busline_id, info in self.busline_info.items():
            for inter in info.intersections:
                key = (busline_id, inter.from_edge)
                self.edge_intersection_map[key] = inter
    
    def _build_connection_map(self):
        """
        【V3.0新增】构建连接关系映射
        
        用于查找各流向的下游路段
        格式: {(from_edge, direction): to_edge}
        """
        self.direction_to_edge_map: Dict[Tuple[str, str], str] = {}
        
        for conn in self.network.connections:
            # 标准化流向
            std_dir = normalize_direction(conn.direction)
            key = (conn.from_edge, std_dir)
            # 如果同一流向有多个下游路段，使用第一个
            if key not in self.direction_to_edge_map:
                self.direction_to_edge_map[key] = conn.to_edge
    
    def _randomize_parameters(self):
        """随机化参数"""
        random.seed(self.config.random_seed)
        
        self.actual_heavy_ratio = random.uniform(
            self.config.heavy_ratio_min,
            self.config.heavy_ratio_max
        )
        self.actual_friction = random.uniform(
            self.config.friction_min,
            self.config.friction_max
        )
    
    def _create_vehicle_types(self) -> ET.Element:
        """创建车辆类型定义"""
        vc = self.config.vehicle_config
        
        dist_elem = ET.Element('vTypeDistribution')
        dist_elem.set('id', 'mixed_traffic')
        
        # 小车类型
        car_elem = ET.SubElement(dist_elem, 'vType')
        car_elem.set('id', 'car')
        car_elem.set('length', f"{vc.length:.1f}")
        car_elem.set('minGap', f"{vc.min_gap:.1f}")
        car_elem.set('tau', f"{vc.tau * self.actual_friction:.2f}")
        car_elem.set('accel', f"{vc.accel:.1f}")
        car_elem.set('decel', f"{vc.decel:.1f}")
        car_elem.set('sigma', f"{vc.sigma:.2f}")
        car_elem.set('maxSpeed', f"{vc.max_speed:.2f}")
        car_elem.set('speedFactor', f"{vc.speed_factor:.2f}")
        car_elem.set('probability', f"{1 - self.actual_heavy_ratio:.4f}")
        
        # 大车类型
        truck_elem = ET.SubElement(dist_elem, 'vType')
        truck_elem.set('id', 'truck')
        truck_elem.set('length', f"{vc.length * vc.heavy_length_factor:.1f}")
        truck_elem.set('minGap', f"{vc.min_gap * 1.2:.1f}")
        truck_elem.set('tau', f"{vc.tau * vc.heavy_tau_factor * self.actual_friction:.2f}")
        truck_elem.set('accel', f"{vc.accel * vc.heavy_accel_factor:.1f}")
        truck_elem.set('decel', f"{vc.decel * 0.8:.1f}")
        truck_elem.set('sigma', f"{vc.sigma:.2f}")
        truck_elem.set('maxSpeed', f"{vc.max_speed * vc.heavy_max_speed_factor:.2f}")
        truck_elem.set('speedFactor', f"{vc.speed_factor:.2f}")
        truck_elem.set('probability', f"{self.actual_heavy_ratio:.4f}")
        
        return dist_elem
    
    def _find_edges_along_busline(
        self,
        busline_id: str,
        target_edge: str,
        num_upstream: int,
        num_downstream: int
    ) -> Tuple[List[str], List[str]]:
        """
        沿公交线路查找上下游路段
        
        Args:
            busline_id: 公交线路ID
            target_edge: 目标路段ID（检测器所在的进口道）
            num_upstream: 需要的上游路段数量
            num_downstream: 需要的下游路段数量
            
        Returns:
            (上游路段列表, 下游路段列表)
            上游路段列表按从远到近的顺序（即发车顺序）
            下游路段列表按从近到远的顺序
        """
        # 获取该公交线路的完整路段序列
        edge_sequence = self.busline_edge_sequences.get(busline_id, [])
        
        if not edge_sequence:
            return [], []
        
        # 查找目标路段在序列中的位置
        try:
            target_index = edge_sequence.index(target_edge)
        except ValueError:
            # 目标路段不在公交线路上，返回空
            return [], []
        
        # 查找上游路段（往前找）
        upstream_edges = []
        upstream_start = max(0, target_index - num_upstream)
        for i in range(upstream_start, target_index):
            upstream_edges.append(edge_sequence[i])
        
        # 查找下游路段（往后找）
        downstream_edges = []
        downstream_end = min(len(edge_sequence), target_index + 1 + num_downstream)
        for i in range(target_index + 1, downstream_end):
            downstream_edges.append(edge_sequence[i])
        
        return upstream_edges, downstream_edges
    
    def _collect_path_intersections(
        self,
        busline_id: str,
        edge_list: List[str],
        all_green_directions: Set[str] = None
    ) -> List[PathEdgeInfo]:
        """
        收集路径上所有路段的交叉口信息
        
        【V3.0更新】支持收集多个流向的green_link_indices
        
        Args:
            busline_id: 公交线路ID
            edge_list: 路段列表
            all_green_directions: 需要全绿的所有流向（如果为None，使用默认）
            
        Returns:
            PathEdgeInfo列表，包含每个路段是否有信号灯等信息
        """
        path_intersections = []
        
        for edge_id in edge_list:
            # 查找该路段是否是交叉口进口道
            key = (busline_id, edge_id)
            inter = self.edge_intersection_map.get(key)
            
            if inter:
                # 是交叉口进口道，记录信号灯信息
                # 使用传入的all_green_directions，或者使用交叉口自身的
                if all_green_directions:
                    # 计算需要全绿的linkIndex（所有指定流向的合并）
                    green_indices = set()
                    for lane_id, lane_info in inter.lane_all_directions.items():
                        for conn in lane_info.connections:
                            std_dir = normalize_direction(conn.direction)
                            if std_dir in all_green_directions and conn.link_index >= 0:
                                green_indices.add(conn.link_index)
                    green_link_indices = sorted(green_indices)
                else:
                    green_link_indices = list(inter.green_link_indices)
                
                path_intersections.append(PathEdgeInfo(
                    edge_id=edge_id,
                    is_intersection_approach=True,
                    traffic_light=inter.traffic_light,
                    direction=inter.primary_direction,
                    green_link_indices=green_link_indices
                ))
            else:
                # 不是交叉口进口道
                path_intersections.append(PathEdgeInfo(
                    edge_id=edge_id,
                    is_intersection_approach=False
                ))
        
        return path_intersections
    
    def _analyze_direction_sharing(
        self,
        busline_id: str,
        edge_id: str,
        primary_direction: str
    ) -> Tuple[Set[str], bool]:
        """
        【V3.0核心函数】分析流向共享情况
        
        分析公交线路主流向所在的车道是否与其他流向共享
        
        Args:
            busline_id: 公交线路ID
            edge_id: 进口道路段ID
            primary_direction: 公交线路的主流向 (L/T/R)
            
        Returns:
            (所有涉及的流向集合, 是否是混合流向)
        """
        key = (busline_id, edge_id)
        inter = self.edge_intersection_map.get(key)
        
        if not inter:
            return {primary_direction}, False
        
        # 收集主流向所在车道的所有流向
        all_directions = set()
        
        for lane_id, lane_info in inter.lane_all_directions.items():
            # 检查这个车道是否包含主流向
            if primary_direction in lane_info.directions:
                # 添加这个车道的所有流向
                all_directions.update(lane_info.directions)
        
        # 判断是否是混合流向
        is_mixed = len(all_directions) > 1
        
        return all_directions, is_mixed
    
    def _count_lanes_for_direction(
        self, 
        busline_id: str,
        edge_id: str, 
        direction: str
    ) -> int:
        """
        统计某路段某流向的可用车道数
        
        Args:
            busline_id: 公交线路ID
            edge_id: 路段ID
            direction: 流向
            
        Returns:
            可用车道数
        """
        key = (busline_id, edge_id)
        inter = self.edge_intersection_map.get(key)
        
        if not inter:
            return 1
        
        count = 0
        for lane_id, lane_info in inter.lane_all_directions.items():
            if direction in lane_info.directions:
                count += 1
        
        return max(count, 1)
    
    def _get_downstream_edge_for_direction(
        self,
        busline_id: str,
        from_edge: str,
        direction: str
    ) -> Optional[str]:
        """
        【V3.0新增】获取指定流向的下游路段
        
        Args:
            busline_id: 公交线路ID
            from_edge: 出发路段
            direction: 流向
            
        Returns:
            下游路段ID，如果找不到则返回None
        """
        # 先尝试从连接关系映射中查找
        key = (from_edge, direction)
        if key in self.direction_to_edge_map:
            return self.direction_to_edge_map[key]
        
        # 如果是主流向，使用公交线路的下游路段
        edge_sequence = self.busline_edge_sequences.get(busline_id, [])
        try:
            idx = edge_sequence.index(from_edge)
            if idx + 1 < len(edge_sequence):
                return edge_sequence[idx + 1]
        except ValueError:
            pass
        
        return None
    
    def _get_green_link_indices_for_direction(
        self,
        busline_id: str,
        edge_id: str,
        direction: str
    ) -> List[int]:
        """
        【V3.0新增】获取指定流向的green_link_indices
        
        Args:
            busline_id: 公交线路ID
            edge_id: 路段ID
            direction: 流向
            
        Returns:
            该流向对应的linkIndex列表
        """
        key = (busline_id, edge_id)
        inter = self.edge_intersection_map.get(key)
        
        if not inter:
            return []
        
        indices = set()
        for lane_id, lane_info in inter.lane_all_directions.items():
            if direction in lane_info.directions:
                for conn in lane_info.connections:
                    std_dir = normalize_direction(conn.direction)
                    if std_dir == direction and conn.link_index >= 0:
                        indices.add(conn.link_index)
        
        return sorted(indices)
    
    def _get_lane_indices_for_direction(
        self,
        busline_id: str,
        edge_id: str,
        direction: str
    ) -> List[int]:
        """
        【V3.1新增】获取指定流向可用的车道索引列表
        
        Args:
            busline_id: 公交线路ID
            edge_id: 路段ID
            direction: 流向
            
        Returns:
            该流向可用的车道索引列表（如[0, 1]表示车道0和车道1）
        """
        key = (busline_id, edge_id)
        inter = self.edge_intersection_map.get(key)
        
        if not inter:
            return [0]  # 默认返回车道0
        
        lane_indices = []
        for lane_id, lane_info in inter.lane_all_directions.items():
            if direction in lane_info.directions:
                lane_indices.append(lane_info.lane_index)
        
        return sorted(lane_indices) if lane_indices else [0]
    
    def _generate_mixed_route_info(
        self,
        busline_id: str,
        edge_id: str,
        primary_direction: str
    ) -> Optional[MixedRouteInfo]:
        """
        【V3.0核心函数】生成混合流向路由信息
        
        Args:
            busline_id: 公交线路ID
            edge_id: 进口道路段ID
            primary_direction: 公交线路的主流向
            
        Returns:
            MixedRouteInfo 或 None
        """
        # 1. 分析流向共享情况
        all_directions, is_mixed = self._analyze_direction_sharing(
            busline_id, edge_id, primary_direction
        )
        
        # 2. 查找上游路段（沿公交线路）
        upstream_edges, primary_downstream = self._find_edges_along_busline(
            busline_id=busline_id,
            target_edge=edge_id,
            num_upstream=self.config.num_upstream_edges,
            num_downstream=self.config.num_downstream_edges
        )
        
        if not primary_downstream:
            print(f"    [警告] 路段 {edge_id} 没有下游路段")
            return None
        
        # 3. 生成流向组ID
        if is_mixed:
            companion = sorted(all_directions - {primary_direction})
            safe_edge_id = edge_id.replace('.', '_')
            group_id = f"{safe_edge_id}_{primary_direction}_with_{'_'.join(companion)}"
        else:
            safe_edge_id = edge_id.replace('.', '_')
            group_id = f"{safe_edge_id}_{primary_direction}"
        
        # 4. 为每个流向生成路由信息
        direction_routes = {}
        all_green_indices = set()
        
        for direction in all_directions:
            # 获取该流向的下游路段
            if direction == primary_direction:
                # 主流向使用公交线路的下游路段
                downstream_edges = primary_downstream
            else:
                # 伴随流向需要查找对应的下游路段
                to_edge = self._get_downstream_edge_for_direction(
                    busline_id, edge_id, direction
                )
                if to_edge:
                    downstream_edges = [to_edge]
                else:
                    # 如果找不到，使用主流向的下游路段（作为fallback）
                    downstream_edges = primary_downstream[:1]
            
            # 构建完整路径
            route_edges = upstream_edges + [edge_id] + downstream_edges
            
            # 获取该流向的车道数
            lane_count = self._count_lanes_for_direction(busline_id, edge_id, direction)
            
            # 【V3.1新增】获取该流向的车道索引列表
            lane_indices = self._get_lane_indices_for_direction(busline_id, edge_id, direction)
            
            # 获取该流向的green_link_indices
            green_indices = self._get_green_link_indices_for_direction(
                busline_id, edge_id, direction
            )
            all_green_indices.update(green_indices)
            
            # 生成路由ID
            route_id = f"route_{safe_edge_id}_{direction}"
            
            direction_routes[direction] = DirectionRouteInfo(
                direction=direction,
                route_id=route_id,
                edges=route_edges,
                lane_count=lane_count,
                to_edge=downstream_edges[0] if downstream_edges else "",
                green_link_indices=green_indices,
                lane_indices=lane_indices  # 【V3.1新增】
            )
        
        # 5. 收集路径上所有交叉口信息
        # 使用主流向的路径（所有流向共用上游）
        all_edges = upstream_edges + [edge_id] + primary_downstream
        path_intersections = self._collect_path_intersections(
            busline_id, all_edges, all_directions
        )
        
        return MixedRouteInfo(
            group_id=group_id,
            target_edge=edge_id,
            primary_direction=primary_direction,
            all_directions=all_directions,
            is_mixed=is_mixed,
            direction_routes=direction_routes,
            path_intersections=path_intersections,
            all_green_link_indices=sorted(all_green_indices)
        )
    
    def _generate_flows_for_direction_route(
        self, 
        dir_route: DirectionRouteInfo,
        base_flow_id: str,
        target_lane_indices: List[int] = None
    ) -> List[ET.Element]:
        """
        【V3.1更新】为单个流向的路由生成流量定义
        
        实现车道均匀分配策略：
        - 发车时使用 departLane="free" 允许车辆自由选择车道
        - 使用 arrivalLane 指定到达路线终点时的目标车道
        - SUMO会根据arrivalLane的要求，让车辆提前变道到能够到达目标车道的位置
        - 这样在进入目标路段（进入交叉口的路段）时，车辆会均匀分配到各个目标车道
        
        Args:
            dir_route: 流向路由信息
            base_flow_id: 基础流量ID前缀
            target_lane_indices: 目标车道索引列表（用于均匀分配）
            
        Returns:
            流量XML元素列表
        """
        elements = []
        
        # 如果没有指定目标车道或只有一个车道，使用旧逻辑
        if not target_lane_indices or len(target_lane_indices) <= 1:
            # 获取流量剖面（乘以车道数）
            profile = self.config.flow_pattern.generate_flow_profile(dir_route.lane_count)
            
            for seg_idx, (begin, end, flow_rate) in enumerate(profile):
                flow_elem = ET.Element('flow')
                flow_elem.set('id', f"{base_flow_id}_{dir_route.direction}_{seg_idx}")
                flow_elem.set('route', dir_route.route_id)
                flow_elem.set('type', 'mixed_traffic')
                flow_elem.set('begin', f"{begin:.1f}")
                flow_elem.set('end', f"{end:.1f}")
                flow_elem.set('vehsPerHour', f"{flow_rate:.0f}")
                flow_elem.set('departLane', 'free')
                flow_elem.set('departSpeed', 'max')
                flow_elem.set('departPos', 'base')
                
                elements.append(flow_elem)
        else:
            # 【V3.1核心】为每个目标车道生成独立的flow，实现均匀分配
            # 获取单车道流量剖面（不乘以车道数）
            profile = self.config.flow_pattern.generate_flow_profile(1)
            
            for lane_idx in target_lane_indices:
                for seg_idx, (begin, end, flow_rate) in enumerate(profile):
                    flow_elem = ET.Element('flow')
                    flow_elem.set('id', f"{base_flow_id}_{dir_route.direction}_L{lane_idx}_{seg_idx}")
                    flow_elem.set('route', dir_route.route_id)
                    flow_elem.set('type', 'mixed_traffic')
                    flow_elem.set('begin', f"{begin:.1f}")
                    flow_elem.set('end', f"{end:.1f}")
                    flow_elem.set('vehsPerHour', f"{flow_rate:.0f}")
                    # 【V3.1关键】发车时自由选择车道
                    flow_elem.set('departLane', 'free')
                    flow_elem.set('departSpeed', 'max')
                    flow_elem.set('departPos', 'base')
                    # 【V3.1关键】指定到达路线终点时的目标车道
                    # SUMO会根据这个设置，让车辆提前变道到能够到达目标车道的位置
                    # 这样在进入目标路段时，车辆会自然分配到对应的车道
                    flow_elem.set('arrivalLane', str(lane_idx))
                    
                    elements.append(flow_elem)
        
        return elements
    
    def _write_mixed_route_file(
        self,
        mixed_route: MixedRouteInfo,
        output_path: str,
        busline_id: str
    ) -> str:
        """
        【V3.0核心函数】写入混合流向路由文件
        
        Args:
            mixed_route: 混合路由信息
            output_path: 输出文件路径
            busline_id: 公交线路ID
            
        Returns:
            输出文件路径
        """
        root = ET.Element('routes')
        root.set('xmlns:xsi', 'http://www.w3.org/2001/XMLSchema-instance')
        root.set('xsi:noNamespaceSchemaLocation', 
                 'http://sumo.dlr.de/xsd/routes_file.xsd')
        
        # 收集信息用于注释
        dir_desc = mixed_route.get_direction_description()
        tl_info_str = ""
        for edge_info in mixed_route.path_intersections:
            if edge_info.traffic_light:
                tl_info_str += f"\n      {edge_info.edge_id}: 信号灯={edge_info.traffic_light}, linkIdx={edge_info.green_link_indices}"
        
        # 各流向路由信息（V3.1更新：显示车道索引）
        routes_info_str = ""
        for direction, dr in sorted(mixed_route.direction_routes.items()):
            lanes_str = ','.join(str(i) for i in dr.lane_indices)
            routes_info_str += f"\n      {direction}: {' -> '.join(dr.edges)} (车道数={dr.lane_count}, 车道索引=[{lanes_str}])"
        
        # 添加注释
        comment = ET.Comment(f"""
    混合流向路由文件 (V3.1 - 车道均匀分配版)
    公交线路: {busline_id}
    目标进口道: {mixed_route.target_edge}
    主流向: {mixed_route.primary_direction}
    流向组合: {dir_desc}
    是否混合: {'是' if mixed_route.is_mixed else '否'}
    
    【V3.1车道均匀分配策略】
    - 每个流向的车辆将均匀分配到对应的目标车道
    - 发车时使用 departLane="free" 允许车辆自由选择车道
    - 使用 arrivalLane 指定到达路线终点时的目标车道
    - SUMO会根据arrivalLane的要求，让车辆提前变道
    - 这样在进入目标路段时，车辆会自然均匀分配到各个目标车道
    
    各流向路由:{routes_info_str}
    
    路径上的信号灯:{tl_info_str if tl_info_str else ' 无'}
    所有全绿linkIndex: {mixed_route.all_green_link_indices}
    
    生成时间: {datetime.now().isoformat()}
    随机种子: {self.config.random_seed}
    大车比例: {self.actual_heavy_ratio:.2%}
    摩擦系数: {self.actual_friction:.2f}
        """)
        root.append(comment)
        
        # 添加车辆类型
        root.append(self._create_vehicle_types())
        
        # 添加各流向的路由定义
        for direction in sorted(mixed_route.direction_routes.keys()):
            dr = mixed_route.direction_routes[direction]
            route_elem = ET.Element('route')
            route_elem.set('id', dr.route_id)
            route_elem.set('edges', ' '.join(dr.edges))
            root.append(route_elem)
        
        # 添加各流向的流量定义（V3.1：传递车道索引实现均匀分配）
        base_flow_id = f"flow_{mixed_route.group_id}"
        for direction in sorted(mixed_route.direction_routes.keys()):
            dr = mixed_route.direction_routes[direction]
            # 【V3.1关键】传递lane_indices，为每个车道生成独立的flow
            flow_elements = self._generate_flows_for_direction_route(
                dr, base_flow_id, target_lane_indices=dr.lane_indices
            )
            for flow_elem in flow_elements:
                root.append(flow_elem)
        
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
        output_dir = os.path.dirname(output_path)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)
        
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(xml_str)
        
        return output_path
    
    def generate_mixed_route(
        self,
        busline_id: str,
        edge_id: str,
        primary_direction: str,
        traffic_light: str,
        output_path: str,
        random_seed: int = None
    ) -> Optional[SingleRouteFile]:
        """
        【V3.0主要接口】生成混合流向的路由文件
        
        Args:
            busline_id: 公交线路ID
            edge_id: 进口道路段ID
            primary_direction: 公交线路的主流向
            traffic_light: 目标信号灯ID
            output_path: 输出文件路径
            random_seed: 随机种子（可选）
            
        Returns:
            SingleRouteFile 或 None
        """
        # 设置随机种子
        if random_seed is not None:
            self.config.random_seed = random_seed
        
        # 随机化参数
        self._randomize_parameters()
        
        # 生成混合路由信息
        mixed_route = self._generate_mixed_route_info(
            busline_id, edge_id, primary_direction
        )
        
        if not mixed_route:
            return None
        
        # 写入文件
        self._write_mixed_route_file(mixed_route, output_path, busline_id)
        
        # 获取路径上所有信号灯
        all_traffic_lights = mixed_route.get_all_traffic_lights()
        
        return SingleRouteFile(
            route_id=mixed_route.group_id,
            busline_id=busline_id,
            edge_id=edge_id,
            primary_direction=primary_direction,
            traffic_light=traffic_light,
            green_link_indices=mixed_route.all_green_link_indices,
            file_path=output_path,
            is_mixed=mixed_route.is_mixed,
            all_directions=mixed_route.all_directions,
            mixed_route_info=mixed_route,
            all_traffic_lights=all_traffic_lights
        )
    
    # ========== 向后兼容的旧接口 ==========
    
    def generate_single_route(
        self,
        busline_id: str,
        edge_id: str,
        direction: str,
        traffic_light: str,
        green_link_indices: List[int],
        output_path: str,
        random_seed: int = None
    ) -> Optional[SingleRouteFile]:
        """
        【向后兼容接口】生成路由文件
        
        V3.0自动转换为混合流向模式
        
        Args:
            busline_id: 公交线路ID
            edge_id: 进口道路段ID
            direction: 流向（作为主流向）
            traffic_light: 目标信号灯ID
            green_link_indices: 原始全绿linkIndex列表（可能会被扩展）
            output_path: 输出文件路径
            random_seed: 随机种子（可选）
            
        Returns:
            SingleRouteFile 或 None
        """
        return self.generate_mixed_route(
            busline_id=busline_id,
            edge_id=edge_id,
            primary_direction=direction,
            traffic_light=traffic_light,
            output_path=output_path,
            random_seed=random_seed
        )
    
    def get_generation_info(self) -> Dict:
        """获取生成信息"""
        return {
            'version': '3.0',
            'random_seed': self.config.random_seed,
            'heavy_ratio': self.actual_heavy_ratio,
            'friction': self.actual_friction,
            'sparse_flow': self.config.flow_pattern.sparse_flow,
            'congestion_flow': self.config.flow_pattern.congestion_flow,
            'num_cycles': self.config.flow_pattern.num_cycles,
            'total_duration': self.config.flow_pattern.get_total_duration(),
            'num_upstream_edges': self.config.num_upstream_edges,
            'num_downstream_edges': self.config.num_downstream_edges
        }


# 向后兼容别名
RouteFileGeneratorV2 = RouteFileGeneratorV3


# ============================================================================
# 便捷函数
# ============================================================================

def generate_routes_for_busline(
    net_file: str,
    busline_xml: str,
    busline_intersection_xml: str,
    busline_id: str,
    output_dir: str,
    random_seed: int = 42,
    sparse_flow: float = 200,
    congestion_flow: float = 2500,
    num_cycles: int = 2,
    num_upstream_edges: int = 3,
    num_downstream_edges: int = 2,
    **kwargs
) -> List[SingleRouteFile]:
    """
    便捷函数：为公交线路生成所有流向的路由文件
    
    【V3.0更新】自动处理混合流向
    
    Args:
        net_file: SUMO路网文件路径
        busline_xml: 公交线路XML文件路径
        busline_intersection_xml: Step1生成的公交线路交叉口流向信息XML
        busline_id: 公交线路ID
        output_dir: 输出目录
        random_seed: 随机种子
        sparse_flow: 稀疏阶段流量
        congestion_flow: 拥堵阶段流量
        num_cycles: 稀疏-拥堵循环次数
        num_upstream_edges: 上游路段数量
        num_downstream_edges: 下游路段数量
        
    Returns:
        SingleRouteFile列表
    """
    # 加载公交线路交叉口流向信息
    busline_info = load_busline_intersection_info(busline_intersection_xml)
    
    # 创建公交线路解析器
    busline_parser = BuslineParser(busline_xml)
    
    # 配置
    flow_pattern = FlowPattern(
        num_cycles=num_cycles,
        sparse_flow=sparse_flow,
        congestion_flow=congestion_flow
    )
    
    config = RouteGeneratorConfig(
        random_seed=random_seed,
        flow_pattern=flow_pattern,
        num_upstream_edges=num_upstream_edges,
        num_downstream_edges=num_downstream_edges
    )
    
    # 生成路由
    generator = RouteFileGeneratorV3(net_file, busline_info, busline_parser, config)
    
    # 获取该线路的交叉口信息
    info = busline_info.get(busline_id)
    if not info:
        return []
    
    route_files = []
    processed = set()
    
    for inter in info.intersections:
        # 【V3.0变更】只处理主流向，混合流向由generate_mixed_route自动处理
        primary_direction = inter.primary_direction
        key = (inter.from_edge, primary_direction)
        if key in processed:
            continue
        processed.add(key)
        
        safe_edge_id = inter.from_edge.replace('.', '_')
        filename = f"{busline_id}_{safe_edge_id}_{primary_direction}.rou.xml"
        filepath = os.path.join(output_dir, filename)
        
        route_file = generator.generate_mixed_route(
            busline_id=busline_id,
            edge_id=inter.from_edge,
            primary_direction=primary_direction,
            traffic_light=inter.traffic_light,
            output_path=filepath
        )
        
        if route_file:
            route_files.append(route_file)
    
    return route_files


# ============================================================================
# 主程序入口
# ============================================================================

if __name__ == "__main__":
    print("=" * 70)
    print("Step 3: 路由文件生成器 (V3.0 - 混合流向版)")
    print("=" * 70)
    print("""
V3.0版本核心更新:
    - 分析公交线路流向所在车道的共享情况
    - 专用车道：单独生成该流向的路由
    - 共享车道：同一路由文件中生成所有相关流向的车流
    
示例:
    路段E_J297_J321.130有两个车道：车道0(L+T)，车道1(R+T)
    当公交线路7S直行(T)通过时：
    - 旧版本：只生成T流向的车流
    - V3.0版本：同时生成L、T、R三个流向的车流

用法:
    请通过main.py运行，或在代码中调用 generate_routes_for_busline 函数
""")
