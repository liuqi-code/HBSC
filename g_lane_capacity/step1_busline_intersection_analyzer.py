#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import xml.etree.ElementTree as ET
from xml.dom import minidom
import os
from typing import Dict, List, Set, Tuple, Optional
from dataclasses import dataclass, field
from collections import defaultdict
from datetime import datetime


# ============================================================================
# 数据结构定义
# ============================================================================

@dataclass
class ConnectionInfo:
    """路网连接信息"""
    from_edge: str          # 来源路段
    to_edge: str            # 目标路段
    from_lane: int          # 来源车道索引
    to_lane: int            # 目标车道索引
    direction: str          # SUMO原始流向 (s/l/r/t等)
    traffic_light: str      # 信号灯ID
    link_index: int         # 信号灯linkIndex
    via: str = ""           # 内部边


@dataclass
class LaneDirectionInfo:
    """车道流向信息"""
    lane_id: str                    # 车道ID (如 E_J395_J396_2)
    lane_index: int                 # 车道索引
    directions: Set[str]            # 该车道的所有流向 (标准化后: L/T/R/U)
    direction_group: str            # 流向组合名称 (L/T/R/L+T/T+R等)
    connections: List[ConnectionInfo] = field(default_factory=list)  # 该车道的所有连接


@dataclass
class IntersectionApproachInfo:
    """交叉口进口道信息（公交线路涉及的）"""
    from_edge: str              # 进入交叉口前的路段
    to_edge: str                # 离开交叉口后的路段
    traffic_light: str          # 信号灯ID
    
    # 公交线路需要的主流向
    primary_direction: str      # 主流向 (L/T/R)
    
    # 可用于该流向的车道列表（从from_edge可以到to_edge的车道）
    available_lanes: List[str] = field(default_factory=list)
    
    # 因共用车道而关联的其他流向
    # key: 车道ID, value: 该车道的所有流向
    lane_all_directions: Dict[str, LaneDirectionInfo] = field(default_factory=dict)
    
    # 该进口道所有需要全绿的流向（包括共用车道带来的）
    all_green_directions: Set[str] = field(default_factory=set)
    
    # 需要全绿的linkIndex列表
    green_link_indices: List[int] = field(default_factory=list)


@dataclass 
class BuslineIntersectionInfo:
    """公交线路涉及的交叉口信息"""
    busline_id: str                                 # 公交线路ID
    intersections: List[IntersectionApproachInfo] = field(default_factory=list)
    
    def get_all_edges(self) -> Set[str]:
        """获取所有涉及的进口道路段"""
        return {inter.from_edge for inter in self.intersections}
    
    def get_all_traffic_lights(self) -> Set[str]:
        """获取所有涉及的信号灯"""
        return {inter.traffic_light for inter in self.intersections if inter.traffic_light}


# ============================================================================
# SUMO方向码标准化
# ============================================================================

# SUMO方向码到标准流向的映射
DIRECTION_MAP = {
    's': 'T',   # straight -> Through (直行)
    'l': 'L',   # left (左转)
    'L': 'L',   # partial left
    'r': 'R',   # right (右转)
    'R': 'R',   # partial right
    't': 'U',   # turn (掉头)
}

# 流向排序优先级（用于生成组合名称）
DIRECTION_ORDER = {'L': 0, 'T': 1, 'R': 2, 'U': 3}


def normalize_direction(sumo_dir: str) -> str:
    """将SUMO方向码转换为标准流向"""
    return DIRECTION_MAP.get(sumo_dir, 'T')


def get_direction_group(directions: Set[str]) -> str:
    """将流向集合转换为组合名称，如 'L+T', 'T+R'"""
    if not directions:
        return 'NONE'
    sorted_dirs = sorted(directions, key=lambda x: DIRECTION_ORDER.get(x, 99))
    return '+'.join(sorted_dirs)


# ============================================================================
# 路网解析器
# ============================================================================

class NetworkParser:
    """
    SUMO路网解析器
    
    解析路网XML，提取边、车道、连接和信号灯信息
    """
    
    def __init__(self, net_file: str):
        """
        初始化路网解析器
        
        Args:
            net_file: SUMO路网文件路径 (.net.xml)
        """
        self.net_file = net_file
        
        # 存储解析结果
        self.edges: Dict[str, dict] = {}                    # edge_id -> {info}
        self.lanes: Dict[str, dict] = {}                    # lane_id -> {info}
        self.connections: List[ConnectionInfo] = []         # 所有连接
        self.edge_connections: Dict[str, List[ConnectionInfo]] = defaultdict(list)  # edge -> connections
        self.lane_connections: Dict[str, List[ConnectionInfo]] = defaultdict(list)  # lane -> connections
        self.tl_phases: Dict[str, List[Tuple[int, str]]] = {}  # tl_id -> [(duration, state)]
        
        self._parse_network()
    
    def _parse_network(self):
        """解析路网XML文件"""
        if not os.path.exists(self.net_file):
            raise FileNotFoundError(f"路网文件不存在: {self.net_file}")
        
        print(f"[Step1] 解析路网文件: {self.net_file}")
        
        tree = ET.parse(self.net_file)
        root = tree.getroot()
        
        # 1. 解析所有edge和lane
        edge_count = 0
        lane_count = 0
        
        for edge in root.findall('edge'):
            edge_id = edge.get('id')
            
            # 跳过内部边（以:开头）
            if edge_id.startswith(':'):
                continue
            
            edge_count += 1
            
            self.edges[edge_id] = {
                'id': edge_id,
                'from': edge.get('from'),
                'to': edge.get('to'),
                'function': edge.get('function', 'normal'),
                'lanes': [],
                'num_lanes': 0
            }
            
            for lane in edge.findall('lane'):
                lane_id = lane.get('id')
                # 从lane_id提取索引，如 "E_J395_J396_2" -> 2
                lane_index = int(lane_id.split('_')[-1])
                lane_count += 1
                
                self.lanes[lane_id] = {
                    'id': lane_id,
                    'edge_id': edge_id,
                    'index': lane_index,
                    'length': float(lane.get('length', 0)),
                    'speed': float(lane.get('speed', 13.89)),
                    'allow': lane.get('allow', ''),
                    'disallow': lane.get('disallow', '')
                }
                self.edges[edge_id]['lanes'].append(lane_id)
            
            self.edges[edge_id]['num_lanes'] = len(self.edges[edge_id]['lanes'])
        
        # 2. 解析信号灯配时
        tl_count = 0
        for tl_logic in root.findall('tlLogic'):
            tl_id = tl_logic.get('id')
            phases = []
            for phase in tl_logic.findall('phase'):
                duration = int(float(phase.get('duration', 0)))
                state = phase.get('state', '')
                phases.append((duration, state))
            if phases:
                self.tl_phases[tl_id] = phases
                tl_count += 1
        
        # 3. 解析所有connection
        conn_count = 0
        for conn in root.findall('connection'):
            from_edge = conn.get('from')
            to_edge = conn.get('to')
            
            # 跳过内部边的连接
            if from_edge.startswith(':') or to_edge.startswith(':'):
                continue
            
            conn_count += 1
            
            from_lane = int(conn.get('fromLane', 0))
            to_lane = int(conn.get('toLane', 0))
            direction = conn.get('dir', 's')
            tl = conn.get('tl', '')
            link_index_str = conn.get('linkIndex', '')
            link_index = int(link_index_str) if link_index_str else -1
            
            conn_info = ConnectionInfo(
                from_edge=from_edge,
                to_edge=to_edge,
                from_lane=from_lane,
                to_lane=to_lane,
                direction=direction,
                traffic_light=tl,
                link_index=link_index,
                via=conn.get('via', '')
            )
            
            self.connections.append(conn_info)
            self.edge_connections[from_edge].append(conn_info)
            
            # 建立车道到连接的映射
            full_lane_id = f"{from_edge}_{from_lane}"
            self.lane_connections[full_lane_id].append(conn_info)
        
        print(f"  解析完成: {edge_count} 条边, {lane_count} 条车道, "
              f"{conn_count} 条连接, {tl_count} 个信号灯")
    
    def get_connections_between_edges(
        self, 
        from_edge: str, 
        to_edge: str
    ) -> List[ConnectionInfo]:
        """
        获取两条边之间的所有连接
        
        Args:
            from_edge: 来源路段ID
            to_edge: 目标路段ID
            
        Returns:
            连接信息列表
        """
        result = []
        for conn in self.edge_connections.get(from_edge, []):
            if conn.to_edge == to_edge:
                result.append(conn)
        return result
    
    def get_lane_all_connections(self, lane_id: str) -> List[ConnectionInfo]:
        """
        获取某车道的所有出向连接
        
        Args:
            lane_id: 车道ID (如 E_J395_J396_2)
            
        Returns:
            该车道的所有连接
        """
        return self.lane_connections.get(lane_id, [])
    
    def get_lane_directions(self, lane_id: str) -> Set[str]:
        """
        获取某车道的所有流向（标准化后）
        
        Args:
            lane_id: 车道ID
            
        Returns:
            流向集合 (如 {'T', 'R'})
        """
        directions = set()
        for conn in self.lane_connections.get(lane_id, []):
            directions.add(normalize_direction(conn.direction))
        return directions
    
    def get_edge_lane_count(self, edge_id: str) -> int:
        """获取路段的车道数"""
        if edge_id in self.edges:
            return self.edges[edge_id]['num_lanes']
        return 0
    
    def get_signal_state_length(self, tl_id: str) -> int:
        """获取信号灯状态字符串长度（即link数量）"""
        if tl_id in self.tl_phases and self.tl_phases[tl_id]:
            return len(self.tl_phases[tl_id][0][1])
        return 0


# ============================================================================
# 公交线路解析器
# ============================================================================

class BuslineParser:
    """
    公交线路XML解析器
    
    解析公交线路XML文件，提取各线路的路段序列
    """
    
    def __init__(self, busline_xml: str):
        """
        初始化公交线路解析器
        
        Args:
            busline_xml: 公交线路XML文件路径
        """
        self.busline_xml = busline_xml
        self.buslines: Dict[str, List[dict]] = {}  # busline_id -> [elements]
        
        self._parse_busline_xml()
    
    def _parse_busline_xml(self):
        """解析公交线路XML文件"""
        if not os.path.exists(self.busline_xml):
            raise FileNotFoundError(f"公交线路XML文件不存在: {self.busline_xml}")
        
        print(f"[Step1] 解析公交线路文件: {self.busline_xml}")
        
        tree = ET.parse(self.busline_xml)
        root = tree.getroot()
        
        # 遍历所有公交线路
        for busline in root.findall('.//busline'):
            busline_id = busline.get('id', 'unknown')
            elements = []
            
            # 遍历线路中的所有路段元素
            for element in busline.findall('element'):
                elem_info = {
                    'id': element.get('id', ''),
                    'type': element.get('type', ''),
                    'length': float(element.get('length', 0)),
                    'dedicated_flag': int(element.get('dedicated_flag', 0)),
                    'tl': element.get('tl', ''),
                    'phase_time': float(element.get('phase_time', 0)) if element.get('phase_time') else 0,
                    'cycle_time': float(element.get('cycle_time', 0)) if element.get('cycle_time') else 0,
                    'from_lane': element.get('from_lane', ''),
                    'dir': element.get('dir', ''),
                    'impacted_edge': element.get('impacted_edge', ''),
                    'impacted_length': float(element.get('impacted_length', 0)) if element.get('impacted_length') else 0,
                    'stop_id': element.get('stop_id', '')
                }
                elements.append(elem_info)
            
            if elements:
                self.buslines[busline_id] = elements
        
        print(f"  解析完成: {len(self.buslines)} 条公交线路")
    
    def get_busline_ids(self) -> List[str]:
        """获取所有公交线路ID"""
        return list(self.buslines.keys())
    
    def get_busline_elements(self, busline_id: str) -> List[dict]:
        """获取指定公交线路的路段元素列表"""
        return self.buslines.get(busline_id, [])
    
    def get_intersection_approaches(self, busline_id: str) -> List[Tuple[str, str, str, str]]:
        """
        获取公交线路涉及的交叉口进口道信息
        
        查找type="i"的路段，以及它的下一个路段
        
        Args:
            busline_id: 公交线路ID
            
        Returns:
            [(from_edge, to_edge, tl_id, dir), ...]
        """
        elements = self.buslines.get(busline_id, [])
        approaches = []
        
        for i, elem in enumerate(elements):
            if elem['type'] == 'i':  # 进入交叉口的路段
                from_edge = elem['id']
                tl_id = elem['tl']
                direction = elem['dir']
                
                # 获取下一个路段
                if i + 1 < len(elements):
                    to_edge = elements[i + 1]['id']
                    approaches.append((from_edge, to_edge, tl_id, direction))
        
        return approaches


# ============================================================================
# 公交线路交叉口流向分析器
# ============================================================================

class BuslineIntersectionAnalyzer:
    """
    公交线路交叉口流向分析器
    
    【核心类】分析公交线路涉及的交叉口流向关系，包括：
    1. 确定公交车从路段1到路段2需要使用哪些车道
    2. 检测共用车道情况（一个车道同时服务多个流向）
    3. 汇总所有需要设置全绿的流向和linkIndex
    """
    
    def __init__(self, network: NetworkParser, busline_parser: BuslineParser):
        """
        初始化分析器
        
        Args:
            network: 路网解析器
            busline_parser: 公交线路解析器
        """
        self.network = network
        self.busline_parser = busline_parser
        
        # 分析结果
        self.busline_info: Dict[str, BuslineIntersectionInfo] = {}
    
    def analyze_all_buslines(self) -> Dict[str, BuslineIntersectionInfo]:
        """
        分析所有公交线路的交叉口流向信息
        
        Returns:
            {busline_id: BuslineIntersectionInfo}
        """
        print("\n[Step1] 分析公交线路交叉口流向...")
        
        for busline_id in self.busline_parser.get_busline_ids():
            info = self._analyze_single_busline(busline_id)
            if info.intersections:
                self.busline_info[busline_id] = info
        
        # 打印摘要
        total_intersections = sum(len(info.intersections) for info in self.busline_info.values())
        print(f"\n  分析完成: {len(self.busline_info)} 条公交线路, "
              f"共 {total_intersections} 个交叉口进口道")
        
        return self.busline_info
    
    def _analyze_single_busline(self, busline_id: str) -> BuslineIntersectionInfo:
        """
        分析单条公交线路的交叉口流向
        
        Args:
            busline_id: 公交线路ID
            
        Returns:
            BuslineIntersectionInfo
        """
        info = BuslineIntersectionInfo(busline_id=busline_id)
        
        # 获取该线路涉及的交叉口进口道
        approaches = self.busline_parser.get_intersection_approaches(busline_id)
        
        for from_edge, to_edge, tl_id, raw_direction in approaches:
            approach_info = self._analyze_intersection_approach(
                from_edge, to_edge, tl_id, raw_direction
            )
            if approach_info:
                info.intersections.append(approach_info)
        
        return info
    
    def _analyze_intersection_approach(
        self,
        from_edge: str,
        to_edge: str,
        tl_id: str,
        raw_direction: str
    ) -> Optional[IntersectionApproachInfo]:
        """
        分析单个交叉口进口道的流向信息
        
        【核心函数】
        
        Args:
            from_edge: 进入交叉口前的路段
            to_edge: 离开交叉口后的路段
            tl_id: 信号灯ID
            raw_direction: 原始流向（来自公交线路XML）
            
        Returns:
            IntersectionApproachInfo 或 None
        """
        # 1. 获取from_edge到to_edge的所有连接
        connections = self.network.get_connections_between_edges(from_edge, to_edge)
        
        if not connections:
            print(f"    [警告] 找不到 {from_edge} -> {to_edge} 的连接")
            return None
        
        # 2. 标准化主流向
        primary_direction = normalize_direction(raw_direction) if raw_direction else 'T'
        
        # 3. 找出可用于该流向的车道（从from_edge可以到to_edge的车道）
        available_lanes = []
        lane_indices = set()
        
        for conn in connections:
            lane_id = f"{from_edge}_{conn.from_lane}"
            if lane_id not in available_lanes:
                available_lanes.append(lane_id)
                lane_indices.add(conn.from_lane)
        
        # 4. 分析每个可用车道的所有流向（检测共用车道）
        lane_all_directions: Dict[str, LaneDirectionInfo] = {}
        all_green_directions = set()
        green_link_indices = []
        
        for lane_id in available_lanes:
            # 获取该车道的所有出向连接
            all_conns = self.network.get_lane_all_connections(lane_id)
            
            # 收集该车道的所有流向
            directions = set()
            for conn in all_conns:
                norm_dir = normalize_direction(conn.direction)
                directions.add(norm_dir)
                
                # 收集需要全绿的linkIndex
                if conn.link_index >= 0:
                    green_link_indices.append(conn.link_index)
            
            # 保存车道流向信息
            lane_info = LaneDirectionInfo(
                lane_id=lane_id,
                lane_index=int(lane_id.split('_')[-1]),
                directions=directions,
                direction_group=get_direction_group(directions),
                connections=all_conns
            )
            lane_all_directions[lane_id] = lane_info
            
            # 累积所有需要全绿的流向
            all_green_directions.update(directions)
        
        # 5. 去重linkIndex
        green_link_indices = sorted(set(green_link_indices))
        
        # 创建进口道信息对象
        approach_info = IntersectionApproachInfo(
            from_edge=from_edge,
            to_edge=to_edge,
            traffic_light=tl_id,
            primary_direction=primary_direction,
            available_lanes=available_lanes,
            lane_all_directions=lane_all_directions,
            all_green_directions=all_green_directions,
            green_link_indices=green_link_indices
        )
        
        return approach_info
    
    def save_to_xml(self, output_path: str) -> str:
        """
        保存分析结果到XML文件
        
        Args:
            output_path: 输出文件路径
            
        Returns:
            输出文件路径
        """
        root = ET.Element('busline_intersection_info')
        root.set('generated_at', datetime.now().isoformat())
        root.set('busline_count', str(len(self.busline_info)))
        
        # 统计摘要
        total_intersections = sum(len(info.intersections) for info in self.busline_info.values())
        root.set('total_intersections', str(total_intersections))
        
        # 遍历所有公交线路
        for busline_id in sorted(self.busline_info.keys()):
            busline_info = self.busline_info[busline_id]
            
            busline_elem = ET.SubElement(root, 'busline')
            busline_elem.set('id', busline_id)
            busline_elem.set('intersection_count', str(len(busline_info.intersections)))
            
            # 遍历该线路的所有交叉口进口道
            for inter in busline_info.intersections:
                inter_elem = ET.SubElement(busline_elem, 'intersection')
                inter_elem.set('from_edge', inter.from_edge)
                inter_elem.set('to_edge', inter.to_edge)
                inter_elem.set('traffic_light', inter.traffic_light)
                inter_elem.set('primary_direction', inter.primary_direction)
                inter_elem.set('all_green_directions', '+'.join(sorted(inter.all_green_directions)))
                inter_elem.set('green_link_indices', ','.join(map(str, inter.green_link_indices)))
                inter_elem.set('available_lane_count', str(len(inter.available_lanes)))
                
                # 可用车道列表
                lanes_elem = ET.SubElement(inter_elem, 'available_lanes')
                lanes_elem.text = ','.join(inter.available_lanes)
                
                # 每个车道的详细流向信息
                lane_details_elem = ET.SubElement(inter_elem, 'lane_details')
                for lane_id, lane_info in inter.lane_all_directions.items():
                    lane_elem = ET.SubElement(lane_details_elem, 'lane')
                    lane_elem.set('id', lane_id)
                    lane_elem.set('index', str(lane_info.lane_index))
                    lane_elem.set('direction_group', lane_info.direction_group)
                    lane_elem.set('directions', '+'.join(sorted(lane_info.directions)))
                    
                    # 连接详情
                    for conn in lane_info.connections:
                        conn_elem = ET.SubElement(lane_elem, 'connection')
                        conn_elem.set('to_edge', conn.to_edge)
                        conn_elem.set('to_lane', str(conn.to_lane))
                        conn_elem.set('dir', conn.direction)
                        conn_elem.set('normalized_dir', normalize_direction(conn.direction))
                        if conn.traffic_light:
                            conn_elem.set('tl', conn.traffic_light)
                        if conn.link_index >= 0:
                            conn_elem.set('linkIndex', str(conn.link_index))
        
        # 格式化输出
        xml_str = ET.tostring(root, encoding='unicode')
        dom = minidom.parseString(xml_str)
        pretty_xml = dom.toprettyxml(indent="    ")
        
        # 清理空行
        lines = pretty_xml.split('\n')
        cleaned_lines = [line for line in lines if line.strip()]
        
        # 确保输出目录存在
        output_dir = os.path.dirname(output_path)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)
        
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(cleaned_lines))
        
        print(f"\n[Step1] 分析结果已保存: {output_path}")
        return output_path
    
    def get_busline_info(self, busline_id: str) -> Optional[BuslineIntersectionInfo]:
        """获取指定公交线路的交叉口信息"""
        return self.busline_info.get(busline_id)
    
    def get_all_busline_ids(self) -> List[str]:
        """获取所有公交线路ID"""
        return list(self.busline_info.keys())
    
    def print_summary(self):
        """打印分析摘要"""
        print("\n" + "=" * 70)
        print("公交线路交叉口流向分析摘要")
        print("=" * 70)
        
        total_intersections = 0
        shared_lane_count = 0
        
        for busline_id, info in sorted(self.busline_info.items()):
            total_intersections += len(info.intersections)
            
            print(f"\n  公交线路 {busline_id}:")
            print(f"    交叉口数: {len(info.intersections)}")
            
            for inter in info.intersections:
                # 检测共用车道
                has_shared = any(
                    len(lane_info.directions) > 1 
                    for lane_info in inter.lane_all_directions.values()
                )
                if has_shared:
                    shared_lane_count += 1
                
                shared_mark = " [共用车道]" if has_shared else ""
                print(f"      {inter.from_edge} -> {inter.to_edge}: "
                      f"主流向={inter.primary_direction}, "
                      f"全绿流向={'+'.join(sorted(inter.all_green_directions))}, "
                      f"车道数={len(inter.available_lanes)}{shared_mark}")
        
        print("\n" + "-" * 70)
        print(f"总计: {len(self.busline_info)} 条公交线路, "
              f"{total_intersections} 个交叉口进口道, "
              f"{shared_lane_count} 处涉及共用车道")
        print("=" * 70)


# ============================================================================
# 便捷函数
# ============================================================================

def analyze_busline_intersections(
    busline_xml: str,
    net_file: str,
    output_path: str = None
) -> Tuple[Dict[str, BuslineIntersectionInfo], str]:
    """
    便捷函数：分析公交线路交叉口流向信息
    
    Args:
        busline_xml: 公交线路XML文件路径
        net_file: SUMO路网文件路径
        output_path: 输出文件路径（可选）
        
    Returns:
        (分析结果字典, 输出文件路径)
    """
    # 解析路网
    network = NetworkParser(net_file)
    
    # 解析公交线路
    busline_parser = BuslineParser(busline_xml)
    
    # 分析交叉口流向
    analyzer = BuslineIntersectionAnalyzer(network, busline_parser)
    result = analyzer.analyze_all_buslines()
    
    # 打印摘要
    analyzer.print_summary()
    
    # 保存结果
    if output_path is None:
        output_dir = os.path.dirname(busline_xml) or '.'
        output_path = os.path.join(output_dir, 'step1_busline_intersection_info.xml')
    
    analyzer.save_to_xml(output_path)
    
    return result, output_path


def load_busline_intersection_info(xml_path: str) -> Dict[str, BuslineIntersectionInfo]:
    """
    从XML文件加载公交线路交叉口流向信息
    
    Args:
        xml_path: XML文件路径
        
    Returns:
        {busline_id: BuslineIntersectionInfo}
    """
    if not os.path.exists(xml_path):
        raise FileNotFoundError(f"文件不存在: {xml_path}")
    
    tree = ET.parse(xml_path)
    root = tree.getroot()
    
    result = {}
    
    for busline_elem in root.findall('busline'):
        busline_id = busline_elem.get('id')
        info = BuslineIntersectionInfo(busline_id=busline_id)
        
        for inter_elem in busline_elem.findall('intersection'):
            # 解析可用车道
            lanes_elem = inter_elem.find('available_lanes')
            available_lanes = lanes_elem.text.split(',') if lanes_elem is not None and lanes_elem.text else []
            
            # 解析全绿流向
            all_green_str = inter_elem.get('all_green_directions', '')
            all_green_directions = set(all_green_str.split('+')) if all_green_str else set()
            
            # 解析全绿linkIndex
            link_indices_str = inter_elem.get('green_link_indices', '')
            green_link_indices = [int(x) for x in link_indices_str.split(',') if x] if link_indices_str else []
            
            # 解析车道详情
            lane_all_directions = {}
            lane_details_elem = inter_elem.find('lane_details')
            if lane_details_elem is not None:
                for lane_elem in lane_details_elem.findall('lane'):
                    lane_id = lane_elem.get('id')
                    dirs_str = lane_elem.get('directions', '')
                    directions = set(dirs_str.split('+')) if dirs_str else set()
                    
                    # 解析连接
                    connections = []
                    for conn_elem in lane_elem.findall('connection'):
                        conn = ConnectionInfo(
                            from_edge=inter_elem.get('from_edge'),
                            to_edge=conn_elem.get('to_edge'),
                            from_lane=int(lane_elem.get('index', 0)),
                            to_lane=int(conn_elem.get('to_lane', 0)),
                            direction=conn_elem.get('dir', 's'),
                            traffic_light=conn_elem.get('tl', ''),
                            link_index=int(conn_elem.get('linkIndex', -1)) if conn_elem.get('linkIndex') else -1
                        )
                        connections.append(conn)
                    
                    lane_info = LaneDirectionInfo(
                        lane_id=lane_id,
                        lane_index=int(lane_elem.get('index', 0)),
                        directions=directions,
                        direction_group=lane_elem.get('direction_group', ''),
                        connections=connections
                    )
                    lane_all_directions[lane_id] = lane_info
            
            approach = IntersectionApproachInfo(
                from_edge=inter_elem.get('from_edge'),
                to_edge=inter_elem.get('to_edge'),
                traffic_light=inter_elem.get('traffic_light', ''),
                primary_direction=inter_elem.get('primary_direction', 'T'),
                available_lanes=available_lanes,
                lane_all_directions=lane_all_directions,
                all_green_directions=all_green_directions,
                green_link_indices=green_link_indices
            )
            info.intersections.append(approach)
        
        result[busline_id] = info
    
    return result


# ============================================================================
# 主程序入口
# ============================================================================

if __name__ == "__main__":
    import sys
    
    print("=" * 70)
    print("Step 1: 公交线路交叉口流向信息提取器")
    print("=" * 70)
    
    if len(sys.argv) >= 3:
        busline_xml = sys.argv[1]
        net_file = sys.argv[2]
        output_path = sys.argv[3] if len(sys.argv) > 3 else None
        
        result, output_file = analyze_busline_intersections(
            busline_xml, net_file, output_path
        )
        
        print(f"\n提取完成！")
        print(f"输出文件: {output_file}")
    else:
        print("""
用法:
    python step1_busline_intersection_analyzer.py <busline_xml> <net_file> [output_path]

参数:
    busline_xml: 公交线路XML文件
    net_file: SUMO路网文件 (.net.xml)
    output_path: 输出文件路径（可选）

示例:
    python step1_busline_intersection_analyzer.py busline.xml network.net.xml

编程调用:
    from step1_busline_intersection_analyzer import analyze_busline_intersections
    
    result, output_path = analyze_busline_intersections(
        'busline.xml',
        'network.net.xml'
    )
""")
