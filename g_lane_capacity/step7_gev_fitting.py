#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
============================================================================
Step 7: 通行能力GEV分布拟合器
"""

import xml.etree.ElementTree as ET
from xml.dom import minidom
import os
import glob
import numpy as np
from typing import Dict, List, Tuple, Optional, Any, Set
from dataclasses import dataclass, field
from collections import defaultdict, Counter
from datetime import datetime
import warnings
import json

try:
    from scipy import stats

    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    warnings.warn("scipy未安装，将使用简化的参数估计方法")


# ============================================================================
# 数据类定义
# ============================================================================

@dataclass
class InstantVehicleRecord:
    """
    瞬时感应线圈检测器的单条车辆记录

    【V3.0新增】state字段，用于区分记录类型：
    - "enter": 车辆头部进入检测器的瞬间（用于计算车头时距）
    - "stay":  车辆停留在检测器上（用于检测红灯拥堵）
    - "leave": 车辆离开检测器（通常被丢弃以节省内存）

    物理意义:
    - enter记录：精确的车头通过时刻，是计算通行能力的唯一可靠数据源
    - stay记录：反映车辆滞留状态，低速stay表明发生了拥堵或红灯停车
    """
    detector_id: str  # 检测器ID
    time: float  # 时间戳 (秒)
    speed: float  # 瞬时速度 (m/s)
    vehicle_id: str  # 车辆ID
    state: str = "enter"  # 记录状态: "enter" | "stay" | "leave"


# ============================================================================
# 兼容性别名：保留旧类名以兼容外部调用
# ============================================================================
VehiclePassRecord = InstantVehicleRecord


@dataclass
class AggregatedInterval:
    """
    时间区间聚合数据

    用于崩溃检测的分钟级数据聚合，包含该区间内的车辆数量和速度统计
    """
    begin: float  # 区间起始时间 (秒)
    end: float  # 区间结束时间 (秒)
    vehicle_count: int  # 区间内车辆数（基于enter记录）
    mean_speed: float  # 区间内平均速度（包含enter和低速stay）
    speeds: List[float] = field(default_factory=list)  # 所有速度记录
    enter_count: int = 0  # 【V3.0新增】仅enter记录的数量
    stay_count: int = 0  # 【V3.0新增】低速stay记录的数量

    @property
    def flow_vph(self) -> float:
        """计算小时流量 (veh/h)，仅基于enter记录"""
        duration = self.end - self.begin
        return (self.enter_count / duration) * 3600 if duration > 0 else 0


@dataclass
class GEVParams:
    """
    GEV (广义极值分布) 参数

    GEV分布形式: F(x) = exp{-[1 + ξ((x-μ)/σ)]^(-1/ξ)}
    - μ (location): 位置参数，对应通行能力的中心值
    - σ (scale): 尺度参数，对应通行能力的离散程度
    - ξ (shape): 形状参数，决定分布类型
        - ξ ≈ 0: Gumbel分布（Type I）
        - ξ > 0: Frechet分布（Type II，重尾）
        - ξ < 0: Weibull分布（Type III，有上界）
    """
    shape: float  # ξ (xi): 形状参数
    location: float  # μ (mu): 位置参数
    scale: float  # σ (sigma): 尺度参数
    n_samples: int = 0  # 样本数量
    log_likelihood: float = 0.0  # 对数似然值
    aic: float = 0.0  # AIC信息准则
    mean_capacity: float = 0.0  # 平均通行能力
    std_capacity: float = 0.0  # 通行能力标准差
    min_capacity: float = 0.0  # 最小通行能力
    max_capacity: float = 0.0  # 最大通行能力

    def get_distribution_type(self) -> str:
        """根据形状参数判断分布类型"""
        if abs(self.shape) < 0.01:
            return "Gumbel"  # Type I: 轻尾分布
        elif self.shape > 0:
            return "Frechet"  # Type II: 重尾分布（可能存在极大值）
        else:
            return "Weibull"  # Type III: 有界分布


@dataclass
class DetectorDiagnostic:
    """
    单个检测器的诊断信息

    记录检测器在单次实验中的完整处理过程，用于调试和质量控制
    """
    detector_id: str  # 检测器ID
    lane_id: str  # 车道ID
    edge_id: str  # 路段ID
    direction: str  # 流向（left/through/right）
    file_path: str  # 数据文件路径
    run_id: str  # 实验运行ID
    has_records: bool = False  # 是否有有效记录
    record_count: int = 0  # 总记录数（含enter和stay）
    enter_count: int = 0  # 【V3.0新增】enter记录数
    stay_count: int = 0  # 【V3.0新增】低速stay记录数
    min_time: float = 0.0  # 最早记录时间
    max_time: float = 0.0  # 最晚记录时间
    duration: float = 0.0  # 数据时间跨度
    min_speed: float = 0.0  # 最小速度
    max_speed: float = 0.0  # 最大速度
    mean_speed: float = 0.0  # 平均速度
    interval_count: int = 0  # 聚合区间数量
    breakdown_detected: bool = False  # 是否检测到崩溃
    breakdown_time: float = 0.0  # 崩溃发生时间
    capacity: Optional[float] = None  # 计算得到的通行能力
    mean_headway: float = 0.0  # 平均车头时距
    valid_headway_count: int = 0  # 有效车头时距数量
    failure_reason: str = ""  # 失败原因


@dataclass
class DirectionDiagnostic:
    """
    某流向的诊断信息汇总

    汇总同一流向在多次实验中的表现，用于评估数据质量和成功率
    """
    edge_id: str  # 路段ID
    direction: str  # 流向
    lane_ids: Set[str] = field(default_factory=set)  # 涉及的车道ID集合
    total_experiments: int = 0  # 总实验次数
    experiments_with_records: int = 0  # 有记录的实验次数
    experiments_with_breakdown: int = 0  # 检测到崩溃的次数
    experiments_with_capacity: int = 0  # 成功计算通行能力的次数
    failure_reasons: Dict[str, int] = field(default_factory=lambda: defaultdict(int))
    capacity_samples: List[float] = field(default_factory=list)  # 通行能力样本
    gev_params: Optional[GEVParams] = None  # GEV拟合参数
    gev_fit_error: str = ""  # GEV拟合错误信息

    def add_diagnostic(self, diag: DetectorDiagnostic):
        """添加单个检测器的诊断信息"""
        self.lane_ids.add(diag.lane_id)
        self.total_experiments += 1
        if diag.has_records:
            self.experiments_with_records += 1
        if diag.breakdown_detected:
            self.experiments_with_breakdown += 1
        if diag.capacity is not None:
            self.experiments_with_capacity += 1
            self.capacity_samples.append(diag.capacity)
        if diag.failure_reason:
            self.failure_reasons[diag.failure_reason] += 1

    def get_success_rate(self) -> float:
        """计算通行能力计算成功率"""
        if self.total_experiments == 0:
            return 0.0
        return self.experiments_with_capacity / self.total_experiments

    def get_breakdown_rate(self) -> float:
        """计算崩溃检测率（在有记录的实验中）"""
        if self.experiments_with_records == 0:
            return 0.0
        return self.experiments_with_breakdown / self.experiments_with_records

    def get_top_failure_reasons(self, n: int = 3) -> List[Tuple[str, int]]:
        """获取最常见的失败原因"""
        return sorted(self.failure_reasons.items(), key=lambda x: -x[1])[:n]


@dataclass
class DirectionCapacityData:
    """
    某流向的通行能力数据 (V3.1更新)

    存储该流向在多次实验中的通行能力样本和GEV拟合结果

    【V3.1更新】
    - capacity_samples 现在存储的是单车道平均通行能力（而非总通行能力）
    - 新增 num_lanes 字段记录该流向的车道数
    """
    edge_id: str  # 路段ID
    direction: str  # 流向
    lane_ids: List[str] = field(default_factory=list)  # 车道ID列表
    lane_capacities: Dict[str, Dict[str, float]] = field(default_factory=dict)
    capacity_samples: List[float] = field(default_factory=list)  # 【V3.1】单车道平均通行能力样本
    gev_params: Optional[GEVParams] = None  # GEV拟合参数
    # 【V3.1新增】统计信息
    num_lanes: int = 0  # 该流向的车道数
    multi_lane_samples: int = 0  # 多车道取平均的样本数
    single_lane_samples: int = 0  # 单车道样本数


# ============================================================================
# 解析器：InstantInductionLoopParser
# ============================================================================

class InstantInductionLoopParser:
    """
    瞬时感应线圈检测器数据解析器 (V3.0 - 红灯截流增强版)

    【核心功能】
    解析SUMO InstantInductionLoop输出的XML文件，实施三重过滤规则：

    【过滤规则】
    规则A (保留进入): 必须保留所有 state="enter" 的记录
        - 物理意义: enter是车头通过检测器的精确时刻，用于计算车头时距

    规则B (保留拥堵证据): 保留 state="stay" 且 speed < stay_speed_threshold 的记录
        - 物理意义: 低速stay表明车辆停留在检测器上（红灯/拥堵），用于触发崩溃检测
        - 默认阈值: 5.0 m/s (约18 km/h)

    规则C (降噪): 丢弃 state="leave" 和高速 state="stay" 记录
        - 物理意义: leave记录冗余（enter已捕获通过），高速stay无拥堵信息

    【使用示例】
    >>> parser = InstantInductionLoopParser(stay_speed_threshold=5.0)
    >>> records = parser.parse_file("detector_output.xml")
    >>> for detector_id, detector_records in records.items():
    ...     for rec in detector_records:
    ...         print(f"{rec.time}: {rec.vehicle_id} {rec.state} @ {rec.speed:.1f} m/s")
    """

    def __init__(self,
                 stay_speed_threshold: float = 5.0,
                 debug: bool = False):
        """
        初始化解析器

        Args:
            stay_speed_threshold: stay记录的速度阈值 (m/s)
                                  低于此速度的stay记录将被保留作为拥堵证据
                                  建议值: 5.0 m/s (约18 km/h，典型的拥堵/停车阈值)
            debug: 是否输出调试信息
        """
        self.stay_speed_threshold = stay_speed_threshold
        self.debug = debug

        # 【V3.0新增】解析统计信息
        self._parse_stats = {
            'total_elements': 0,  # 原始XML元素总数
            'enter_kept': 0,  # 保留的enter记录数
            'stay_kept': 0,  # 保留的低速stay记录数
            'stay_discarded': 0,  # 丢弃的高速stay记录数
            'leave_discarded': 0,  # 丢弃的leave记录数
            'parse_errors': 0  # 解析错误数
        }

    def parse_file(self, file_path: str) -> Dict[str, List[InstantVehicleRecord]]:
        """
        解析单个检测器输出XML文件

        实施三重过滤规则：
        1. 规则A: 保留所有 state="enter" 记录
        2. 规则B: 保留 state="stay" 且 speed < stay_speed_threshold 的记录
        3. 规则C: 丢弃 state="leave" 和高速 state="stay" 记录

        Args:
            file_path: XML文件路径

        Returns:
            Dict[detector_id -> List[InstantVehicleRecord]]
            按检测器ID分组的记录列表，每个列表按时间排序
        """
        if not os.path.exists(file_path):
            if self.debug:
                print(f"  [警告] 文件不存在: {file_path}")
            return {}

        # 重置解析统计
        self._parse_stats = {k: 0 for k in self._parse_stats}

        try:
            tree = ET.parse(file_path)
            root = tree.getroot()

            # 按检测器ID分组存储记录
            records: Dict[str, List[InstantVehicleRecord]] = defaultdict(list)

            # 遍历所有 instantOut 元素
            for elem in root.findall('.//instantOut'):
                self._parse_stats['total_elements'] += 1

                # 获取记录状态
                state = elem.get('state', '')

                # ====== 规则C: 丢弃 leave 记录 ======
                if state == 'leave':
                    self._parse_stats['leave_discarded'] += 1
                    continue

                # 获取速度值
                try:
                    speed = float(elem.get('speed', 0))
                    speed = max(0, speed)  # 确保速度非负
                except (ValueError, TypeError):
                    speed = 0.0

                # ====== 规则C: 丢弃高速 stay 记录 ======
                if state == 'stay' and speed >= self.stay_speed_threshold:
                    self._parse_stats['stay_discarded'] += 1
                    continue

                # ====== 规则A + 规则B: 保留 enter 和低速 stay ======
                detector_id = elem.get('id')
                if not detector_id:
                    continue

                try:
                    record = InstantVehicleRecord(
                        detector_id=detector_id,
                        time=float(elem.get('time', 0)),
                        speed=speed,
                        vehicle_id=elem.get('vehID', ''),
                        state=state  # 【V3.0关键】保留state字段
                    )
                    records[detector_id].append(record)

                    # 更新统计
                    if state == 'enter':
                        self._parse_stats['enter_kept'] += 1
                    elif state == 'stay':
                        self._parse_stats['stay_kept'] += 1

                except (ValueError, TypeError) as e:
                    self._parse_stats['parse_errors'] += 1
                    if self.debug:
                        print(f"  [警告] 解析记录失败: {e}")
                    continue

            # 按时间排序每个检测器的记录
            for detector_id in records:
                records[detector_id].sort(key=lambda r: r.time)

            # 输出调试信息
            if self.debug and self._parse_stats['total_elements'] > 0:
                print(f"  [解析] {os.path.basename(file_path)}: "
                      f"原始{self._parse_stats['total_elements']}条 → "
                      f"enter={self._parse_stats['enter_kept']}, "
                      f"stay(低速)={self._parse_stats['stay_kept']}, "
                      f"丢弃stay(高速)={self._parse_stats['stay_discarded']}, "
                      f"丢弃leave={self._parse_stats['leave_discarded']}")

            return dict(records)

        except ET.ParseError as e:
            if self.debug:
                print(f"  [错误] XML解析失败: {file_path}, {e}")
            return {}

    def get_parse_stats(self) -> Dict[str, int]:
        """获取最近一次解析的统计信息"""
        return self._parse_stats.copy()


# ============================================================================
# 兼容性别名：保留旧类名
# ============================================================================
InstantE1Parser = InstantInductionLoopParser


# ============================================================================
# 崩溃检测器：BreakdownDetector
# ============================================================================

class BreakdownDetector:
    """
    交通流崩溃检测器 (V3.2 - 数据空缺检测增强版)

    【核心功能】
    1. 检测交通流崩溃时刻（平均速度持续低于阈值）
    2. 【V3.2新增】检测数据空缺导致的隐式崩溃
    3. 回溯至崩溃前的稳态期计算饱和流通行能力

    【V3.2关键更新：数据空缺检测】

    问题背景:
        即使检测器位置优化到-3.5米，仍可能因随机性导致红灯期间
        没有车辆正好停在检测器上，造成数据空缺（例如700秒无记录）。

    解决方案:
        检测连续超过阈值时间（gap_threshold_seconds）无记录的情况，
        将数据空缺视为"隐式崩溃"，因为正常饱和流不会出现如此长的空缺。

    物理依据:
        - 饱和流下，车头时距约1.8-2.2秒
        - 超过120秒无记录，必定不是正常饱和流状态
        - 极可能是红灯导致车辆全部停在检测器前方（而非上方）

    【V3.0/V3.1历史功能：双模式数据处理】

    模式1: 崩溃检测 (_aggregate_records_for_detection)
        - 使用所有记录（enter + 低速stay）
        - 物理意义: 红灯亮起时，大量零速度的stay记录能瞬间拉低平均速度
        - 确保能够检测到红灯导致的崩溃

    模式2: 通行能力计算 (_calculate_capacity_from_headways)
        - 严格只使用 state="enter" 记录
        - 物理意义: 通行能力由车头通过停止线的时间差决定
        - stay记录是持续时间的伪影，会导致计算出的时距接近0（流量趋于无穷大）

    【60秒缓冲区机制】
    在回溯计算通行能力时，跳过崩溃前的最后60秒（可配置）
    物理意义: 激波到达检测器的瞬间存在非稳态过渡，应排除在外

    【使用示例】
    >>> detector = BreakdownDetector(
    ...     speed_threshold_ratio=0.5,    # 速度阈值 = 限速 × 0.5
    ...     sustained_periods=3,           # 连续3个区间低于阈值触发崩溃
    ...     lookback_periods=5,            # 回溯5个区间计算通行能力
    ...     buffer_seconds=60.0,           # 跳过崩溃前60秒
    ...     gap_threshold_seconds=120.0    # 【V3.2新增】数据空缺阈值120秒
    ... )
    >>> capacity, diagnostic = detector.detect_and_calculate_capacity(records)
    """

    def __init__(self,
                 aggregation_interval: float = 60.0,
                 min_headway: float = 0.5,
                 max_headway: float = 3.5,
                 min_vehicles_for_capacity: int = 10,
                 fixed_speed_limit: float = 16.67,
                 speed_threshold_ratio: float = 0.5,
                 min_speed_drop: float = 5.0,
                 sustained_periods: int = 3,
                 lookback_periods: int = 5,
                 buffer_seconds: float = 60.0,
                 gap_threshold_seconds: float = 120.0,
                 debug: bool = False):
        """
        初始化崩溃检测器

        Args:
            aggregation_interval: 数据聚合区间时长 (秒)，默认60秒
            min_headway: 有效车头时距的最小值 (秒)，过小可能是噪声
            max_headway: 有效车头时距的最大值 (秒)，过大表示车流间断
            min_vehicles_for_capacity: 计算通行能力所需的最小车辆数
            fixed_speed_limit: 路段限速 (m/s)，默认60 km/h ≈ 16.67 m/s
            speed_threshold_ratio: 崩溃速度阈值比例，阈值 = 限速 × 比例
            min_speed_drop: 触发崩溃所需的最小速度下降 (m/s)
            sustained_periods: 触发崩溃所需的连续低速区间数
            lookback_periods: 回溯计算通行能力的区间数
            buffer_seconds: 【V3.0新增】崩溃前的缓冲时间 (秒)，跳过激波过渡期
            gap_threshold_seconds: 【V3.2新增】数据空缺阈值 (秒)
                                   连续超过此时间无记录视为隐式崩溃
                                   默认120秒（饱和流下约60辆车的通过时间）
            debug: 是否输出调试信息
        """
        self.aggregation_interval = aggregation_interval
        self.min_headway = min_headway
        self.max_headway = max_headway
        self.min_vehicles_for_capacity = min_vehicles_for_capacity
        self.fixed_speed_limit = fixed_speed_limit
        self.speed_threshold_ratio = speed_threshold_ratio
        self.min_speed_drop = min_speed_drop
        self.sustained_periods = sustained_periods
        self.lookback_periods = lookback_periods
        self.buffer_seconds = buffer_seconds  # 【V3.0新增】
        self.gap_threshold_seconds = gap_threshold_seconds  # 【V3.2新增】
        self.debug = debug

        # 计算速度阈值
        self.speed_threshold = fixed_speed_limit * speed_threshold_ratio

    def _aggregate_records_for_detection(self,
                                         records: List[InstantVehicleRecord]
                                         ) -> List[AggregatedInterval]:
        """
        【模式1: 崩溃检测】聚合记录用于崩溃检测

        关键点: 使用所有有效记录（enter + 低速stay）计算平均速度
        物理意义: 红灯亮起时，stay记录的零速度能瞬间拉低平均速度，触发崩溃阈值

        Args:
            records: 所有车辆记录（含enter和stay）

        Returns:
            按时间排序的聚合区间列表
        """
        if not records:
            return []

        min_time = min(r.time for r in records)
        max_time = max(r.time for r in records)

        intervals = []
        current_time = min_time

        while current_time < max_time:
            interval_end = current_time + self.aggregation_interval

            # 获取该区间内的所有记录（enter + stay）
            interval_records = [r for r in records
                                if current_time <= r.time < interval_end]

            if interval_records:
                # 分离 enter 和 stay 记录
                enter_records = [r for r in interval_records if r.state == 'enter']
                stay_records = [r for r in interval_records if r.state == 'stay']

                # 【关键】使用所有记录计算平均速度（用于崩溃检测）
                all_speeds = [r.speed for r in interval_records]

                intervals.append(AggregatedInterval(
                    begin=current_time,
                    end=interval_end,
                    vehicle_count=len(interval_records),  # 总记录数
                    mean_speed=np.mean(all_speeds) if all_speeds else 0,  # 含stay的平均速度
                    speeds=all_speeds,
                    enter_count=len(enter_records),  # 【V3.0】仅enter数量
                    stay_count=len(stay_records)  # 【V3.0】stay数量
                ))

            current_time = interval_end

        return intervals

    def _detect_breakdown(self,
                          intervals: List[AggregatedInterval]
                          ) -> Optional[int]:
        """
        检测交通流崩溃时刻

        崩溃定义: 连续 sustained_periods 个区间的平均速度低于阈值
                  且存在明显的速度下降（min_speed_drop）

        Args:
            intervals: 聚合区间列表

        Returns:
            崩溃开始的区间索引，若未检测到则返回None
        """
        if len(intervals) < self.sustained_periods:
            return None

        for i in range(len(intervals) - self.sustained_periods + 1):
            # 检查连续 sustained_periods 个区间是否都低于速度阈值
            low_speed_count = sum(
                1 for j in range(self.sustained_periods)
                if intervals[i + j].mean_speed < self.speed_threshold
            )

            if low_speed_count == self.sustained_periods:
                # 检查是否存在明显的速度下降
                if i > 0:
                    speed_drop = intervals[i - 1].mean_speed - intervals[i].mean_speed
                    if speed_drop >= self.min_speed_drop:
                        if self.debug:
                            print(f"    [崩溃检测] 区间{i}: 速度从"
                                  f"{intervals[i - 1].mean_speed:.1f}下降到"
                                  f"{intervals[i].mean_speed:.1f} m/s "
                                  f"(下降{speed_drop:.1f})")
                        return i
                else:
                    # 第一个区间就低速，可能是仿真初期
                    return i

        return None

    def _detect_data_gap(self,
                         records: List[InstantVehicleRecord]
                         ) -> Optional[Tuple[float, float]]:
        """
        【V3.2新增】检测数据空缺导致的隐式崩溃

        当检测器记录之间存在超过阈值的时间间隔时，视为隐式崩溃。

        物理依据:
            - 正常饱和流下，车头时距约1.8-2.2秒
            - 超过120秒无任何记录，必定不是正常饱和流状态
            - 极可能是红灯导致车辆全部停在检测器前方（而非上方）

        Args:
            records: 所有车辆记录

        Returns:
            Tuple[gap_start_time, gap_duration] 如果检测到数据空缺
            None 如果没有显著数据空缺
        """
        if len(records) < 2:
            return None

        # 按时间排序
        sorted_records = sorted(records, key=lambda r: r.time)

        # 检测最大时间间隔
        max_gap = 0.0
        max_gap_start = 0.0

        for i in range(1, len(sorted_records)):
            gap = sorted_records[i].time - sorted_records[i - 1].time
            if gap > max_gap:
                max_gap = gap
                max_gap_start = sorted_records[i - 1].time

        # 判断是否超过阈值
        if max_gap >= self.gap_threshold_seconds:
            if self.debug:
                print(f"    [V3.2 数据空缺检测] 发现数据空缺: "
                      f"起始时间={max_gap_start:.1f}s, 持续={max_gap:.1f}s "
                      f"(阈值={self.gap_threshold_seconds}s)")
            return (max_gap_start, max_gap)

        return None

    def _filter_enter_records(self,
                              records: List[InstantVehicleRecord]
                              ) -> List[InstantVehicleRecord]:
        """
        【模式2: 通行能力计算】筛选仅enter记录

        关键点: 通行能力由车头通过停止线的时间差决定
        物理意义: stay记录是持续时间伪影，会导致时距接近0（流量趋于无穷大）

        Args:
            records: 所有记录

        Returns:
            仅包含 state="enter" 的记录列表
        """
        return [r for r in records if r.state == 'enter']

    def _calculate_capacity_from_headways(self,
                                          records: List[InstantVehicleRecord],
                                          start_time: float,
                                          end_time: float
                                          ) -> Tuple[Optional[float], float, int]:
        """
        【模式2: 通行能力计算】基于车头时距计算通行能力

        关键流程:
        1. 筛选时间范围内的记录
        2. 【V3.0关键】过滤只保留 state="enter" 记录
        3. 计算相邻车辆的车头时距
        4. 过滤异常时距
        5. 计算平均时距和通行能力

        Args:
            records: 所有车辆记录（含enter和stay）
            start_time: 回溯窗口起始时间
            end_time: 回溯窗口结束时间（考虑buffer后）

        Returns:
            Tuple[通行能力(veh/h), 平均时距(s), 有效时距数量]
        """
        # 获取时间范围内的记录
        range_records = [r for r in records
                         if start_time <= r.time < end_time]

        # 【V3.0关键步骤】仅使用 enter 记录计算车头时距
        enter_records = self._filter_enter_records(range_records)
        enter_records = sorted(enter_records, key=lambda r: r.time)

        if self.debug:
            all_count = len(range_records)
            enter_count = len(enter_records)
            stay_count = all_count - enter_count
            print(f"    [时距计算] 回溯窗口: {start_time:.0f}s - {end_time:.0f}s, "
                  f"总记录={all_count}, enter={enter_count}, stay={stay_count}")

        # 检查是否有足够的enter记录
        if len(enter_records) < self.min_vehicles_for_capacity:
            return None, 0, 0

        # 计算车头时距
        headways = []
        for i in range(1, len(enter_records)):
            h = enter_records[i].time - enter_records[i - 1].time

            # 过滤异常时距
            if self.min_headway <= h <= self.max_headway:
                headways.append(h)

        # 检查是否有足够的有效时距
        min_valid_headways = self.min_vehicles_for_capacity // 2
        if len(headways) < min_valid_headways:
            return None, 0, len(headways)

        # 计算平均时距和通行能力
        mean_headway = np.mean(headways)
        capacity = 3600.0 / mean_headway  # veh/h

        if self.debug:
            print(f"    [时距结果] 有效时距={len(headways)}, "
                  f"平均时距={mean_headway:.2f}s, "
                  f"通行能力={capacity:.0f} veh/h")

        return capacity, mean_headway, len(headways)

    def detect_and_calculate_capacity(self,
                                      records: List[InstantVehicleRecord]
                                      ) -> Tuple[Optional[float], Dict[str, Any]]:
        """
        检测崩溃并计算通行能力（主入口方法）(V3.2更新)

        完整流程：
        1. 检查记录数量是否足够
        2. 使用所有记录聚合数据（含stay，用于检测）
        3. 检测崩溃时刻 T（基于速度阈值）
        4. 【V3.2新增】如果未检测到速度崩溃，检测数据空缺
        5. 确定回溯窗口：[T - lookback_periods × interval, T - buffer_seconds]
        6. 在回溯窗口内，仅用enter记录计算车头时距
        7. 计算通行能力

        Args:
            records: 车辆记录列表（含enter和stay）

        Returns:
            Tuple[通行能力(veh/h), 诊断信息字典]
        """
        # 初始化诊断信息
        diagnostic = {
            'total_records': len(records),
            'enter_count': 0,
            'stay_count': 0,
            'min_time': 0.0,
            'max_time': 0.0,
            'duration': 0.0,
            'min_speed': 0.0,
            'max_speed': 0.0,
            'mean_speed': 0.0,
            'total_intervals': 0,
            'breakdown_detected': False,
            'breakdown_time': 0.0,
            'breakdown_type': '',  # 【V3.2新增】崩溃类型: 'speed' 或 'data_gap'
            'data_gap_duration': 0.0,  # 【V3.2新增】数据空缺持续时间
            'capacity': None,
            'mean_headway': 0,
            'valid_headway_count': 0,
            'reason': ''
        }

        # 检查记录数量
        if len(records) < self.min_vehicles_for_capacity:
            diagnostic['reason'] = f'记录数不足({len(records)}<{self.min_vehicles_for_capacity})'
            return None, diagnostic

        # 【V3.0】分别统计enter和stay记录
        enter_records = [r for r in records if r.state == 'enter']
        stay_records = [r for r in records if r.state == 'stay']
        diagnostic['enter_count'] = len(enter_records)
        diagnostic['stay_count'] = len(stay_records)

        # 基本统计信息
        times = [r.time for r in records]
        speeds = [r.speed for r in records]
        diagnostic['min_time'] = min(times)
        diagnostic['max_time'] = max(times)
        diagnostic['duration'] = max(times) - min(times)
        diagnostic['min_speed'] = min(speeds)
        diagnostic['max_speed'] = max(speeds)
        diagnostic['mean_speed'] = np.mean(speeds)

        # 【模式1】使用所有记录（含stay）进行崩溃检测
        intervals = self._aggregate_records_for_detection(records)
        diagnostic['total_intervals'] = len(intervals)

        if not intervals:
            diagnostic['reason'] = '无有效时间区间'
            return None, diagnostic

        # 检测崩溃
        breakdown_idx = self._detect_breakdown(intervals)
        breakdown_time = None  # 用于存储崩溃时间

        if breakdown_idx is not None:
            # 检测到速度崩溃
            breakdown_time = intervals[breakdown_idx].begin
            diagnostic['breakdown_detected'] = True
            diagnostic['breakdown_time'] = breakdown_time
            diagnostic['breakdown_type'] = 'speed'

            if self.debug:
                print(f"    [崩溃类型] 速度崩溃，时间={breakdown_time:.1f}s")
        else:
            # 【V3.2新增】未检测到速度崩溃，尝试检测数据空缺
            gap_result = self._detect_data_gap(records)

            if gap_result is not None:
                gap_start, gap_duration = gap_result
                breakdown_time = gap_start  # 将空缺起始时间作为崩溃时间

                diagnostic['breakdown_detected'] = True
                diagnostic['breakdown_time'] = breakdown_time
                diagnostic['breakdown_type'] = 'data_gap'
                diagnostic['data_gap_duration'] = gap_duration

                if self.debug:
                    print(f"    [崩溃类型] 数据空缺崩溃，时间={breakdown_time:.1f}s, "
                          f"空缺持续={gap_duration:.1f}s")
            else:
                # 既没有速度崩溃，也没有数据空缺
                low_speed_intervals = sum(1 for iv in intervals
                                          if iv.mean_speed < self.speed_threshold)
                if low_speed_intervals == 0:
                    diagnostic['reason'] = (f'无低速区间且无数据空缺(阈值={self.speed_threshold:.1f}m/s, '
                                            f'stay记录={len(stay_records)})')
                elif low_speed_intervals < self.sustained_periods:
                    diagnostic['reason'] = (f'低速区间不连续({low_speed_intervals}<'
                                            f'{self.sustained_periods})且无数据空缺')
                else:
                    diagnostic['reason'] = '速度下降不明显且无数据空缺'
                return None, diagnostic

        # 【V3.0/V3.2关键】确定回溯窗口（跳过buffer区）
        # 对于数据空缺崩溃，回溯到空缺前的饱和流期
        if diagnostic['breakdown_type'] == 'data_gap':
            # 数据空缺情况：回溯窗口为空缺前的一段时间
            end_time = breakdown_time  # 空缺起始时间
            start_time = max(diagnostic['min_time'],
                             breakdown_time - self.lookback_periods * self.aggregation_interval)
        else:
            # 速度崩溃情况：使用原有逻辑
            lookback_start_idx = max(0, breakdown_idx - self.lookback_periods)
            start_time = intervals[lookback_start_idx].begin
            end_time = breakdown_time - self.buffer_seconds

        # 确保end_time > start_time
        if end_time <= start_time:
            # 缓冲区太大或时间窗口不合理
            end_time = breakdown_time
            if self.debug:
                print(f"    [警告] 回溯窗口调整: end_time <= start_time，已使用崩溃时间作为结束时间")

        if self.debug:
            print(f"    [回溯] 崩溃时间={breakdown_time:.0f}s, "
                  f"回溯窗口=[{start_time:.0f}s, {end_time:.0f}s], "
                  f"类型={diagnostic['breakdown_type']}")

        # 【模式2】仅用enter记录计算通行能力
        capacity, mean_headway, headway_count = self._calculate_capacity_from_headways(
            records, start_time, end_time
        )

        diagnostic['capacity'] = capacity
        diagnostic['mean_headway'] = mean_headway
        diagnostic['valid_headway_count'] = headway_count

        if capacity is None:
            diagnostic['reason'] = (f'有效车头时距不足({headway_count}<'
                                    f'{self.min_vehicles_for_capacity // 2})')

        return capacity, diagnostic


# ============================================================================
# 兼容性别名：保留旧类名
# ============================================================================
HeadwayBreakdownDetector = BreakdownDetector


# ============================================================================
# GEV分布拟合器 (V3.3 - 稳健拟合增强版)
# ============================================================================

class GEVFitter:
    """
    GEV (广义极值分布) 拟合器 (V3.3 - 稳健拟合增强版)

    【V3.3核心更新】
    1. 拟合前离群值处理：使用IQR方法剔除异常样本
    2. 形状参数约束：限制ξ在合理范围内[-2, 2]
    3. 多策略拟合：尝试多种方法选择最优结果
    4. 拟合质量诊断：记录离群值剔除和约束应用情况

    【物理背景】
    通行能力的GEV分布形状参数应在合理范围内：
    - |ξ| < 0.5: 接近Gumbel分布，最常见
    - 0.5 < |ξ| < 2: 轻度偏态，可接受
    - |ξ| > 2: 极端偏态，可能存在数据问题

    【使用示例】
    >>> fitter = GEVFitter(
    ...     outlier_method='iqr',      # 离群值检测方法
    ...     iqr_multiplier=1.5,        # IQR倍数
    ...     shape_bounds=(-2, 2),      # 形状参数约束
    ...     enable_constraint=True     # 是否启用约束
    ... )
    >>> params = fitter.fit(capacity_samples)
    """

    def __init__(self,
                 outlier_method: str = 'iqr',
                 iqr_multiplier: float = 1.5,
                 shape_bounds: Tuple[float, float] = (-2.0, 2.0),
                 enable_constraint: bool = True,
                 min_samples_after_filter: int = 3,
                 debug: bool = False):
        """
        初始化GEV拟合器

        Args:
            outlier_method: 离群值检测方法
                - 'iqr': 四分位距方法 (默认)
                - 'zscore': Z分数方法
                - 'none': 不进行离群值处理
            iqr_multiplier: IQR方法的倍数 (默认1.5，使用3.0更宽松)
            shape_bounds: 形状参数约束范围 (默认[-2, 2])
            enable_constraint: 是否启用形状参数约束 (默认True)
            min_samples_after_filter: 过滤后最少需要的样本数 (默认3)
            debug: 是否输出调试信息
        """
        self.outlier_method = outlier_method
        self.iqr_multiplier = iqr_multiplier
        self.shape_bounds = shape_bounds
        self.enable_constraint = enable_constraint
        self.min_samples_after_filter = min_samples_after_filter
        self.debug = debug

        # 拟合诊断信息
        self._last_fit_info = {}

    def _detect_outliers_iqr(self, data: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        使用IQR方法检测离群值

        Args:
            data: 原始数据

        Returns:
            Tuple[过滤后数据, 离群值数组]
        """
        q1, q3 = np.percentile(data, [25, 75])
        iqr = q3 - q1
        lower_bound = q1 - self.iqr_multiplier * iqr
        upper_bound = q3 + self.iqr_multiplier * iqr

        mask = (data >= lower_bound) & (data <= upper_bound)
        filtered_data = data[mask]
        outliers = data[~mask]

        return filtered_data, outliers

    def _detect_outliers_zscore(self, data: np.ndarray, threshold: float = 3.0
                                ) -> Tuple[np.ndarray, np.ndarray]:
        """
        使用Z分数方法检测离群值

        Args:
            data: 原始数据
            threshold: Z分数阈值 (默认3.0)

        Returns:
            Tuple[过滤后数据, 离群值数组]
        """
        mean = np.mean(data)
        std = np.std(data)

        if std == 0:
            return data, np.array([])

        z_scores = np.abs((data - mean) / std)
        mask = z_scores <= threshold
        filtered_data = data[mask]
        outliers = data[~mask]

        return filtered_data, outliers

    def _filter_outliers(self, data: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        根据配置的方法过滤离群值

        Args:
            data: 原始数据

        Returns:
            Tuple[过滤后数据, 离群值数组]
        """
        if self.outlier_method == 'iqr':
            return self._detect_outliers_iqr(data)
        elif self.outlier_method == 'zscore':
            return self._detect_outliers_zscore(data)
        else:
            return data, np.array([])

    def _fit_unconstrained(self, data: np.ndarray) -> Tuple[float, float, float, float]:
        """
        无约束GEV拟合

        Returns:
            Tuple[shape, location, scale, log_likelihood]
        """
        if SCIPY_AVAILABLE:
            shape, loc, scale = stats.genextreme.fit(data)
            xi = -shape  # scipy使用-ξ
            log_likelihood = np.sum(stats.genextreme.logpdf(data, shape, loc, scale))
            return xi, loc, scale, log_likelihood
        else:
            # 矩估计法
            mean = np.mean(data)
            std = np.std(data)
            scale = std * np.sqrt(6) / np.pi
            loc = mean - 0.5772 * scale
            return 0.0, loc, scale, 0.0

    def _fit_constrained(self, data: np.ndarray) -> Tuple[float, float, float, float]:
        """
        带形状参数约束的GEV拟合

        使用L-BFGS-B优化器进行约束极大似然估计

        Returns:
            Tuple[shape, location, scale, log_likelihood]
        """
        if not SCIPY_AVAILABLE:
            return self._fit_unconstrained(data)

        from scipy.optimize import minimize

        # 初始估计（使用矩估计）
        mean = np.mean(data)
        std = np.std(data)
        init_scale = std * np.sqrt(6) / np.pi
        init_loc = mean - 0.5772 * init_scale
        init_shape = 0.0

        def neg_log_likelihood(params):
            """负对数似然函数"""
            xi, loc, scale = params
            if scale <= 0:
                return 1e10

            # 转换为scipy的参数形式
            try:
                ll = np.sum(stats.genextreme.logpdf(data, -xi, loc, scale))
                if np.isnan(ll) or np.isinf(ll):
                    return 1e10
                return -ll
            except:
                return 1e10

        # 设置约束
        bounds = [
            (self.shape_bounds[0], self.shape_bounds[1]),  # shape (ξ)
            (None, None),  # location (μ)
            (1e-6, None)  # scale (σ) > 0
        ]

        # 多起点优化以避免局部最优
        best_result = None
        best_nll = float('inf')

        init_points = [
            (init_shape, init_loc, init_scale),
            (0.0, init_loc, init_scale),
            (-0.5, init_loc, init_scale * 1.2),
            (0.5, init_loc, init_scale * 0.8),
        ]

        for init_point in init_points:
            try:
                result = minimize(
                    neg_log_likelihood,
                    init_point,
                    method='L-BFGS-B',
                    bounds=bounds,
                    options={'maxiter': 1000}
                )
                if result.fun < best_nll:
                    best_nll = result.fun
                    best_result = result
            except:
                continue

        if best_result is not None and best_result.success:
            xi, loc, scale = best_result.x
            log_likelihood = -best_result.fun
            return xi, loc, scale, log_likelihood
        else:
            # 回退到无约束拟合，然后截断
            xi, loc, scale, ll = self._fit_unconstrained(data)
            xi_clipped = np.clip(xi, self.shape_bounds[0], self.shape_bounds[1])
            return xi_clipped, loc, scale, ll

    def fit(self, data: np.ndarray) -> GEVParams:
        """
        拟合GEV分布（V3.3增强版）

        流程：
        1. 离群值检测与过滤
        2. 选择拟合策略（约束/无约束）
        3. 执行拟合
        4. 质量检查与诊断

        Args:
            data: 通行能力样本数组

        Returns:
            GEV分布参数

        Raises:
            ValueError: 样本数量不足
        """
        original_data = np.array(data)

        if len(original_data) < 3:
            raise ValueError("样本数量不足(至少需要3个)")

        # 重置诊断信息
        self._last_fit_info = {
            'original_count': len(original_data),
            'outliers_removed': 0,
            'outliers': [],
            'constraint_applied': False,
            'constraint_active': False,
            'fit_method': 'unconstrained'
        }

        # ====== Step 1: 离群值处理 ======
        filtered_data, outliers = self._filter_outliers(original_data)

        self._last_fit_info['outliers_removed'] = len(outliers)
        self._last_fit_info['outliers'] = outliers.tolist() if len(outliers) > 0 else []

        # 如果过滤后样本不足，使用原始数据
        if len(filtered_data) < self.min_samples_after_filter:
            if self.debug:
                print(f"    [GEV拟合] 过滤后样本不足({len(filtered_data)}), 使用原始数据")
            filtered_data = original_data
            self._last_fit_info['outliers_removed'] = 0
            self._last_fit_info['outliers'] = []

        if self.debug and len(outliers) > 0:
            print(f"    [GEV拟合] 剔除{len(outliers)}个离群值: {[f'{x:.0f}' for x in outliers]}")

        # ====== Step 2: 选择拟合策略并执行 ======
        if self.enable_constraint and SCIPY_AVAILABLE:
            # 首先尝试无约束拟合
            xi_unc, loc_unc, scale_unc, ll_unc = self._fit_unconstrained(filtered_data)

            # 检查是否需要约束
            if self.shape_bounds[0] <= xi_unc <= self.shape_bounds[1]:
                # 形状参数在合理范围内，使用无约束结果
                xi, loc, scale, log_likelihood = xi_unc, loc_unc, scale_unc, ll_unc
                self._last_fit_info['fit_method'] = 'unconstrained'
            else:
                # 形状参数超出范围，使用约束拟合
                xi, loc, scale, log_likelihood = self._fit_constrained(filtered_data)
                self._last_fit_info['fit_method'] = 'constrained'
                self._last_fit_info['constraint_applied'] = True
                self._last_fit_info['constraint_active'] = True
                self._last_fit_info['original_shape'] = xi_unc

                if self.debug:
                    print(f"    [GEV拟合] 形状参数约束: {xi_unc:.2f} -> {xi:.2f}")
        else:
            # 无约束拟合
            xi, loc, scale, log_likelihood = self._fit_unconstrained(filtered_data)
            self._last_fit_info['fit_method'] = 'unconstrained'

        # ====== Step 3: 计算统计量 ======
        # 使用原始数据计算描述性统计（用于报告）
        # 但GEV参数是基于过滤后数据拟合的

        # AIC计算
        aic = 2 * 3 - 2 * log_likelihood if log_likelihood != 0 else 0

        # 计算理论均值和标准差
        if SCIPY_AVAILABLE and abs(xi) > 0.001:
            try:
                # GEV的理论均值和标准差
                if xi < 1:
                    from scipy.special import gamma
                    g1 = gamma(1 - xi)
                    theoretical_mean = loc + scale * (g1 - 1) / xi
                else:
                    theoretical_mean = np.mean(filtered_data)

                if xi < 0.5:
                    g2 = gamma(1 - 2 * xi)
                    theoretical_var = (scale ** 2) * (g2 - g1 ** 2) / (xi ** 2)
                    theoretical_std = np.sqrt(theoretical_var) if theoretical_var > 0 else np.std(filtered_data)
                else:
                    theoretical_std = np.std(filtered_data)
            except:
                theoretical_mean = np.mean(filtered_data)
                theoretical_std = np.std(filtered_data)
        else:
            theoretical_mean = np.mean(filtered_data)
            theoretical_std = np.std(filtered_data)

        # ====== Step 4: 构建返回结果 ======
        return GEVParams(
            shape=xi,
            location=loc,
            scale=scale,
            n_samples=len(filtered_data),  # 使用过滤后的样本数
            log_likelihood=log_likelihood,
            aic=aic,
            mean_capacity=np.mean(filtered_data),
            std_capacity=np.std(filtered_data),
            min_capacity=np.min(filtered_data),
            max_capacity=np.max(filtered_data)
        )

    def get_last_fit_info(self) -> Dict[str, Any]:
        """
        获取最近一次拟合的诊断信息

        Returns:
            包含以下键的字典:
            - original_count: 原始样本数
            - outliers_removed: 剔除的离群值数量
            - outliers: 离群值列表
            - constraint_applied: 是否应用了约束
            - constraint_active: 约束是否实际生效
            - fit_method: 使用的拟合方法
            - original_shape: (仅当约束生效时) 约束前的形状参数
        """
        return self._last_fit_info.copy()


# ============================================================================
# 通行能力分析器
# ============================================================================

class CapacityAnalyzer:
    """
    通行能力分析器 (V3.3 - 稳健拟合增强版)

    【核心功能】
    处理多个实验的检测器输出，检测崩溃，计算通行能力，拟合GEV分布

    【V3.3更新】
    - 使用增强版GEVFitter（离群值处理+形状参数约束）
    - 新增gev_config参数配置GEV拟合行为
    - 拟合统计信息输出

    【V3.0更新】
    - 使用 InstantInductionLoopParser 解析数据（含低速stay记录）
    - 使用 BreakdownDetector 进行双模式处理
    - 增强的诊断信息输出
    """

    def __init__(self,
                 busline_intersection_xml: str,
                 results_dir: str,
                 breakdown_config: Dict[str, Any] = None,
                 gev_config: Dict[str, Any] = None,
                 stay_speed_threshold: float = 5.0,
                 debug: bool = False):
        """
        初始化分析器

        Args:
            busline_intersection_xml: 公交线路交叉口信息XML路径
            results_dir: 实验结果目录
            breakdown_config: 崩溃检测器配置字典
            gev_config: 【V3.3新增】GEV拟合器配置字典，可包含：
                - outlier_method: 离群值检测方法 ('iqr'/'zscore'/'none')
                - iqr_multiplier: IQR倍数 (默认1.5)
                - shape_bounds: 形状参数约束范围 (默认(-2, 2))
                - enable_constraint: 是否启用约束 (默认True)
            stay_speed_threshold: 【V3.0新增】stay记录速度阈值 (m/s)
            debug: 是否输出调试信息
        """
        self.busline_intersection_xml = busline_intersection_xml
        self.results_dir = results_dir
        self.debug = debug

        # 【V3.0】使用新的解析器，保留低速stay记录
        self.parser = InstantInductionLoopParser(
            stay_speed_threshold=stay_speed_threshold,
            debug=debug
        )

        # 【V3.3】配置GEV拟合器
        gev_cfg = gev_config or {}
        self.gev_fitter = GEVFitter(
            outlier_method=gev_cfg.get('outlier_method', 'iqr'),
            iqr_multiplier=gev_cfg.get('iqr_multiplier', 1.5),
            shape_bounds=gev_cfg.get('shape_bounds', (-2.0, 2.0)),
            enable_constraint=gev_cfg.get('enable_constraint', True),
            debug=debug
        )

        # 配置崩溃检测器
        breakdown_cfg = (breakdown_config or {}).copy()
        breakdown_cfg.pop('debug', None)
        self.breakdown_detector = BreakdownDetector(debug=debug, **breakdown_cfg)

        # 存储结果和诊断信息
        self.direction_capacities: Dict[str, Dict[str, DirectionCapacityData]] = defaultdict(dict)
        self.detector_diagnostics: List[DetectorDiagnostic] = []
        self.direction_diagnostics: Dict[str, Dict[str, DirectionDiagnostic]] = defaultdict(dict)

        # 处理统计
        self.stats = {
            'files_processed': 0,
            'files_empty': 0,
            'files_parse_error': 0,
            'detectors_processed': 0,
            'detectors_not_mapped': 0,
            'detectors_no_records': 0,
            'capacities_calculated': 0,
            'no_capacity': 0,
            'total_enter_records': 0,  # 【V3.0新增】
            'total_stay_records': 0  # 【V3.0新增】
        }

        self.expected_directions: Dict[str, Set[str]] = defaultdict(set)

        # 构建检测器-流向映射
        self._build_detector_direction_mapping()

    def _build_detector_direction_mapping(self):
        """构建检测器ID到流向的映射"""
        from step1_busline_intersection_analyzer import load_busline_intersection_info

        busline_info = load_busline_intersection_info(self.busline_intersection_xml)
        self.lane_to_direction: Dict[str, Tuple[str, str]] = {}

        for info in busline_info.values():
            for inter in info.intersections:
                edge_id = inter.from_edge
                for lane_id, lane_info in inter.lane_all_directions.items():
                    direction = lane_info.direction_group
                    self.lane_to_direction[lane_id] = (edge_id, direction)
                    self.expected_directions[edge_id].add(direction)

        print(f"[Step7] 建立了 {len(self.lane_to_direction)} 个检测器-流向映射")
        print(f"[Step7] 预期有 {len(self.expected_directions)} 个路段, "
              f"{sum(len(d) for d in self.expected_directions.values())} 个流向")

    def _extract_lane_from_detector_id(self,
                                       detector_id: str
                                       ) -> Tuple[Optional[str], Optional[str]]:
        """从检测器ID提取路段ID和车道ID"""
        if detector_id.startswith('det_'):
            lane_id = detector_id[4:]
            if lane_id in self.lane_to_direction:
                return self.lane_to_direction[lane_id][0], lane_id
        return None, None

    def _get_direction_for_lane(self, lane_id: str) -> Optional[str]:
        """获取车道对应的流向"""
        if lane_id in self.lane_to_direction:
            return self.lane_to_direction[lane_id][1]
        return None

    def process_all_experiments(self, file_pattern: str = "*_E1_instant_out.xml"):
        """
        处理所有实验结果

        Args:
            file_pattern: 检测器输出文件匹配模式
        """
        print("\n" + "=" * 70)
        print("处理实验结果 (V3.2 - 数据空缺检测增强版)")
        print("=" * 70)
        print(f"  stay记录速度阈值: {self.parser.stay_speed_threshold} m/s")
        print(f"  缓冲区时间: {self.breakdown_detector.buffer_seconds} s")
        print(f"  【V3.2】数据空缺阈值: {self.breakdown_detector.gap_threshold_seconds} s")

        # 搜索文件
        search_path = os.path.join(self.results_dir, file_pattern)
        files = glob.glob(search_path)
        print(f"找到 {len(files)} 个检测器输出文件")

        if not files:
            print(f"  [警告] 在 {self.results_dir} 中没有找到匹配 {file_pattern} 的文件")
            return

        # 初始化流向诊断
        for edge_id, directions in self.expected_directions.items():
            for direction in directions:
                self.direction_diagnostics[edge_id][direction] = DirectionDiagnostic(
                    edge_id=edge_id, direction=direction
                )

        # 处理每个文件
        for file_idx, file_path in enumerate(sorted(files)):
            if (file_idx + 1) % 100 == 0:
                print(f"  处理进度: {file_idx + 1}/{len(files)}")
            self._process_single_file(file_path)

        # 【V3.0/V3.2】输出记录和崩溃统计
        print(f"\n记录统计: enter={self.stats['total_enter_records']}, "
              f"stay(低速)={self.stats['total_stay_records']}")
        speed_bd = self.stats.get('speed_breakdown_count', 0)
        gap_bd = self.stats.get('gap_breakdown_count', 0)
        print(f"崩溃统计: 速度崩溃={speed_bd}, 数据空缺崩溃={gap_bd}")

        # 聚合并拟合GEV
        self._aggregate_and_fit()

        # 打印诊断摘要
        self._print_diagnostic_summary()

    def _process_single_file(self, file_path: str):
        """处理单个检测器输出文件"""
        filename = os.path.basename(file_path)
        parts = filename.replace('_E1_instant_out.xml', '').rsplit('_run_', 1)
        run_id = f"{parts[0]}_run_{parts[1]}" if len(parts) == 2 else filename.replace('.xml', '')

        self.stats['files_processed'] += 1

        # 使用新的解析器（保留低速stay记录）
        records = self.parser.parse_file(file_path)

        if not records:
            self.stats['files_empty'] += 1
            return

        # 处理每个检测器
        for detector_id, detector_records in records.items():
            self._process_single_detector(detector_id, detector_records, file_path, run_id)

    def _process_single_detector(self,
                                 detector_id: str,
                                 records: List[InstantVehicleRecord],
                                 file_path: str,
                                 run_id: str):
        """处理单个检测器的数据"""
        self.stats['detectors_processed'] += 1

        # 提取路段和车道信息
        edge_id, lane_id = self._extract_lane_from_detector_id(detector_id)
        if not edge_id or not lane_id:
            self.stats['detectors_not_mapped'] += 1
            return

        direction = self._get_direction_for_lane(lane_id)
        if not direction:
            self.stats['detectors_not_mapped'] += 1
            return

        # 创建诊断记录
        diag = DetectorDiagnostic(
            detector_id=detector_id,
            lane_id=lane_id,
            edge_id=edge_id,
            direction=direction,
            file_path=file_path,
            run_id=run_id
        )

        if not records:
            diag.has_records = False
            diag.failure_reason = '无车辆通过记录'
            self.stats['detectors_no_records'] += 1
        else:
            diag.has_records = True
            diag.record_count = len(records)

            # 【V3.0】统计enter和stay记录
            enter_count = sum(1 for r in records if r.state == 'enter')
            stay_count = sum(1 for r in records if r.state == 'stay')
            diag.enter_count = enter_count
            diag.stay_count = stay_count
            self.stats['total_enter_records'] += enter_count
            self.stats['total_stay_records'] += stay_count

            # 使用双模式崩溃检测器
            capacity, diagnostic_info = self.breakdown_detector.detect_and_calculate_capacity(records)

            # 填充诊断信息
            diag.min_time = diagnostic_info.get('min_time', 0)
            diag.max_time = diagnostic_info.get('max_time', 0)
            diag.duration = diagnostic_info.get('duration', 0)
            diag.min_speed = diagnostic_info.get('min_speed', 0)
            diag.max_speed = diagnostic_info.get('max_speed', 0)
            diag.mean_speed = diagnostic_info.get('mean_speed', 0)
            diag.interval_count = diagnostic_info.get('total_intervals', 0)
            diag.breakdown_detected = diagnostic_info.get('breakdown_detected', False)
            diag.breakdown_time = diagnostic_info.get('breakdown_time', 0)
            diag.capacity = capacity
            diag.mean_headway = diagnostic_info.get('mean_headway', 0)
            diag.valid_headway_count = diagnostic_info.get('valid_headway_count', 0)
            diag.failure_reason = diagnostic_info.get('reason', '')

            # 【V3.2新增】统计崩溃类型
            breakdown_type = diagnostic_info.get('breakdown_type', '')
            if breakdown_type == 'speed':
                self.stats['speed_breakdown_count'] = self.stats.get('speed_breakdown_count', 0) + 1
            elif breakdown_type == 'data_gap':
                self.stats['gap_breakdown_count'] = self.stats.get('gap_breakdown_count', 0) + 1

            if capacity is not None:
                self.stats['capacities_calculated'] += 1

                # 存储通行能力数据
                if direction not in self.direction_capacities[edge_id]:
                    self.direction_capacities[edge_id][direction] = DirectionCapacityData(
                        edge_id=edge_id, direction=direction
                    )

                dc = self.direction_capacities[edge_id][direction]
                if lane_id not in dc.lane_ids:
                    dc.lane_ids.append(lane_id)
                if run_id not in dc.lane_capacities:
                    dc.lane_capacities[run_id] = {}
                dc.lane_capacities[run_id][lane_id] = capacity
            else:
                self.stats['no_capacity'] += 1

        # 记录诊断信息
        self.detector_diagnostics.append(diag)
        if edge_id in self.direction_diagnostics and direction in self.direction_diagnostics[edge_id]:
            self.direction_diagnostics[edge_id][direction].add_diagnostic(diag)

    def _aggregate_and_fit(self):
        """
        【V3.3更新】聚合通行能力数据并拟合GEV分布

        V3.3更新：使用增强版GEVFitter，包含离群值处理和形状参数约束
        V3.1更新：改为计算单车道平均通行能力，避免双峰问题
        """
        print("\n" + "-" * 70)
        print("聚合数据并拟合GEV分布 (V3.3 - 稳健拟合增强版)")
        print("-" * 70)
        print("【V3.3】离群值处理 + 形状参数约束")
        print(f"  离群值方法: {self.gev_fitter.outlier_method}")
        print(f"  IQR倍数: {self.gev_fitter.iqr_multiplier}")
        print(f"  形状参数约束: {self.gev_fitter.shape_bounds}")

        fitted_count = 0
        failed_count = 0

        # 用于统计的计数器
        total_multi_lane_runs = 0
        total_single_lane_runs = 0

        # 【V3.3新增】拟合质量统计
        outliers_removed_total = 0
        constraints_applied_count = 0
        shape_before_constraint = []
        shape_after_constraint = []

        for edge_id, directions in self.direction_capacities.items():
            for direction, dc in directions.items():
                # 【V3.1核心变更】聚合同一流向各车道的通行能力
                multi_lane_count = 0
                single_lane_count = 0

                for run_id, lane_caps in dc.lane_capacities.items():
                    if lane_caps:
                        num_lanes_with_data = len(lane_caps)

                        if num_lanes_with_data > 1:
                            # 多车道：取平均值（单车道通行能力）
                            avg_capacity = sum(lane_caps.values()) / num_lanes_with_data
                            dc.capacity_samples.append(avg_capacity)
                            multi_lane_count += 1
                            total_multi_lane_runs += 1
                        else:
                            # 单车道：直接使用
                            dc.capacity_samples.append(sum(lane_caps.values()))
                            single_lane_count += 1
                            total_single_lane_runs += 1

                # 【V3.1】更新统计字段
                dc.num_lanes = len(dc.lane_ids)
                dc.multi_lane_samples = multi_lane_count
                dc.single_lane_samples = single_lane_count

                # 更新诊断信息
                if edge_id in self.direction_diagnostics and direction in self.direction_diagnostics[edge_id]:
                    self.direction_diagnostics[edge_id][direction].capacity_samples = dc.capacity_samples.copy()

                # 拟合GEV分布
                if len(dc.capacity_samples) >= 3:
                    try:
                        dc.gev_params = self.gev_fitter.fit(np.array(dc.capacity_samples))
                        fitted_count += 1

                        # 【V3.3】获取拟合诊断信息
                        fit_info = self.gev_fitter.get_last_fit_info()
                        outliers_removed_total += fit_info.get('outliers_removed', 0)

                        if fit_info.get('constraint_active', False):
                            constraints_applied_count += 1
                            shape_before_constraint.append(fit_info.get('original_shape', 0))
                            shape_after_constraint.append(dc.gev_params.shape)

                        # 输出信息
                        lane_info = f"车道数={dc.num_lanes}" if dc.num_lanes > 0 else ""
                        outlier_info = f", 剔除{fit_info['outliers_removed']}个" if fit_info[
                                                                                     'outliers_removed'] > 0 else ""
                        constraint_info = ""
                        if fit_info.get('constraint_active', False):
                            constraint_info = f", ξ约束:{fit_info['original_shape']:.1f}→{dc.gev_params.shape:.1f}"

                        print(f"  OK {edge_id} -> {direction}: "
                              f"μ={dc.gev_params.location:.0f}, "
                              f"ξ={dc.gev_params.shape:.3f}, "
                              f"σ={dc.gev_params.scale:.1f} "
                              f"({dc.gev_params.get_distribution_type()}, N={dc.gev_params.n_samples}"
                              f"{outlier_info}{constraint_info})")

                        if edge_id in self.direction_diagnostics and direction in self.direction_diagnostics[edge_id]:
                            self.direction_diagnostics[edge_id][direction].gev_params = dc.gev_params

                    except Exception as e:
                        failed_count += 1
                        print(f"  FAIL {edge_id} -> {direction}: 拟合失败 - {e}")
                        if edge_id in self.direction_diagnostics and direction in self.direction_diagnostics[edge_id]:
                            self.direction_diagnostics[edge_id][direction].gev_fit_error = str(e)
                else:
                    failed_count += 1
                    print(f"  FAIL {edge_id} -> {direction}: 样本不足 ({len(dc.capacity_samples)}个)")

        print(f"\n拟合结果: 成功 {fitted_count} 个, 失败 {failed_count} 个")
        print(f"【V3.1统计】多车道取平均: {total_multi_lane_runs}次, 单车道: {total_single_lane_runs}次")

        # 【V3.3新增】拟合质量统计
        print(f"【V3.3统计】离群值剔除: 共{outliers_removed_total}个")
        print(f"【V3.3统计】形状参数约束生效: {constraints_applied_count}次")
        if constraints_applied_count > 0:
            avg_before = np.mean([abs(x) for x in shape_before_constraint])
            avg_after = np.mean([abs(x) for x in shape_after_constraint])
            print(f"           约束前平均|ξ|: {avg_before:.2f}, 约束后: {avg_after:.2f}")

        # 存储全局统计
        self.stats['multi_lane_averaged'] = total_multi_lane_runs
        self.stats['single_lane_direct'] = total_single_lane_runs
        self.stats['outliers_removed_total'] = outliers_removed_total
        self.stats['constraints_applied'] = constraints_applied_count

    def _print_diagnostic_summary(self):
        """打印诊断摘要"""
        print("\n" + "=" * 70)
        print("诊断摘要 (V3.3 - 稳健拟合增强版)")
        print("=" * 70)

        print("\n【总体统计】")
        print(f"  处理文件数:       {self.stats['files_processed']}")
        print(f"  空文件数:         {self.stats['files_empty']}")
        print(f"  处理检测器数:     {self.stats['detectors_processed']}")
        print(f"  未映射检测器:     {self.stats['detectors_not_mapped']}")
        print(f"  计算通行能力:     {self.stats['capacities_calculated']}")
        print(f"  未得到通行能力:   {self.stats['no_capacity']}")

        if self.stats['detectors_processed'] > 0:
            success_rate = self.stats['capacities_calculated'] / self.stats['detectors_processed']
            print(f"  成功率:           {success_rate:.2%}")

        # 【V3.3新增】拟合质量统计
        outliers_total = self.stats.get('outliers_removed_total', 0)
        constraints_count = self.stats.get('constraints_applied', 0)
        if outliers_total > 0 or constraints_count > 0:
            print("\n【V3.3 拟合质量统计】")
            print(f"  离群值剔除总数:   {outliers_total}")
            print(f"  形状约束生效次数: {constraints_count}")

        # 【V3.2新增】崩溃类型统计
        speed_breakdown_count = self.stats.get('speed_breakdown_count', 0)
        gap_breakdown_count = self.stats.get('gap_breakdown_count', 0)
        if speed_breakdown_count > 0 or gap_breakdown_count > 0:
            print("\n【V3.2 崩溃类型统计】")
            print(f"  速度崩溃检测:     {speed_breakdown_count} 次")
            print(f"  数据空缺崩溃:     {gap_breakdown_count} 次")
            total_breakdowns = speed_breakdown_count + gap_breakdown_count
            if total_breakdowns > 0:
                print(f"  数据空缺占比:     {gap_breakdown_count / total_breakdowns:.1%}")

        # 【V3.0新增】记录类型统计
        print("\n【V3.0 记录类型统计】")
        print(f"  enter记录总数:    {self.stats['total_enter_records']}")
        print(f"  stay记录总数:     {self.stats['total_stay_records']}")
        if self.stats['total_enter_records'] > 0:
            stay_ratio = self.stats['total_stay_records'] / (
                    self.stats['total_enter_records'] + self.stats['total_stay_records']
            )
            print(f"  stay占比:         {stay_ratio:.2%}")

        # 【V3.1新增】车道平均统计
        print("\n【V3.1 单车道通行能力统计】")
        multi_lane = self.stats.get('multi_lane_averaged', 0)
        single_lane = self.stats.get('single_lane_direct', 0)
        total_samples = multi_lane + single_lane
        print(f"  多车道取平均:     {multi_lane} 次")
        print(f"  单车道直接使用:   {single_lane} 次")
        if total_samples > 0:
            print(f"  多车道占比:       {multi_lane / total_samples:.1%}")
        print(f"  注: 通行能力单位为 veh/h/lane（单车道）")

        print("\n【预期与实际对比】")
        print(f"  预期路段数:       {len(self.expected_directions)}")
        print(f"  实际路段数:       {len(self.direction_capacities)}")
        print(f"  预期流向数:       {sum(len(d) for d in self.expected_directions.values())}")
        print(f"  实际流向数:       {sum(len(d) for d in self.direction_capacities.values())}")

        print("\n【失败原因统计】")
        failure_reasons = Counter()
        for diag in self.detector_diagnostics:
            if diag.failure_reason:
                reason = diag.failure_reason.split('(')[0].strip()
                failure_reasons[reason] += 1

        for reason, count in failure_reasons.most_common(10):
            pct = count / len(self.detector_diagnostics) * 100 if self.detector_diagnostics else 0
            print(f"  {reason}: {count} ({pct:.1f}%)")

        print("\n【各流向详细统计】")
        print("-" * 100)
        print(f"{'状态':<4} {'路段ID':<26} {'流向':<8} {'实验':<6} {'有记录':<7} "
              f"{'有崩溃':<7} {'有通行能力':<9} {'成功率':<8}")
        print("-" * 100)

        all_directions = []
        for edge_id, directions in self.direction_diagnostics.items():
            for direction, dd in directions.items():
                all_directions.append((edge_id, direction, dd))

        all_directions.sort(key=lambda x: -x[2].get_success_rate())

        for edge_id, direction, dd in all_directions:
            success_rate = dd.get_success_rate()
            status = "OK" if dd.gev_params else "FAIL"
            print(f"{status:<4} {edge_id:<26} {direction:<8} {dd.total_experiments:<6} "
                  f"{dd.experiments_with_records:<7} {dd.experiments_with_breakdown:<7} "
                  f"{dd.experiments_with_capacity:<9} {success_rate:.1%}")

        print("-" * 100)

        # 输出未得到通行能力的流向
        no_sample = [(e, d, dd) for e, d, dd in all_directions if dd.experiments_with_capacity == 0]
        if no_sample:
            print(f"\n【未得到任何通行能力样本的流向 ({len(no_sample)}个)】")
            for edge_id, direction, dd in no_sample[:15]:
                top_reasons = dd.get_top_failure_reasons(2)
                reasons_str = ", ".join(f"{r.split('(')[0]}({c})" for r, c in top_reasons)
                print(f"  {edge_id} -> {direction}: {reasons_str}")
            if len(no_sample) > 15:
                print(f"  ... 还有 {len(no_sample) - 15} 个流向")

    def save_results(self, output_path: str):
        """保存GEV拟合结果到XML文件"""
        root = ET.Element('capacity_gev_distributions')
        root.set('generated_at', datetime.now().isoformat())
        root.set('method', 'headway')
        root.set('version', '3.1')  # 【V3.1标记】
        root.set('capacity_unit', 'veh/h/lane')  # 【V3.1】明确单位是单车道通行能力

        # 元数据
        meta = ET.SubElement(root, 'metadata')
        meta.set('total_edges', str(len(self.direction_capacities)))
        meta.set('total_directions', str(sum(len(d) for d in self.direction_capacities.values())))
        meta.set('fitted_count', str(sum(1 for d in self.direction_capacities.values()
                                         for dc in d.values() if dc.gev_params)))

        # 【V3.1】添加说明
        meta.set('note', 'V3.1: capacity values are per-lane average to avoid bimodal distribution')

        for key, value in self.stats.items():
            meta.set(key, str(value))

        # 各路段数据
        edges_elem = ET.SubElement(root, 'edges')

        for edge_id in sorted(self.direction_capacities.keys()):
            edge_elem = ET.SubElement(edges_elem, 'edge')
            edge_elem.set('id', edge_id)

            for direction, dc in sorted(self.direction_capacities[edge_id].items()):
                dir_elem = ET.SubElement(edge_elem, 'direction')
                dir_elem.set('type', direction)
                dir_elem.set('sample_count', str(len(dc.capacity_samples)))
                # 【V3.1】添加车道数信息
                dir_elem.set('num_lanes', str(dc.num_lanes))
                dir_elem.set('multi_lane_samples', str(dc.multi_lane_samples))
                dir_elem.set('single_lane_samples', str(dc.single_lane_samples))

                lanes_elem = ET.SubElement(dir_elem, 'lanes')
                lanes_elem.text = ','.join(dc.lane_ids)

                gev_elem = ET.SubElement(dir_elem, 'gev_distribution')
                if dc.gev_params:
                    gev_elem.set('fitted', 'true')
                    gev_elem.set('type', dc.gev_params.get_distribution_type())
                    gev_elem.set('unit', 'veh/h/lane')  # 【V3.1】明确单位
                    ET.SubElement(gev_elem, 'shape').text = f"{dc.gev_params.shape:.6f}"
                    ET.SubElement(gev_elem, 'location').text = f"{dc.gev_params.location:.2f}"
                    ET.SubElement(gev_elem, 'scale').text = f"{dc.gev_params.scale:.2f}"
                    ET.SubElement(gev_elem, 'mean').text = f"{dc.gev_params.mean_capacity:.2f}"
                    ET.SubElement(gev_elem, 'std').text = f"{dc.gev_params.std_capacity:.2f}"
                    # 【V3.1】如果需要总通行能力，可以乘以车道数
                    if dc.num_lanes > 0:
                        ET.SubElement(gev_elem,
                                      'total_capacity_estimate').text = f"{dc.gev_params.mean_capacity * dc.num_lanes:.2f}"
                else:
                    gev_elem.set('fitted', 'false')

                if dc.capacity_samples:
                    samples_elem = ET.SubElement(dir_elem, 'raw_samples')
                    samples_elem.text = ','.join(f"{s:.1f}" for s in dc.capacity_samples)

        # 格式化XML
        xml_str = ET.tostring(root, encoding='unicode')
        try:
            dom = minidom.parseString(xml_str)
            xml_str = '\n'.join(line for line in dom.toprettyxml(indent="    ").split('\n')
                                if line.strip())
        except:
            pass

        os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(xml_str)

        print(f"\n结果已保存: {output_path}")

    def save_diagnostic_report(self, output_path: str):
        """保存诊断报告到JSON文件"""
        report = {
            'generated_at': datetime.now().isoformat(),
            'version': '3.0',  # 【V3.0标记】
            'summary': {
                'stats': self.stats,
                'expected_edges': len(self.expected_directions),
                'expected_directions': sum(len(d) for d in self.expected_directions.values()),
                'actual_edges': len(self.direction_capacities),
                'actual_directions': sum(len(d) for d in self.direction_capacities.values()),
                'fitted_count': sum(1 for d in self.direction_capacities.values()
                                    for dc in d.values() if dc.gev_params)
            },
            'failure_reasons': dict(Counter(
                diag.failure_reason.split('(')[0].strip()
                for diag in self.detector_diagnostics if diag.failure_reason
            )),
            'direction_details': {}
        }

        for edge_id, directions in self.direction_diagnostics.items():
            report['direction_details'][edge_id] = {}
            for direction, dd in directions.items():
                report['direction_details'][edge_id][direction] = {
                    'lane_ids': list(dd.lane_ids),
                    'total_experiments': dd.total_experiments,
                    'experiments_with_records': dd.experiments_with_records,
                    'experiments_with_breakdown': dd.experiments_with_breakdown,
                    'experiments_with_capacity': dd.experiments_with_capacity,
                    'success_rate': dd.get_success_rate(),
                    'breakdown_rate': dd.get_breakdown_rate(),
                    'failure_reasons': dict(dd.failure_reasons),
                    'sample_count': len(dd.capacity_samples),
                    'gev_fitted': dd.gev_params is not None,
                    'gev_fit_error': dd.gev_fit_error
                }

                if dd.gev_params:
                    report['direction_details'][edge_id][direction]['gev'] = {
                        'shape': dd.gev_params.shape,
                        'location': dd.gev_params.location,
                        'scale': dd.gev_params.scale,
                        'mean': dd.gev_params.mean_capacity,
                        'std': dd.gev_params.std_capacity,
                        'type': dd.gev_params.get_distribution_type()
                    }

        os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(report, f, indent=2, ensure_ascii=False)

        print(f"诊断报告已保存: {output_path}")

    def get_summary(self) -> Dict[str, Any]:
        """获取处理摘要"""
        return {
            'total_edges': len(self.direction_capacities),
            'total_directions': sum(len(d) for d in self.direction_capacities.values()),
            'fitted_count': sum(1 for d in self.direction_capacities.values()
                                for dc in d.values() if dc.gev_params),
            'expected_edges': len(self.expected_directions),
            'expected_directions': sum(len(d) for d in self.expected_directions.values()),
            'processing_stats': self.stats.copy()
        }


# ============================================================================
# 公共API函数（保持与main.py的兼容性）
# ============================================================================

def fit_capacity_gev(busline_intersection_xml: str,
                     results_dir: str,
                     output_path: str,
                     breakdown_config: Dict[str, Any] = None,
                     file_pattern: str = "*_E1_instant_out.xml",
                     stay_speed_threshold: float = 5.0,
                     debug: bool = False,
                     save_diagnostic: bool = True) -> Dict[str, Any]:
    """
    拟合通行能力GEV分布（主入口函数）

    【V3.3更新】
    新增稳健拟合功能：离群值处理 + 形状参数约束
    通过breakdown_config传入以下参数：
    - outlier_method: 离群值检测方法 ('iqr'/'zscore'/'none')
    - iqr_multiplier: IQR倍数 (默认1.5)
    - shape_bounds: 形状参数约束 (默认(-2, 2))
    - enable_constraint: 是否启用约束 (默认True)

    【V3.2更新】
    新增 gap_threshold_seconds 参数支持

    【V3.0更新】
    新增 stay_speed_threshold 参数

    Args:
        busline_intersection_xml: 公交线路交叉口信息XML路径
        results_dir: 实验结果目录
        output_path: GEV分布结果输出路径
        breakdown_config: 配置字典，可包含：
            崩溃检测参数：
            - aggregation_interval: 聚合区间 (秒)
            - min_headway: 最小车头时距 (秒)
            - max_headway: 最大车头时距 (秒)
            - min_vehicles_for_capacity: 最小车辆数
            - fixed_speed_limit: 限速 (m/s)
            - speed_threshold_ratio: 速度阈值比例
            - min_speed_drop: 最小速度下降 (m/s)
            - sustained_periods: 连续低速区间数
            - lookback_periods: 回溯区间数
            - buffer_seconds: 缓冲时间 (秒)
            - gap_threshold_seconds: 数据空缺阈值 (秒)
            GEV拟合参数【V3.3新增】：
            - outlier_method: 离群值方法 ('iqr'/'zscore'/'none')
            - iqr_multiplier: IQR倍数
            - shape_bounds: 形状参数约束范围
            - enable_constraint: 是否启用形状约束
        file_pattern: 检测器输出文件匹配模式
        stay_speed_threshold: stay记录速度阈值 (m/s)
        debug: 是否输出调试信息
        save_diagnostic: 是否保存诊断报告

    Returns:
        处理摘要字典
    """
    # 【V3.3新增】从breakdown_config中提取gev_config
    gev_config = None
    if breakdown_config:
        gev_config = {
            'outlier_method': breakdown_config.pop('outlier_method', 'iqr'),
            'iqr_multiplier': breakdown_config.pop('iqr_multiplier', 1.5),
            'shape_bounds': breakdown_config.pop('shape_bounds', (-2.0, 2.0)),
            'enable_constraint': breakdown_config.pop('enable_constraint', True),
        }

    # 创建分析器
    analyzer = CapacityAnalyzer(
        busline_intersection_xml=busline_intersection_xml,
        results_dir=results_dir,
        breakdown_config=breakdown_config,
        gev_config=gev_config,
        stay_speed_threshold=stay_speed_threshold,
        debug=debug
    )

    # 处理所有实验
    analyzer.process_all_experiments(file_pattern)

    # 保存结果
    analyzer.save_results(output_path)

    # 保存诊断报告
    if save_diagnostic:
        diagnostic_path = output_path.replace('.xml', '_diagnostic.json')
        analyzer.save_diagnostic_report(diagnostic_path)

    return analyzer.get_summary()


# ============================================================================
# 主程序入口
# ============================================================================

if __name__ == "__main__":
    print("=" * 70)
    print("Step 7: 通行能力GEV分布拟合器 (V3.3 - 稳健拟合增强版)")
    print("=" * 70)
    print("\n【V3.3核心更新】")
    print("  1. 拟合前离群值处理")
    print("     - IQR方法自动剔除异常样本")
    print("     - 可配置: outlier_method, iqr_multiplier")
    print("  2. 形状参数约束")
    print("     - 限制ξ在[-2, 2]范围内")
    print("     - 使用约束优化避免极端参数")
    print("     - 可配置: shape_bounds, enable_constraint")
    print("\n【V3.2历史功能】")
    print("  - 数据空缺检测 (gap_threshold_seconds)")
    print("\n【V3.0/V3.1历史功能】")
    print("  - 双模式数据处理策略")
    print("  - 单车道通行能力计算")
    print("\n请通过main.py运行完整流程")

