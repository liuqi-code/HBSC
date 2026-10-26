#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Step 6: 批量实验运行器 
"""

import os
import sys
import subprocess
import random
import json
import time
from typing import Dict, List, Set, Tuple, Optional, Any
from dataclasses import dataclass, field, asdict
from datetime import datetime
from collections import defaultdict

# 导入各步骤的模块
from step1_busline_intersection_analyzer import (
    BuslineIntersectionInfo,
    analyze_busline_intersections,
    load_busline_intersection_info,
    BuslineParser
)
from step2_detector_config_generator import (
    DetectorConfigGenerator,
    DetectorConfigProcessor
)
from step3_route_generator import (
    RouteFileGeneratorV3,
    RouteGeneratorConfig,
    FlowPattern,
    VehicleTypeConfig,
    SingleRouteFile,
    MixedRouteInfo
)
from step4_network_modifier import (
    ModifiedNetworkGeneratorV2,
    SignalCycleConfig
)
from step5_sumo_config_generator import (
    SumoConfigGenerator,
    SumoConfigParams
)


# ============================================================================
# 数据结构定义
# ============================================================================

@dataclass
class RouteExperimentConfig:
    """
    【V3.0更新】单个流向组的实验配置
    
    记录一次实验的所有配置信息和结果
    """
    route_id: str                   # 实验ID (如 "7X_E_J100_J73_130_T_run_001")
    busline_id: str                 # 公交线路ID
    edge_id: str                    # 目标进口道路段ID
    primary_direction: str          # 公交线路主流向（V3.0重命名）
    traffic_light: str              # 目标信号灯ID
    green_link_indices: List[int]   # 所有需要全绿的linkIndex（合并后）
    run_number: int                 # 运行序号
    random_seed: int                # 随机种子
    heavy_ratio: float              # 大车比例
    friction: float                 # 摩擦系数
    
    # 生成的文件路径
    modified_net_file: str = ""
    route_file: str = ""
    detector_config_file: str = ""
    sumo_config_file: str = ""
    
    # 输出文件路径
    detector_output_file: str = ""
    
    # V3.0新增：混合流向信息
    is_mixed: bool = False                              # 是否是混合流向
    all_directions: Set[str] = field(default_factory=set)  # 所有涉及的流向
    
    # 路径上所有信号灯
    all_traffic_lights: List[Tuple[str, List[int]]] = field(default_factory=list)
    
    # 状态
    success: bool = False
    duration_seconds: float = 0.0
    error_message: str = ""


@dataclass
class BatchRunnerConfig:
    """
    批量运行器配置
    
    包含所有可配置的参数
    """
    # 输入文件
    busline_xml: str                # 公交线路XML文件路径
    net_file: str                   # SUMO路网文件路径
    
    # 输出目录
    output_dir: str = "./output"
    
    # 实验参数
    num_runs_per_route: int = 10    # 每个流向的实验次数
    
    # 随机参数范围
    seed_start: int = 1000
    heavy_ratio_range: Tuple[float, float] = (0.05, 0.15)
    friction_range: Tuple[float, float] = (0.8, 1.0)
    
    # 流量参数
    sparse_flow: float = 200.0
    congestion_flow: float = 2500.0
    num_cycles: int = 2
    congestion_ratio: float = 0.5
    warmup_time: float = 300.0
    cycle_duration: float = 1800.0
    
    # 车辆参数
    vehicle_tau: float = 1.3
    vehicle_sigma: float = 0.5
    vehicle_min_gap: float = 2.5
    vehicle_max_speed: float = 16.67
    
    # 信号灯周期参数
    signal_num_cycles: int = 4
    signal_cycle_duration: float = 120.0
    signal_green_ratio: float = 0.6
    
    # 上下游路段数量
    num_upstream_edges: int = 3     # 沿公交线路往前查找的路段数量
    num_downstream_edges: int = 2   # 沿公交线路往后查找的路段数量
    
    # SUMO配置
    sumo_binary: str = "sumo"
    step_length: float = 0.1
    
    # 要处理的公交线路列表
    busline_filter: List[str] = field(default_factory=list)


# ============================================================================
# 批量实验运行器 (V3.0)
# ============================================================================

class BatchExperimentRunner:
    """
    批量实验运行器 V3.0 - 混合流向版
    
    【核心类】按公交线路主流向组织实验，自动处理共享车道的混合流向
    
    V3.0核心改动：
    - 只遍历公交线路的主流向，不再遍历所有流向
    - 路由生成时自动包含共享车道的伴随流向
    - 信号灯设置时包含所有涉及流向的linkIndex
    """
    
    def __init__(self, config: BatchRunnerConfig):
        """
        初始化运行器
        
        Args:
            config: 批量运行器配置
        """
        self.config = config
        
        # 验证输入文件
        self._validate_input_files()
        
        # 创建输出目录结构
        self._create_output_directories()
        
        # 存储公交线路信息
        self.busline_info: Dict[str, BuslineIntersectionInfo] = {}
        
        # 公交线路解析器（用于获取线路路段序列）
        self.busline_parser: BuslineParser = None
        
        # 存储所有流向组信息（V3.0：改为主流向信息）
        self.route_infos: List[Dict] = []
        
        # 存储实验配置和结果
        self.experiments: List[RouteExperimentConfig] = []
        self.results_summary: Dict[str, Any] = {}
        
        # 检测器配置模板
        self.detector_config_template: str = ""
    
    def _validate_input_files(self):
        """验证输入文件是否存在"""
        if not os.path.exists(self.config.busline_xml):
            raise FileNotFoundError(f"公交线路XML文件不存在: {self.config.busline_xml}")
        if not os.path.exists(self.config.net_file):
            raise FileNotFoundError(f"路网文件不存在: {self.config.net_file}")
    
    def _create_output_directories(self):
        """创建输出目录结构"""
        self.output_base = os.path.abspath(self.config.output_dir)
        
        # 各类文件的输出目录
        self.networks_dir = os.path.join(self.output_base, "networks")
        self.routes_dir = os.path.join(self.output_base, "routes")
        self.detectors_dir = os.path.join(self.output_base, "detectors")
        self.configs_dir = os.path.join(self.output_base, "configs")
        self.results_dir = os.path.join(self.output_base, "results")
        self.logs_dir = os.path.join(self.output_base, "logs")
        
        # 创建目录
        for dir_path in [self.networks_dir, self.routes_dir, self.detectors_dir,
                         self.configs_dir, self.results_dir, self.logs_dir]:
            os.makedirs(dir_path, exist_ok=True)
        
        print(f"输出目录: {self.output_base}")
    
    def _run_step1(self):
        """
        Step 1: 分析公交线路交叉口流向信息
        """
        print("\n" + "=" * 70)
        print("Step 1: 分析公交线路交叉口流向信息")
        print("=" * 70)
        
        output_path = os.path.join(self.output_base, "step1_busline_intersection_info.xml")
        
        # 分析公交线路
        self.busline_info, _ = analyze_busline_intersections(
            busline_xml=self.config.busline_xml,
            net_file=self.config.net_file,
            output_path=output_path
        )
        
        # 创建公交线路解析器（用于获取线路路段序列）
        self.busline_parser = BuslineParser(self.config.busline_xml)
        
        # 应用过滤器
        if self.config.busline_filter:
            filtered = {}
            for bid in self.config.busline_filter:
                if bid in self.busline_info:
                    filtered[bid] = self.busline_info[bid]
                else:
                    print(f"  [警告] 公交线路 {bid} 不存在")
            self.busline_info = filtered
        
        # 收集所有主流向信息（V3.0变更）
        self._collect_route_infos()
        
        print(f"\n将处理 {len(self.busline_info)} 条公交线路, 共 {len(self.route_infos)} 个主流向")
    
    def _collect_route_infos(self):
        """
        【V3.0变更】收集所有主流向信息
        
        只收集公交线路的primary_direction，不再收集all_green_directions中的所有流向
        """
        self.route_infos = []
        processed = set()
        
        for busline_id, info in self.busline_info.items():
            for inter in info.intersections:
                # 【V3.0核心变更】只使用primary_direction
                primary_direction = inter.primary_direction
                
                # 生成唯一键（去重）
                key = (busline_id, inter.from_edge, primary_direction, inter.traffic_light)
                if key in processed:
                    continue
                processed.add(key)
                
                self.route_infos.append({
                    'busline_id': busline_id,
                    'edge_id': inter.from_edge,
                    'primary_direction': primary_direction,  # V3.0重命名
                    'traffic_light': inter.traffic_light,
                    'original_green_link_indices': list(inter.green_link_indices)
                })
    
    def _run_step2(self):
        """
        Step 2: 生成检测器配置（模板）
        """
        print("\n" + "=" * 70)
        print("Step 2: 生成检测器配置模板")
        print("=" * 70)
        
        self.detector_config_template = os.path.join(
            self.detectors_dir, "detector_template.add.xml"
        )
        
        generator = DetectorConfigGenerator(
            busline_info=self.busline_info,
            detector_position=-1.0,
            output_filename="E1_instant_out.xml"
        )
        generator.save_to_xml(self.detector_config_template)
    
    def _generate_experiment_configs(self) -> List[RouteExperimentConfig]:
        """
        【V3.0变更】生成所有实验的配置
        
        Returns:
            实验配置列表
        """
        experiments = []
        random.seed(self.config.seed_start)
        
        for route_info in self.route_infos:
            busline_id = route_info['busline_id']
            edge_id = route_info['edge_id']
            primary_direction = route_info['primary_direction']
            traffic_light = route_info['traffic_light']
            original_green_link_indices = route_info['original_green_link_indices']
            
            # 生成流向的唯一ID
            safe_edge_id = edge_id.replace('.', '_')
            route_base_id = f"{busline_id}_{safe_edge_id}_{primary_direction}"
            
            for run_num in range(1, self.config.num_runs_per_route + 1):
                run_id = f"{route_base_id}_run_{run_num:03d}"
                
                # 生成随机参数
                seed = self.config.seed_start + hash(run_id) % 100000
                heavy_ratio = random.uniform(*self.config.heavy_ratio_range)
                friction = random.uniform(*self.config.friction_range)
                
                # 生成文件路径
                exp = RouteExperimentConfig(
                    route_id=run_id,
                    busline_id=busline_id,
                    edge_id=edge_id,
                    primary_direction=primary_direction,
                    traffic_light=traffic_light,
                    green_link_indices=original_green_link_indices,  # 初始值，会被更新
                    run_number=run_num,
                    random_seed=seed,
                    heavy_ratio=heavy_ratio,
                    friction=friction,
                    modified_net_file=os.path.join(self.networks_dir, f"{run_id}_net.xml"),
                    route_file=os.path.join(self.routes_dir, f"{run_id}_rou.xml"),
                    detector_config_file=os.path.join(self.detectors_dir, f"{run_id}_detectors.add.xml"),
                    sumo_config_file=os.path.join(self.configs_dir, f"{run_id}.sumocfg"),
                    detector_output_file=os.path.join(self.results_dir, f"{run_id}_E1_instant_out.xml")
                )
                experiments.append(exp)
        
        return experiments
    
    def _run_single_experiment(self, exp: RouteExperimentConfig):
        """
        【V3.0更新】运行单个实验
        
        Args:
            exp: 实验配置
        """
        print(f"\n  实验: {exp.route_id}")
        exp_start = time.time()
        
        try:
            # Step 3: 生成混合流向路由文件
            route_file_info = self._generate_route_file(exp)
            
            if route_file_info is None:
                raise RuntimeError("路由生成失败")
            
            # 更新实验配置中的信息（来自路由生成结果）
            exp.all_traffic_lights = route_file_info.all_traffic_lights
            exp.green_link_indices = route_file_info.green_link_indices
            exp.is_mixed = route_file_info.is_mixed
            exp.all_directions = route_file_info.all_directions
            
            # Step 4: 生成修改后的路网（修改所有涉及流向的信号灯）
            self._generate_modified_network(exp)
            
            # Step 2: 生成检测器配置
            self._generate_detector_config(exp)
            
            # Step 5: 生成SUMO配置
            self._generate_sumo_config(exp)
            
            # Step 6: 运行SUMO仿真
            self._run_sumo_simulation(exp)
            
            exp.success = True
            
            # 打印混合流向信息
            if exp.is_mixed:
                dirs_str = '+'.join(sorted(exp.all_directions))
                print(f"    ✓ 成功 (混合流向: {dirs_str}, 信号灯数: {len(exp.all_traffic_lights)})")
            else:
                print(f"    ✓ 成功 (单流向: {exp.primary_direction}, 信号灯数: {len(exp.all_traffic_lights)})")
            
        except Exception as e:
            exp.success = False
            exp.error_message = str(e)
            print(f"    ✗ 失败: {e}")
        
        exp.duration_seconds = time.time() - exp_start
    
    def _generate_route_file(self, exp: RouteExperimentConfig) -> Optional[SingleRouteFile]:
        """
        【V3.0更新】生成混合流向路由文件
        
        Args:
            exp: 实验配置
            
        Returns:
            SingleRouteFile 或 None
        """
        # 配置流量模式
        flow_pattern = FlowPattern(
            num_cycles=self.config.num_cycles,
            cycle_duration=self.config.cycle_duration,
            congestion_ratio=self.config.congestion_ratio,
            sparse_flow=self.config.sparse_flow,
            congestion_flow=self.config.congestion_flow,
            warmup_time=self.config.warmup_time
        )
        
        # 配置车辆参数
        vehicle_config = VehicleTypeConfig(
            tau=self.config.vehicle_tau,
            sigma=self.config.vehicle_sigma,
            min_gap=self.config.vehicle_min_gap,
            max_speed=self.config.vehicle_max_speed
        )
        
        # 配置路由生成器
        route_config = RouteGeneratorConfig(
            random_seed=exp.random_seed,
            heavy_ratio_min=exp.heavy_ratio,
            heavy_ratio_max=exp.heavy_ratio,
            friction_min=exp.friction,
            friction_max=exp.friction,
            flow_pattern=flow_pattern,
            vehicle_config=vehicle_config,
            num_upstream_edges=self.config.num_upstream_edges,
            num_downstream_edges=self.config.num_downstream_edges
        )
        
        # 创建路由生成器 V3.0
        generator = RouteFileGeneratorV3(
            net_file=self.config.net_file,
            busline_info=self.busline_info,
            busline_parser=self.busline_parser,
            config=route_config
        )
        
        # 生成混合流向路由
        return generator.generate_mixed_route(
            busline_id=exp.busline_id,
            edge_id=exp.edge_id,
            primary_direction=exp.primary_direction,
            traffic_light=exp.traffic_light,
            output_path=exp.route_file,
            random_seed=exp.random_seed
        )
    
    def _generate_modified_network(self, exp: RouteExperimentConfig):
        """
        【V3.0更新】生成修改后的路网
        
        修改所有涉及流向的信号灯linkIndex
        
        Args:
            exp: 实验配置
        """
        # 配置信号灯周期
        cycle_config = SignalCycleConfig(
            num_cycles=self.config.signal_num_cycles,
            cycle_duration=self.config.signal_cycle_duration,
            green_ratio=self.config.signal_green_ratio
        )
        
        # 创建路网修改器
        generator = ModifiedNetworkGeneratorV2(self.config.net_file, cycle_config)
        
        # 生成修改后的路网（使用包含所有涉及流向linkIndex的列表）
        generator.generate_modified_network_for_route(
            target_edge=exp.edge_id,
            target_direction=exp.primary_direction,
            all_traffic_lights=exp.all_traffic_lights,
            output_path=exp.modified_net_file
        )
    
    def _generate_detector_config(self, exp: RouteExperimentConfig):
        """
        生成检测器配置
        
        Args:
            exp: 实验配置
        """
        processor = DetectorConfigProcessor(self.detector_config_template)
        processor.create_modified_config(
            run_id=exp.route_id,
            output_config_path=exp.detector_config_file,
            output_data_dir=self.results_dir
        )
    
    def _generate_sumo_config(self, exp: RouteExperimentConfig):
        """
        生成SUMO配置
        
        Args:
            exp: 实验配置
        """
        # 计算仿真结束时间
        flow_pattern = FlowPattern(
            num_cycles=self.config.num_cycles,
            cycle_duration=self.config.cycle_duration,
            warmup_time=self.config.warmup_time
        )
        end_time = flow_pattern.get_total_duration()
        
        # 配置SUMO参数
        params = SumoConfigParams(
            begin_time=0,
            end_time=end_time,
            step_length=self.config.step_length,
            random_seed=exp.random_seed,
            output_prefix=f"{exp.route_id}_"
        )
        
        # 生成配置文件
        generator = SumoConfigGenerator(params)
        generator.generate(
            output_path=exp.sumo_config_file,
            net_file=exp.modified_net_file,
            route_file=exp.route_file,
            detector_file=exp.detector_config_file,
            output_dir=self.results_dir
        )
    
    def _run_sumo_simulation(self, exp: RouteExperimentConfig):
        """
        运行SUMO仿真
        
        Args:
            exp: 实验配置
        """
        cmd = [
            self.config.sumo_binary,
            '-c', exp.sumo_config_file,
            '--no-step-log', 'true'
        ]
        
        # 运行仿真
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=3600  # 1小时超时
        )
        
        # 保存日志
        log_file = os.path.join(self.logs_dir, f"{exp.route_id}.log")
        with open(log_file, 'w') as f:
            f.write(f"Command: {' '.join(cmd)}\n")
            f.write(f"Return code: {result.returncode}\n")
            f.write(f"\n{'=' * 40}\nSTDOUT:\n{'=' * 40}\n")
            f.write(result.stdout)
            f.write(f"\n{'=' * 40}\nSTDERR:\n{'=' * 40}\n")
            f.write(result.stderr)
        
        if result.returncode != 0:
            raise RuntimeError(f"SUMO返回错误码 {result.returncode}")
    
    def run_all(self) -> Dict[str, Any]:
        """
        运行所有实验
        
        Returns:
            结果摘要
        """
        print("\n" + "#" * 70)
        print("#" + " 批量实验运行器 V3.0 - 混合流向版 ".center(54) + "#")
        print("#" * 70)
        
        # Step 1: 分析公交线路
        self._run_step1()
        
        # Step 2: 生成检测器配置模板
        self._run_step2()
        
        # 生成所有实验配置
        self.experiments = self._generate_experiment_configs()
        
        total_experiments = len(self.experiments)
        print(f"\n总实验数: {total_experiments}")
        print(f"  = {len(self.route_infos)} 个主流向 × {self.config.num_runs_per_route} 次实验")
        print(f"  上游路段数: {self.config.num_upstream_edges}")
        print(f"  下游路段数: {self.config.num_downstream_edges}")
        print(f"\n【V3.0】混合流向模式：共享车道的伴随流向将自动加入实验")
        
        # 保存实验配置
        config_file = os.path.join(self.output_base, "experiment_configs.json")
        with open(config_file, 'w', encoding='utf-8') as f:
            # 将数据转换为可序列化的格式
            serializable_experiments = []
            for exp in self.experiments:
                exp_dict = {
                    'route_id': exp.route_id,
                    'busline_id': exp.busline_id,
                    'edge_id': exp.edge_id,
                    'primary_direction': exp.primary_direction,
                    'traffic_light': exp.traffic_light,
                    'green_link_indices': exp.green_link_indices,
                    'run_number': exp.run_number,
                    'random_seed': exp.random_seed,
                    'heavy_ratio': exp.heavy_ratio,
                    'friction': exp.friction,
                    'modified_net_file': exp.modified_net_file,
                    'route_file': exp.route_file,
                    'detector_config_file': exp.detector_config_file,
                    'sumo_config_file': exp.sumo_config_file,
                    'detector_output_file': exp.detector_output_file,
                    'is_mixed': exp.is_mixed,
                    'all_directions': list(exp.all_directions),
                    'all_traffic_lights': [
                        [tl, list(indices)] for tl, indices in exp.all_traffic_lights
                    ],
                    'success': exp.success,
                    'error_message': exp.error_message
                }
                serializable_experiments.append(exp_dict)
            json.dump(serializable_experiments, f, indent=2)
        
        # 运行所有实验
        start_time = time.time()
        
        for i, exp in enumerate(self.experiments):
            print(f"\n进度: {i + 1}/{total_experiments}")
            self._run_single_experiment(exp)
        
        total_time = time.time() - start_time
        
        # 统计结果
        successful = sum(1 for exp in self.experiments if exp.success)
        failed = total_experiments - successful
        mixed_count = sum(1 for exp in self.experiments if exp.is_mixed)
        
        self.results_summary = {
            'version': '3.0',
            'total_experiments': total_experiments,
            'successful': successful,
            'failed': failed,
            'mixed_experiments': mixed_count,
            'single_experiments': total_experiments - mixed_count,
            'total_time_seconds': total_time,
            'average_time_seconds': total_time / total_experiments if total_experiments > 0 else 0,
            'num_buslines': len(self.busline_info),
            'num_primary_directions': len(self.route_infos),
            'num_runs_per_route': self.config.num_runs_per_route,
            'num_upstream_edges': self.config.num_upstream_edges,
            'num_downstream_edges': self.config.num_downstream_edges
        }
        
        # 保存结果摘要
        summary_file = os.path.join(self.output_base, "results_summary.json")
        with open(summary_file, 'w', encoding='utf-8') as f:
            json.dump(self.results_summary, f, indent=2, default=str)
        
        # 打印摘要
        print("\n" + "#" * 70)
        print("# 批量实验完成 (V3.0)")
        print("#" * 70)
        print(f"  公交线路数:    {len(self.busline_info)}")
        print(f"  主流向数:      {len(self.route_infos)}")
        print(f"  总实验数:      {total_experiments}")
        print(f"  成功:          {successful}/{total_experiments}")
        print(f"  失败:          {failed}/{total_experiments}")
        print(f"  混合流向实验:  {mixed_count}/{total_experiments}")
        print(f"  单流向实验:    {total_experiments - mixed_count}/{total_experiments}")
        print(f"  总耗时:        {total_time:.1f}秒")
        print(f"  结果目录:      {self.output_base}")
        
        if failed > 0:
            print(f"\n失败的实验 (前10个):")
            fail_count = 0
            for exp in self.experiments:
                if not exp.success:
                    print(f"  - {exp.route_id}: {exp.error_message}")
                    fail_count += 1
                    if fail_count >= 10:
                        print(f"  ... 还有 {failed - 10} 个失败的实验")
                        break
        
        print("#" * 70)
        
        return self.results_summary


# ============================================================================
# 便捷函数
# ============================================================================

def run_batch_experiments(
    busline_xml: str,
    net_file: str,
    output_dir: str = "./output",
    num_runs_per_route: int = 10,
    sparse_flow: float = 200,
    congestion_flow: float = 2500,
    num_cycles: int = 2,
    heavy_ratio_range: Tuple[float, float] = (0.05, 0.15),
    friction_range: Tuple[float, float] = (0.8, 1.0),
    signal_num_cycles: int = 4,
    signal_cycle_duration: float = 120.0,
    signal_green_ratio: float = 0.6,
    num_upstream_edges: int = 3,
    num_downstream_edges: int = 2,
    sumo_binary: str = "sumo",
    busline_filter: List[str] = None,
    **kwargs
) -> Dict[str, Any]:
    """
    便捷函数：运行批量实验
    
    【V3.0更新】自动处理混合流向
    
    Args:
        busline_xml: 公交线路XML文件
        net_file: 路网文件
        output_dir: 输出目录
        num_runs_per_route: 每个流向实验次数
        sparse_flow: 稀疏流量
        congestion_flow: 拥堵流量
        num_cycles: 稀疏-拥堵循环次数
        heavy_ratio_range: 大车比例范围
        friction_range: 摩擦系数范围
        signal_num_cycles: 信号灯周期数量
        signal_cycle_duration: 单个周期时长
        signal_green_ratio: 全绿阶段占比
        num_upstream_edges: 上游路段数量
        num_downstream_edges: 下游路段数量
        sumo_binary: SUMO可执行文件
        busline_filter: 要处理的公交线路列表
        
    Returns:
        结果摘要
    """
    config = BatchRunnerConfig(
        busline_xml=busline_xml,
        net_file=net_file,
        output_dir=output_dir,
        num_runs_per_route=num_runs_per_route,
        sparse_flow=sparse_flow,
        congestion_flow=congestion_flow,
        num_cycles=num_cycles,
        heavy_ratio_range=heavy_ratio_range,
        friction_range=friction_range,
        signal_num_cycles=signal_num_cycles,
        signal_cycle_duration=signal_cycle_duration,
        signal_green_ratio=signal_green_ratio,
        num_upstream_edges=num_upstream_edges,
        num_downstream_edges=num_downstream_edges,
        sumo_binary=sumo_binary,
        busline_filter=busline_filter or []
    )
    
    runner = BatchExperimentRunner(config)
    return runner.run_all()


# ============================================================================
# 主程序入口
# ============================================================================

if __name__ == "__main__":
    print("=" * 70)
    print("Step 6: 批量实验运行器 (V3.0 - 混合流向版)")
    print("=" * 70)
    print("""
V3.0版本核心更新:
    - 按公交线路主流向组织实验，不再遍历所有流向
    - 自动检测共享车道，生成混合流向路由
    - 信号灯设置包含所有涉及流向的linkIndex
    
示例:
    路段有两个车道：车道0(L+T)，车道1(R+T)
    当公交线路直行(T)通过时：
    - 旧版本：分别进行L、T、R三次独立实验
    - V3.0版本：进行一次实验，同时包含L、T、R三个流向的车流

用法:
    请通过main.py运行，或调用 run_batch_experiments 函数
""")
