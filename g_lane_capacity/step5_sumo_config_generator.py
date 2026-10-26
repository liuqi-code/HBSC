#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import xml.etree.ElementTree as ET
from xml.dom import minidom
import os
from typing import Dict, List, Optional
from dataclasses import dataclass, field
from datetime import datetime


# ============================================================================
# 数据结构定义
# ============================================================================

@dataclass
class SumoConfigParams:
    """SUMO仿真配置参数"""
    # 时间参数
    begin_time: float = 0.0         # 开始时间
    end_time: float = 3600.0        # 结束时间
    step_length: float = 0.1        # 仿真步长
    
    # 随机种子
    random_seed: int = 42
    
    # 输出控制
    no_step_log: bool = True        # 不输出步进日志
    verbose: bool = False           # 详细输出
    no_warnings: bool = True        # 不输出警告
    
    # 车辆行为
    collision_action: str = "warn"  # 碰撞处理方式
    time_to_teleport: int = -1      # 传送时间阈值（-1表示禁用）
    
    # 输出文件
    output_prefix: str = ""         # 输出文件前缀
    tripinfo_output: bool = True    # 是否输出tripinfo
    statistic_output: bool = True   # 是否输出统计信息


# ============================================================================
# SUMO配置文件生成器
# ============================================================================

class SumoConfigGenerator:
    """
    SUMO配置文件生成器
    
    生成标准的SUMO配置文件 (.sumocfg)
    """
    
    def __init__(self, params: SumoConfigParams = None):
        """
        初始化生成器
        
        Args:
            params: 仿真配置参数
        """
        self.params = params or SumoConfigParams()
    
    def _make_relative_path(
        self, 
        target_path: str, 
        base_dir: str
    ) -> str:
        """
        计算相对路径
        
        Args:
            target_path: 目标文件绝对路径
            base_dir: 基准目录（配置文件所在目录）
            
        Returns:
            相对路径
        """
        if not os.path.isabs(target_path):
            return target_path
        
        try:
            rel_path = os.path.relpath(target_path, base_dir)
            # Windows驱动器问题
            if len(rel_path) >= 2 and rel_path[1] == ':':
                return target_path
            # 统一使用正斜杠
            rel_path = rel_path.replace('\\', '/')
            return rel_path
        except ValueError:
            return target_path
    
    def generate(
        self,
        output_path: str,
        net_file: str,
        route_file: str,
        detector_file: str = None,
        output_dir: str = None
    ) -> str:
        """
        生成SUMO配置文件
        
        Args:
            output_path: 输出配置文件路径 (.sumocfg)
            net_file: 路网文件路径
            route_file: 路由文件路径
            detector_file: 检测器配置文件路径（可选）
            output_dir: 仿真输出目录（可选，默认与配置文件同目录）
            
        Returns:
            输出文件路径
        """
        # 确定配置文件目录
        config_dir = os.path.dirname(os.path.abspath(output_path))
        if not config_dir:
            config_dir = os.getcwd()
        
        # 创建根元素
        root = ET.Element('configuration')
        root.set('xmlns:xsi', 'http://www.w3.org/2001/XMLSchema-instance')
        root.set('xsi:noNamespaceSchemaLocation', 
                 'http://sumo.dlr.de/xsd/sumoConfiguration.xsd')
        
        # ========== 输入部分 ==========
        input_elem = ET.SubElement(root, 'input')
        
        # 路网文件
        net_elem = ET.SubElement(input_elem, 'net-file')
        net_elem.set('value', self._make_relative_path(net_file, config_dir))
        
        # 路由文件
        route_elem = ET.SubElement(input_elem, 'route-files')
        route_elem.set('value', self._make_relative_path(route_file, config_dir))
        
        # 检测器配置文件（附加文件）
        if detector_file:
            add_elem = ET.SubElement(input_elem, 'additional-files')
            add_elem.set('value', self._make_relative_path(detector_file, config_dir))
        
        # ========== 时间部分 ==========
        time_elem = ET.SubElement(root, 'time')
        
        begin_elem = ET.SubElement(time_elem, 'begin')
        begin_elem.set('value', str(self.params.begin_time))
        
        end_elem = ET.SubElement(time_elem, 'end')
        end_elem.set('value', str(self.params.end_time))
        
        step_elem = ET.SubElement(time_elem, 'step-length')
        step_elem.set('value', str(self.params.step_length))
        
        # ========== 输出部分 ==========
        output_elem = ET.SubElement(root, 'output')
        
        if output_dir is None:
            output_dir = config_dir
        
        prefix = self.params.output_prefix
        
        # tripinfo输出
        if self.params.tripinfo_output:
            tripinfo_elem = ET.SubElement(output_elem, 'tripinfo-output')
            tripinfo_file = os.path.join(output_dir, f"{prefix}tripinfo.xml")
            tripinfo_elem.set('value', self._make_relative_path(tripinfo_file, config_dir))
        
        # 统计输出
        if self.params.statistic_output:
            stat_elem = ET.SubElement(output_elem, 'statistic-output')
            stat_file = os.path.join(output_dir, f"{prefix}statistics.xml")
            stat_elem.set('value', self._make_relative_path(stat_file, config_dir))
        
        # ========== 处理部分 ==========
        processing = ET.SubElement(root, 'processing')
        
        collision_elem = ET.SubElement(processing, 'collision.action')
        collision_elem.set('value', self.params.collision_action)
        
        teleport_elem = ET.SubElement(processing, 'time-to-teleport')
        teleport_elem.set('value', str(self.params.time_to_teleport))
        
        # ========== 随机数部分 ==========
        random_elem = ET.SubElement(root, 'random_number')
        
        seed_elem = ET.SubElement(random_elem, 'seed')
        seed_elem.set('value', str(self.params.random_seed))
        
        # ========== 报告部分 ==========
        report_elem = ET.SubElement(root, 'report')
        
        if self.params.no_step_log:
            no_step_elem = ET.SubElement(report_elem, 'no-step-log')
            no_step_elem.set('value', 'true')
        
        verbose_elem = ET.SubElement(report_elem, 'verbose')
        verbose_elem.set('value', 'true' if self.params.verbose else 'false')
        
        if self.params.no_warnings:
            no_warn_elem = ET.SubElement(report_elem, 'no-warnings')
            no_warn_elem.set('value', 'true')
        
        # ========== 格式化输出 ==========
        xml_str = ET.tostring(root, encoding='unicode')
        
        try:
            dom = minidom.parseString(xml_str)
            pretty_xml = dom.toprettyxml(indent="    ")
            # 清理空行
            lines = pretty_xml.split('\n')
            cleaned_lines = [line for line in lines if line.strip()]
            xml_str = '\n'.join(cleaned_lines)
        except Exception:
            pass
        
        # 确保输出目录存在
        os.makedirs(config_dir, exist_ok=True)
        
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(xml_str)
        
        print(f"[Step5] SUMO配置文件已生成: {output_path}")
        return output_path


# ============================================================================
# 便捷函数
# ============================================================================

def create_sumo_config(
    output_path: str,
    net_file: str,
    route_file: str,
    detector_file: str = None,
    end_time: float = 3600.0,
    random_seed: int = 42,
    output_prefix: str = "",
    step_length: float = 0.1
) -> str:
    """
    便捷函数：创建SUMO配置文件
    
    Args:
        output_path: 输出配置文件路径
        net_file: 路网文件路径
        route_file: 路由文件路径
        detector_file: 检测器配置文件路径
        end_time: 仿真结束时间
        random_seed: 随机种子
        output_prefix: 输出文件前缀
        step_length: 仿真步长
        
    Returns:
        输出文件路径
    """
    params = SumoConfigParams(
        end_time=end_time,
        random_seed=random_seed,
        output_prefix=output_prefix,
        step_length=step_length
    )
    
    generator = SumoConfigGenerator(params)
    
    return generator.generate(
        output_path=output_path,
        net_file=net_file,
        route_file=route_file,
        detector_file=detector_file
    )


# ============================================================================
# 主程序入口
# ============================================================================

if __name__ == "__main__":
    import sys
    
    print("=" * 70)
    print("Step 5: SUMO配置文件生成器")
    print("=" * 70)
    
    if len(sys.argv) >= 4:
        output_path = sys.argv[1]
        net_file = sys.argv[2]
        route_file = sys.argv[3]
        detector_file = sys.argv[4] if len(sys.argv) > 4 else None
        
        result = create_sumo_config(
            output_path=output_path,
            net_file=net_file,
            route_file=route_file,
            detector_file=detector_file
        )
        
        print(f"\n生成完成！")
        print(f"输出文件: {result}")
    else:
        print("""
用法:
    python step5_sumo_config_generator.py <output_path> <net_file> <route_file> [detector_file]

参数:
    output_path: 输出配置文件路径 (.sumocfg)
    net_file: 路网文件路径
    route_file: 路由文件路径
    detector_file: 检测器配置文件路径（可选）

示例:
    python step5_sumo_config_generator.py simulation.sumocfg network.net.xml routes.rou.xml detectors.add.xml

编程调用:
    from step5_sumo_config_generator import create_sumo_config
    
    config_path = create_sumo_config(
        output_path='simulation.sumocfg',
        net_file='network.net.xml',
        route_file='routes.rou.xml',
        detector_file='detectors.add.xml',
        end_time=3600,
        random_seed=42
    )
""")
