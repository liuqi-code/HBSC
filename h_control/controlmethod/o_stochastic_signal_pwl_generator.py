import math
import numpy as np
from collections import defaultdict


# ============================================================================
# 配置常量（可通过 pwl_config 覆盖）
# ============================================================================

_DEFAULT_N_FLOW_SAMPLES = 5
_DEFAULT_N_CAPACITY_SAMPLES = 5
_DEFAULT_STARTUP_LOST_TIME = 2.0
_DEFAULT_MAX_CROSS_CYCLES = 2
_DEFAULT_AVG_VEHICLE_LENGTH = 7.0
_DEFAULT_JAM_DENSITY = 140.0      # veh/km
_DEFAULT_FREE_FLOW_SPEED = 50.0   # km/h

_DEFAULT_CAPACITY_MEAN = 1600.0   # veh/h
_DEFAULT_CAPACITY_STD = 160.0     # veh/h


# ============================================================================
# 精简版单周期冲击波模型（内嵌，无外部依赖）
# ============================================================================

class _SingleCycleShockwaveLite:
    """
    精简版单周期冲击波模型。

    核心算法与 step2_single_cycle_shockwave.SingleCycleShockwave 完全一致，
    但去掉了对 step1_data_and_parsers 中 PassStatus / PassTimeResult 的依赖。

    时间坐标系: 以该周期的红灯起始为 t=0
    空间坐标系: 停止线为 x=0，向上游为正
    周期内相位: [0, red) 红灯, [red, cycle) 绿灯
    """

    # 通过状态常量
    PASSED_IN_CYCLE = "passed_in_cycle"
    NEED_NEXT_CYCLE = "need_next_cycle"

    def __init__(self, arrival_rate, capacity, cycle_time, green_time,
                 initial_queue_length,
                 average_vehicle_length=7.0,
                 jam_density=140.0,
                 free_flow_speed=50.0,
                 startup_lost_time=2.0):
        """
        参数:
            arrival_rate          : float, 到达流率 (veh/h)
            capacity              : float, 通行能力/饱和流率 (veh/h)
            cycle_time            : float, 信号周期时长 (s)
            green_time            : float, 绿灯时长 (s)
            initial_queue_length  : float, 初始排队长度 (m)，周期开始时的队尾距停止线距离
            average_vehicle_length: float, 平均车辆长度 (m)
            jam_density           : float, 阻塞密度 (veh/km)
            free_flow_speed       : float, 自由流速度 (km/h)
            startup_lost_time     : float, 启动损失时间 τ (s)
        """
        self.arrival_rate = arrival_rate
        self.capacity = capacity
        self.cycle_time = cycle_time
        self.green_time = green_time
        self.red_time = cycle_time - green_time
        self.initial_queue = initial_queue_length
        self.avg_veh_len = average_vehicle_length
        self.tau = startup_lost_time

        # 转换单位并计算波速
        q_a = arrival_rate / 3600.0          # veh/s
        q_s = capacity / 3600.0              # veh/s
        k_j = jam_density / 1000.0           # veh/m
        v_f = free_flow_speed * 1000.0 / 3600.0  # m/s

        k_a = q_a / v_f if v_f > 0 else 0
        if k_j > k_a and q_a > 0:
            self.v_q = q_a / (k_j - k_a)
        else:
            self.v_q = q_a * average_vehicle_length if q_a > 0 else 0

        self.v_c = q_s * average_vehicle_length

        # 有效绿灯区间
        self.effective_green_start = self.red_time + self.tau
        self.effective_green_duration = max(0, self.cycle_time - self.effective_green_start)

        # 构建信号时段和队尾轨迹
        self._build_segments_and_queue()

    def _build_segments_and_queue(self):
        """构建信号时段序列 + 队尾轨迹"""
        # ---- 信号时段 ----
        self.segments = []  # [(start, end, is_effective_green), ...]
        if self.red_time > 0:
            self.segments.append((0.0, self.red_time, False))
        eff_start = self.effective_green_start
        if self.tau > 0 and eff_start < self.cycle_time:
            self.segments.append((self.red_time, min(eff_start, self.cycle_time), False))
        if eff_start < self.cycle_time:
            self.segments.append((eff_start, self.cycle_time, True))

        # ---- 队尾轨迹 ----
        self.queue_trajectory = [(0.0, self.initial_queue)]
        current_dq = self.initial_queue

        for (seg_start, seg_end, is_eff_green) in self.segments:
            net_rate = (self.v_q - self.v_c) if is_eff_green else self.v_q
            duration = seg_end - seg_start
            dq_end = current_dq + net_rate * duration
            dq_end = max(0.0, dq_end)

            # 检查是否在段内清空
            if current_dq > 0 and net_rate < 0:
                time_to_clear = current_dq / abs(net_rate)
                if time_to_clear < duration:
                    self.queue_trajectory.append((seg_start + time_to_clear, 0.0))
                    dq_end = 0.0

            self.queue_trajectory.append((seg_end, dq_end))
            current_dq = dq_end

        self.queue_end_length = current_dq

    def get_queue_at_time(self, t):
        """获取时刻 t 的队尾距停止线距离 d_q(t)，分段线性插值"""
        if t <= 0:
            return self.initial_queue
        if t >= self.cycle_time:
            return self.queue_end_length

        for i in range(len(self.queue_trajectory) - 1):
            t1, d1 = self.queue_trajectory[i]
            t2, d2 = self.queue_trajectory[i + 1]
            if t1 <= t <= t2:
                if t2 > t1:
                    return d1 + (d2 - d1) * (t - t1) / (t2 - t1)
                else:
                    return d1
        return self.queue_trajectory[-1][1]

    def find_pass_time(self, t_join, H, carry_over_work=0.0):
        """
        计算实际通过时刻。

        参数:
            t_join         : float, 公交追上队尾（或开始等待）的时刻（周期内时间）
            H              : float, 本周期需要的放行工作量（秒）
            carry_over_work: float, 上一周期遗留的剩余工作量（秒）

        返回:
            tuple: (status, t_pass_in_cycle, remaining_work)
                status          : str, PASSED_IN_CYCLE 或 NEED_NEXT_CYCLE
                t_pass_in_cycle : float, 周期内通过时刻（若跨周期则为 cycle_time）
                remaining_work  : float, 剩余工作量（若 > 0 则需跨周期）
        """
        total_H = H + carry_over_work

        if total_H <= 1e-9:
            # 无需等待放行工作量
            if self._is_effective_green(t_join):
                return (self.PASSED_IN_CYCLE, t_join, 0.0)
            else:
                if t_join < self.effective_green_start:
                    return (self.PASSED_IN_CYCLE, self.effective_green_start, 0.0)
                else:
                    return (self.NEED_NEXT_CYCLE, self.cycle_time, 0.0)

        # Event-driven: 在有效绿灯段上累计放行时间
        cumulative = 0.0
        for (seg_start, seg_end, is_eff_green) in self.segments:
            if seg_end <= t_join:
                continue
            actual_start = max(seg_start, t_join)
            if is_eff_green:
                remaining_H = total_H - cumulative
                seg_duration = seg_end - actual_start
                if seg_duration >= remaining_H:
                    t_pass = actual_start + remaining_H
                    return (self.PASSED_IN_CYCLE, t_pass, 0.0)
                else:
                    cumulative += seg_duration

        remaining_work = total_H - cumulative
        return (self.NEED_NEXT_CYCLE, self.cycle_time, remaining_work)

    def _is_effective_green(self, t):
        """判断时刻 t 是否在有效绿灯区间"""
        t_mod = t % self.cycle_time
        return self.effective_green_start <= t_mod < self.cycle_time


# ============================================================================
# 流量采样
# ============================================================================

def _parse_traffic_flow_attribute(attr_str):
    """
    解析格式 B 的 traffic_flow 属性字符串。

    输入格式: "{100:[flow,occ];200:[flow,occ];...}"
    返回: dict, {int_time_key: [flow_value, occupancy_value], ...}
           解析失败则返回空 dict。

    示例:
        "{100:[350.5,0.12];200:[420.0,0.18]}"
        -> {100: [350.5, 0.12], 200: [420.0, 0.18]}
    """
    result = {}
    if not attr_str or not isinstance(attr_str, str):
        return result

    # 去掉首尾花括号和空白
    s = attr_str.strip()
    if s.startswith("{"):
        s = s[1:]
    if s.endswith("}"):
        s = s[:-1]
    s = s.strip()
    if not s:
        return result

    # 按分号拆分各时间段
    for segment in s.split(";"):
        segment = segment.strip()
        if not segment:
            continue
        # 格式: "time_key:[val1,val2]"
        colon_idx = segment.find(":")
        if colon_idx < 0:
            continue
        try:
            time_key = int(segment[:colon_idx].strip())
        except (ValueError, TypeError):
            continue
        bracket_part = segment[colon_idx + 1:].strip()
        # 去掉方括号
        if bracket_part.startswith("["):
            bracket_part = bracket_part[1:]
        if bracket_part.endswith("]"):
            bracket_part = bracket_part[:-1]
        parts = bracket_part.split(",")
        vals = []
        for p in parts:
            try:
                vals.append(float(p.strip()))
            except (ValueError, TypeError):
                vals.append(0.0)
        result[time_key] = vals

    return result


def _load_flow_lambda_from_xml(flow_xml_path, trigger_time, cycle_time, num_cycles,
                               first_cycle_start, target_lane_id=None):
    """
    从离线流量 XML 中加载每个周期对应时段的流量 λ 值。

    [V4 修正] 同时支持两种 XML 格式：

    格式 A（<Lane> + <TimeSlot> 结构）：
        <TrafficFlow>
          <Lane id="xxx">
            <TimeSlot begin="0" end="300" flow="400.0" />
          </Lane>
        </TrafficFlow>

    格式 B（<element> + traffic_flow 属性结构）：
        <data>
          <busline id="...">
            <element id="target_lane_id" ... traffic_flow="{100:[flow,occ];200:[flow,occ];...}" />
          </busline>
        </data>

    解析优先级：
      1. 格式 A：target_lane_id 精确匹配 → flow_source = "xml_target_lane_timeslot"
      2. 格式 B：target_lane_id 精确匹配 → flow_source = "xml_target_lane_attribute"
      3. 格式 A：所有 lane 混合 fallback → flow_source = "xml_all_lanes_fallback"
      4. 默认 600 veh/h → flow_source = "default_fallback"

    参数:
        flow_xml_path    : str or None, 离线流量 XML 文件路径
        trigger_time     : float, 触发时刻 (仿真时间)
        cycle_time       : float, 信号周期时长 (s)
        num_cycles       : int, 需要覆盖的周期数
        first_cycle_start: float, 第一个周期的起始时刻
        target_lane_id   : str or None, 目标进口 lane ID

    返回:
        tuple: (lambdas, flow_load_info)
            lambdas       : list[float], 每个周期的流量 λ (veh/h)，长度 = num_cycles
            flow_load_info: dict, {
                "flow_source":        str,  见上述四种
                "target_lane_id":     str,  目标 lane ID 或 "none"
                "flow_fallback_used": bool, 是否使用了 fallback
                "xml_format_used":    str,  "timeslot" | "attribute" | "none"
                "occupancy_meta":     dict or None, 格式 B 时保留的 occupancy 数据（调试用）
            }
    """
    lambdas = []
    flow_source = "default_fallback"
    flow_fallback_used = True
    xml_format_used = "none"
    occupancy_meta = None
    effective_lane_id = target_lane_id if target_lane_id is not None else "none"

    if flow_xml_path is not None:
        try:
            import xml.etree.ElementTree as ET
            tree = ET.parse(flow_xml_path)
            root = tree.getroot()

            # ==============================================================
            # 尝试格式 A：<Lane> + <TimeSlot> 结构
            # ==============================================================
            target_slots_a = []
            all_slots_a = []

            for lane_elem in root.findall(".//Lane"):
                lane_id_in_xml = lane_elem.get("id", "")
                for slot in lane_elem.findall("TimeSlot"):
                    try:
                        begin = float(slot.get("begin", 0))
                        end = float(slot.get("end", 0))
                        flow = float(slot.get("flow", 0))
                    except (ValueError, TypeError):
                        continue
                    slot_tuple = (begin, end, flow)
                    all_slots_a.append(slot_tuple)
                    if target_lane_id is not None and lane_id_in_xml == target_lane_id:
                        target_slots_a.append(slot_tuple)

            # 格式 A：优先 target lane 精确匹配
            if target_slots_a:
                for i in range(num_cycles):
                    cycle_mid = first_cycle_start + (i + 0.5) * cycle_time
                    best_flow = 0.0
                    best_dist = float("inf")
                    for (begin, end, flow) in target_slots_a:
                        slot_mid = (begin + end) / 2.0
                        dist = abs(slot_mid - cycle_mid)
                        if dist < best_dist:
                            best_dist = dist
                            best_flow = flow
                    lambdas.append(best_flow)
                flow_source = "xml_target_lane_timeslot"
                flow_fallback_used = False
                xml_format_used = "timeslot"

            # ==============================================================
            # 尝试格式 B：<element id="target_lane_id" traffic_flow="...">
            # ==============================================================
            if not lambdas and target_lane_id is not None:
                elem_b = root.find(f".//element[@id='{target_lane_id}']")
                if elem_b is not None:
                    tf_attr = elem_b.get("traffic_flow", "")
                    parsed = _parse_traffic_flow_attribute(tf_attr)
                    if parsed:
                        # 排序后的时间键列表
                        sorted_keys = sorted(parsed.keys())

                        # 保留 occupancy 数据用于调试
                        occ_data = {}
                        for tk, vals in parsed.items():
                            if len(vals) >= 2:
                                occ_data[tk] = vals[1]
                        if occ_data:
                            occupancy_meta = occ_data

                        # 按 cycle_mid 最近邻匹配
                        for i in range(num_cycles):
                            cycle_mid = first_cycle_start + (i + 0.5) * cycle_time
                            # 在离散时间键中找最接近 cycle_mid 的
                            best_key = sorted_keys[0]
                            best_dist = float("inf")
                            for tk in sorted_keys:
                                dist = abs(float(tk) - cycle_mid)
                                if dist < best_dist:
                                    best_dist = dist
                                    best_key = tk
                            # 取第一个值 flow_value
                            vals = parsed[best_key]
                            flow_val = vals[0] if len(vals) >= 1 else 0.0
                            lambdas.append(flow_val)

                        flow_source = "xml_target_lane_attribute"
                        flow_fallback_used = False
                        xml_format_used = "attribute"
                    else:
                        print(f"[o_stochastic_pwl WARNING] target_lane_id='{target_lane_id}' "
                              f"found in format-B XML but traffic_flow attribute is empty or unparseable: "
                              f"'{tf_attr[:80]}'")
                else:
                    # 格式 B 中也找不到 target_lane_id
                    print(f"[o_stochastic_pwl WARNING] target_lane_id='{target_lane_id}' "
                          f"not found in format-B XML (no matching <element id='...'>)")

            # ==============================================================
            # 格式 A fallback：所有 lane 混合
            # ==============================================================
            if not lambdas and all_slots_a:
                for i in range(num_cycles):
                    cycle_mid = first_cycle_start + (i + 0.5) * cycle_time
                    best_flow = 0.0
                    best_dist = float("inf")
                    for (begin, end, flow) in all_slots_a:
                        slot_mid = (begin + end) / 2.0
                        dist = abs(slot_mid - cycle_mid)
                        if dist < best_dist:
                            best_dist = dist
                            best_flow = flow
                    lambdas.append(best_flow)
                flow_source = "xml_all_lanes_fallback"
                flow_fallback_used = True
                xml_format_used = "timeslot"
                if target_lane_id is not None:
                    print(f"[o_stochastic_pwl WARNING] target_lane_id='{target_lane_id}' "
                          f"not found in flow XML (neither format A target nor format B), "
                          f"falling back to all lanes (format A)")

        except Exception as e:
            print(f"[o_stochastic_pwl WARNING] Failed to load flow XML: {e}")
            lambdas = []

    # ==============================================================
    # 最终 fallback：默认流量 600 veh/h
    # ==============================================================
    if len(lambdas) < num_cycles:
        default_flow = 600.0  # veh/h，保守默认值
        if len(lambdas) == 0:
            flow_source = "default_fallback"
            flow_fallback_used = True
            xml_format_used = "none"
        while len(lambdas) < num_cycles:
            lambdas.append(default_flow)

    flow_load_info = {
        "flow_source": flow_source,
        "target_lane_id": effective_lane_id,
        "flow_fallback_used": flow_fallback_used,
        "xml_format_used": xml_format_used,
        "occupancy_meta": occupancy_meta,
    }

    return lambdas, flow_load_info


def _sample_flows_poisson(lambda_val, cycle_time, n_samples):
    """
    基于泊松分布抽样一个周期内的流量值。

    参考 step3._sample_flows_for_cycle() 的逻辑。

    参数:
        lambda_val : float, 该周期的流量 λ (veh/h)
        cycle_time : float, 周期时长 (s)
        n_samples  : int, 抽样数量

    返回:
        np.ndarray: 流量样本 (veh/h)，长度 = n_samples
    """
    if lambda_val <= 0:
        return np.zeros(n_samples)

    delta_t_hours = cycle_time / 3600.0
    lambda_dt = lambda_val * delta_t_hours

    if lambda_dt > 100:
        # 正态近似
        N_samples = np.random.normal(lambda_dt, np.sqrt(lambda_dt), n_samples)
        N_samples = np.maximum(N_samples, 0)
    else:
        N_samples = np.random.poisson(lambda_dt, n_samples).astype(float)

    flow_samples = N_samples / delta_t_hours
    return flow_samples


def _sample_capacities(n_samples, capacity_params):
    """
    抽样通行能力值。

    [V3 修正] 优先从 Capacity XML 文件中按目标 lane 读取 raw_samples 并随机抽样。
    若 XML 中找不到目标 lane 或文件不存在，再 fallback 到参数化分布抽样。

    抽样优先级：
      1. capacity_xml_path + target_lane_id → 从 raw_samples 中随机有放回抽样
      2. dist_type == "gev" → scipy.stats.genextreme（若 scipy 可用）
      3. dist_type == "normal" → np.random.normal
      4. dist_type == "uniform" → np.random.uniform

    参数:
        n_samples      : int, 抽样数量
        capacity_params: dict or None
            通用字段:
                "capacity_xml_path": str, Capacity_GEV_Distributions.xml 路径
                "target_lane_id":   str, 目标车道 ID（如 "E_J106_J138_1"）
                "dist_type":        str, 参数化分布类型（"normal"|"gev"|"uniform"）
            normal:  {"mean": float, "std": float}
            gev:     {"shape": float, "loc": float, "scale": float}
            uniform: {"low": float, "high": float}

    返回:
        tuple: (samples, capacity_sample_info)
            samples             : np.ndarray, 通行能力样本 (veh/h)，长度 = n_samples
            capacity_sample_info: dict, {
                "capacity_dist_type":   str,  实际使用的分布/来源类型
                "capacity_params_used": dict, 实际使用的参数
            }
    """
    if capacity_params is None:
        capacity_params = {}

    # ==================================================================
    # 优先级 1：从 Capacity XML 的 raw_samples 中按目标 lane 抽样
    # ==================================================================
    cap_xml_path = capacity_params.get("capacity_xml_path", None)
    target_lane_id = capacity_params.get("target_lane_id", None)

    if cap_xml_path is not None and target_lane_id is not None:
        raw_samples = _load_raw_samples_from_capacity_xml(cap_xml_path, target_lane_id)
        if raw_samples is not None and len(raw_samples) > 0:
            # 有放回随机抽样
            indices = np.random.randint(0, len(raw_samples), size=n_samples)
            samples = np.array([raw_samples[i] for i in indices])
            samples = np.maximum(samples, 100.0)

            capacity_sample_info = {
                "capacity_dist_type": "xml_raw_samples",
                "capacity_params_used": {
                    "xml_path": cap_xml_path,
                    "target_lane_id": target_lane_id,
                    "pool_size": len(raw_samples),
                    "pool_mean": float(np.mean(raw_samples)),
                    "pool_std": float(np.std(raw_samples)),
                },
            }
            return samples, capacity_sample_info
        else:
            # 目标 lane 在 XML 中不存在，打印警告，继续 fallback
            print(f"[o_stochastic_pwl WARNING] target_lane_id='{target_lane_id}' "
                  f"not found in capacity XML, falling back to parametric distribution")

    # ==================================================================
    # 优先级 2/3/4：参数化分布抽样（fallback）
    # ==================================================================
    dist_type = capacity_params.get("dist_type", "normal")
    actual_dist_type = dist_type
    params_used = {}

    if dist_type == "gev":
        shape = capacity_params.get("shape", 0.0)
        loc = capacity_params.get("loc", _DEFAULT_CAPACITY_MEAN)
        scale = capacity_params.get("scale", _DEFAULT_CAPACITY_STD)
        params_used = {"shape": shape, "loc": loc, "scale": scale}
        try:
            from scipy.stats import genextreme
            samples = genextreme.rvs(c=shape, loc=loc, scale=scale, size=n_samples)
            actual_dist_type = "gev"
        except ImportError:
            print("[o_stochastic_pwl WARNING] GEV requested but scipy not available, "
                  "falling back to normal distribution")
            samples = np.random.normal(loc, scale, n_samples)
            actual_dist_type = "gev_fallback_normal"
            params_used["fallback_reason"] = "scipy_not_available"

    elif dist_type == "uniform":
        mean = capacity_params.get("mean", _DEFAULT_CAPACITY_MEAN)
        std = capacity_params.get("std", _DEFAULT_CAPACITY_STD)
        low = capacity_params.get("low", mean - 2 * std)
        high = capacity_params.get("high", mean + 2 * std)
        samples = np.random.uniform(low, high, n_samples)
        params_used = {"low": low, "high": high}

    else:
        # normal (默认)
        mean = capacity_params.get("mean", _DEFAULT_CAPACITY_MEAN)
        std = capacity_params.get("std", _DEFAULT_CAPACITY_STD)
        samples = np.random.normal(mean, std, n_samples)
        actual_dist_type = "normal"
        params_used = {"mean": mean, "std": std}

    samples = np.maximum(samples, 100.0)

    capacity_sample_info = {
        "capacity_dist_type": actual_dist_type,
        "capacity_params_used": params_used,
    }

    return samples, capacity_sample_info


def _load_raw_samples_from_capacity_xml(xml_path, target_lane_id):
    """
    从 Capacity_GEV_Distributions.xml 中按目标 lane ID 读取 raw_samples。

    XML 结构:
        <capacity_gev_distributions>
          <edges>
            <edge id="...">
              <direction ...>
                <lanes>E_J106_J138_1</lanes>
                <raw_samples>1500.8,1456.4,1621.7,...</raw_samples>
              </direction>
            </edge>
          </edges>
        </capacity_gev_distributions>

    参数:
        xml_path       : str, XML 文件路径
        target_lane_id : str, 目标车道 ID（匹配 <lanes> 标签内容）

    返回:
        list[float] or None: raw_samples 列表，找不到则返回 None
    """
    try:
        import xml.etree.ElementTree as ET
        tree = ET.parse(xml_path)
        root = tree.getroot()

        for direction_elem in root.findall(".//direction"):
            lanes_elem = direction_elem.find("lanes")
            if lanes_elem is None:
                continue
            lanes_text = (lanes_elem.text or "").strip()

            # 匹配逻辑：完全匹配或目标 lane 出现在 lanes 列表中
            # （有些 direction 可能有多个 lane，用逗号或空格分隔）
            lane_list = [ln.strip() for ln in lanes_text.replace(",", " ").split()]
            if target_lane_id not in lane_list:
                continue

            # 找到了目标 lane，读取 raw_samples
            raw_elem = direction_elem.find("raw_samples")
            if raw_elem is None or not raw_elem.text:
                continue

            try:
                samples = [float(s.strip()) for s in raw_elem.text.strip().split(",")
                           if s.strip()]
                if samples:
                    print(f"[o_stochastic_pwl] Loaded {len(samples)} capacity raw_samples "
                          f"for lane '{target_lane_id}' from XML")
                    return samples
            except (ValueError, TypeError):
                continue

        # 遍历完所有 direction 都没找到
        return None

    except Exception as e:
        print(f"[o_stochastic_pwl WARNING] Failed to load capacity XML '{xml_path}': {e}")
        return None


# ============================================================================
# 单场景计算（核心）
# ============================================================================

def _compute_single_scenario(virtual_arrival, capacity, flows,
                             cycle_times, cycle_time, green_time,
                             initial_queue_m,
                             startup_lost_time, max_cross_cycles,
                             avg_veh_len, jam_density, free_flow_speed):
    """
    对单个 (关键点, capacity, flow 组合) 计算实际通过时刻 t_pass。

    核心算法与 step3._compute_single_scenario_worker() 一致，
    但做了以下简化以适应在线场景：
      - 不使用 multiprocessing（任务量小，串行即可）
      - 不使用 dataclass（直接返回 tuple）
      - 初始队列来自在线读取，而非 XML

    参数:
        virtual_arrival  : float, 虚拟到达停止线时刻（绝对时间）
        capacity         : float, 通行能力 (veh/h)
        flows            : list[float], 每个周期的流量 (veh/h)
        cycle_times      : list[(start, end)], 每个周期的起止时刻
        cycle_time       : float, 周期时长 (s)
        green_time       : float, 绿灯时长 (s)
        initial_queue_m  : float, 第一个周期的初始排队长度 (m)
        startup_lost_time: float, 启动损失时间 (s)
        max_cross_cycles : int, 最大跨周期数
        avg_veh_len      : float, 平均车辆长度 (m)
        jam_density      : float, 阻塞密度 (veh/km)
        free_flow_speed  : float, 自由流速度 (km/h)

    返回:
        tuple: (t_pass_absolute, delay, cycles_crossed)
    """
    num_cycles = len(cycle_times)

    # ====== 确定虚拟到达时间所在的目标周期 ======
    target_cycle_index = None
    for i, (c_start, c_end) in enumerate(cycle_times):
        if c_start <= virtual_arrival < c_end:
            target_cycle_index = i
            break

    if target_cycle_index is None:
        if virtual_arrival < cycle_times[0][0]:
            target_cycle_index = 0
        else:
            target_cycle_index = len(cycle_times) - 1

    # ====== 逐周期冲击波演化直到目标周期 ======
    current_queue = 0.0
    sw = None

    for i in range(target_cycle_index + 1):
        c_start, c_end = cycle_times[i]
        flow = flows[i] if i < len(flows) else 0.0

        # 初始排队
        if i == 0:
            init_q = initial_queue_m
        else:
            init_q = current_queue

        sw = _SingleCycleShockwaveLite(
            arrival_rate=flow,
            capacity=capacity,
            cycle_time=cycle_time,
            green_time=green_time,
            initial_queue_length=init_q,
            average_vehicle_length=avg_veh_len,
            jam_density=jam_density,
            free_flow_speed=free_flow_speed,
            startup_lost_time=startup_lost_time,
        )
        current_queue = sw.queue_end_length

        if i < target_cycle_index:
            continue

        # ====== 目标周期：计算通过时间 ======
        t_in_cycle = virtual_arrival - c_start

        # 在这个简化版本中，虚拟到达时刻对应的队尾位置即为公交需要穿越的排队
        # (与 step3 中 bus_position=0 即公交已在停止线附近的语义一致)
        d_join = sw.get_queue_at_time(t_in_cycle)
        if sw.v_c > 0:
            H = d_join / sw.v_c
        else:
            H = 0.0

        status, t_pass_cycle, remaining_work = sw.find_pass_time(t_in_cycle, H)

        # ====== 跨周期逻辑 ======
        current_cycle_idx = target_cycle_index
        total_crossed = 0
        cycle_start_abs = c_start

        while status == _SingleCycleShockwaveLite.NEED_NEXT_CYCLE:
            total_crossed += 1
            if total_crossed > max_cross_cycles:
                break
            current_cycle_idx += 1
            if current_cycle_idx >= num_cycles:
                break

            next_start, next_end = cycle_times[current_cycle_idx]
            next_flow = flows[current_cycle_idx] if current_cycle_idx < len(flows) else flow
            next_init_q = sw.queue_end_length

            sw = _SingleCycleShockwaveLite(
                arrival_rate=next_flow,
                capacity=capacity,
                cycle_time=cycle_time,
                green_time=green_time,
                initial_queue_length=next_init_q,
                average_vehicle_length=avg_veh_len,
                jam_density=jam_density,
                free_flow_speed=free_flow_speed,
                startup_lost_time=startup_lost_time,
            )

            status, t_pass_cycle, remaining_work = sw.find_pass_time(
                0.0, 0.0, carry_over_work=remaining_work)
            cycle_start_abs = next_start

        t_pass_abs = cycle_start_abs + t_pass_cycle
        delay = max(0, t_pass_abs - virtual_arrival)
        return (t_pass_abs, delay, total_crossed)

    # 默认（不应到达）
    return (virtual_arrival, 0.0, 0)


# ============================================================================
# 关键点生成
# ============================================================================

def _generate_key_points(t_arr_lb, t_arr_ub, cycle_time, red_time, time_in_cycle,
                         current_time, startup_lost_time=2.0):
    """
    在 [t_arr_lb, t_arr_ub] 范围内生成 PWL 关键时间点。

    在信号周期边界（红灯起始、绿灯起始、有效绿灯起始）处加密采样。

    参数:
        t_arr_lb          : float, arrival window 下界
        t_arr_ub          : float, arrival window 上界
        cycle_time        : float, 信号周期时长
        red_time          : float, 红灯时长
        time_in_cycle     : float, 触发时刻在周期中的偏移（从红灯起始算起）
        current_time      : float, 当前仿真时间
        startup_lost_time : float, 启动损失时间

    返回:
        list[float]: 排好序的关键点列表
    """
    # 周期锚点
    cycle_red_start_abs = current_time - time_in_cycle

    key_times = set()
    key_times.add(round(t_arr_lb, 6))
    key_times.add(round(t_arr_ub, 6))

    n_start = math.floor((t_arr_lb - cycle_red_start_abs) / cycle_time)
    n_end = math.ceil((t_arr_ub - cycle_red_start_abs) / cycle_time) + 1

    for n in range(n_start, n_end + 1):
        t_red_start = cycle_red_start_abs + n * cycle_time
        t_green_start = t_red_start + red_time
        t_eff_green_start = t_green_start + startup_lost_time

        for t_key in [t_red_start, t_green_start, t_eff_green_start]:
            if t_arr_lb - cycle_time <= t_key <= t_arr_ub + cycle_time:
                key_times.add(round(t_key, 6))

    # 均匀补点：至少保证每周期 6 个点
    window_width = t_arr_ub - t_arr_lb
    n_uniform = max(8, int(window_width / cycle_time * 6) + 2)
    step = window_width / max(n_uniform - 1, 1)
    for i in range(n_uniform):
        t_sample = t_arr_lb + i * step
        key_times.add(round(t_sample, 6))

    # 在跳变点附近加密
    eps = 0.5
    extra = set()
    for t_key in key_times:
        if t_arr_lb <= t_key <= t_arr_ub:
            for offset in [-eps, eps]:
                t_extra = t_key + offset
                if t_arr_lb <= t_extra <= t_arr_ub:
                    extra.add(round(t_extra, 6))
    key_times.update(extra)

    # 过滤和排序
    result = sorted(t for t in key_times if t_arr_lb <= t <= t_arr_ub)

    if len(result) < 2:
        result = [t_arr_lb, t_arr_ub]

    return result


# ============================================================================
# 聚合与后处理
# ============================================================================

def _aggregate_and_correct(key_points, all_results):
    """
    按关键点聚合场景结果，然后应用 T_pass 单调性修正。

    参数:
        key_points  : list[float], 关键点列表
        all_results : list[tuple], 每个元素为 (keypoint_idx, t_pass, delay, cycles_crossed)

    返回:
        (breakpoints, values): 修正后的 PWL 断点和值
    """
    # ---- 按关键点分组 ----
    grouped = defaultdict(list)
    for (kp_idx, t_pass, delay, crossed) in all_results:
        grouped[kp_idx].append(t_pass)

    mean_t_pass = []
    for i, x in enumerate(key_points):
        t_pass_list = grouped.get(i, [])
        if len(t_pass_list) > 0:
            mean_t_pass.append(float(np.mean(t_pass_list)))
        else:
            # 无结果：t_pass = x（零延误）
            mean_t_pass.append(x)

    breakpoints = list(key_points)
    values = list(mean_t_pass)

    # ---- 强制 t_pass >= x ----
    for i in range(len(breakpoints)):
        if values[i] < breakpoints[i]:
            values[i] = breakpoints[i]

    # ---- 保序回归（PAVA 算法）----
    values = _isotonic_regression_pava(values)

    # ---- 再次强制 t_pass >= x（保序回归可能拉低某些点）----
    for i in range(len(breakpoints)):
        if values[i] < breakpoints[i]:
            values[i] = breakpoints[i]

    # ---- 兜底：再做一次单调性强制 ----
    for i in range(1, len(values)):
        if values[i] < values[i - 1]:
            values[i] = values[i - 1]

    return breakpoints, values


def _isotonic_regression_pava(y_list):
    """
    保序回归（PAVA 算法），输入和输出均为 list[float]。

    将输入序列投影到单调非递减的序列上，使 L2 距离最小。
    """
    n = len(y_list)
    if n <= 1:
        return list(y_list)

    y = [float(v) for v in y_list]
    # PAVA: Pool Adjacent Violators Algorithm
    blocks = [[i, i, y[i]] for i in range(n)]

    idx = 0
    while idx < len(blocks) - 1:
        curr_start, curr_end, curr_sum = blocks[idx]
        curr_avg = curr_sum / (curr_end - curr_start + 1)

        next_start, next_end, next_sum = blocks[idx + 1]
        next_avg = next_sum / (next_end - next_start + 1)

        if curr_avg > next_avg:
            blocks[idx] = [curr_start, next_end, curr_sum + next_sum]
            blocks.pop(idx + 1)
            if idx > 0:
                idx -= 1
        else:
            idx += 1

    result = [0.0] * n
    for block in blocks:
        start, end, total = block
        avg = total / (end - start + 1)
        for i in range(start, end + 1):
            result[i] = avg

    return result


# ============================================================================
# 主入口函数
# ============================================================================

def generate_stochastic_pwl(signal_timing, t_arr_lb, t_arr_ub, pwl_config,
                            online_context=None):
    """
    在线随机 PWL 生成器 —— 主入口函数。

    本函数被 l_realtime_signal_pwl_builder.py 通过 pwl_generator_func 接口调用。

    参数:
        signal_timing : dict, 信号配时信息（来自 builder 的 _get_signal_timing）
            必含字段: "cycle_time", "phase_time", "red_time",
                      "time_in_cycle", "current_time", "signal_id"
        t_arr_lb      : float, arrival window 下界
        t_arr_ub      : float, arrival window 上界
        pwl_config    : dict, PWL 配置参数
            可含字段: "n_flow_samples", "n_capacity_samples",
                      "stochastic_startup_lost_time", "stochastic_max_cross_cycles",
                      "flow_xml_path", "capacity_params"
        online_context: dict or None, 在线上下文信息
            可含字段:
                "bus_id"           : str
                "next_stop_id"     : str
                "first_signal_id"  : str
                "lane_id"          : str, 首个目标信号的目标进口 lane
                "trigger_time"     : float
                "current_queue_m"  : float, 来自 p_online_queue_reader
                "flow_xml_path"    : str, 离线流量 XML（优先于 pwl_config 中的）
                "capacity_params"  : dict（优先于 pwl_config 中的）
                "geo_info"         : dict

    返回:
        dict: {
            "breakpoints": list[float],
            "values":      list[float],   # through-time T_pass(x)
            "meta": {
                "generator_type":       "stochastic_online",
                "base_num_cycles":      int,
                "num_cycles":           int,
                "num_keypoints_raw":    int,
                "n_flow_samples":       int,
                "n_capacity_samples":   int,
                "n_total_scenarios":    int,
                "queue_source":         str,
                "initial_queue_m":      float,
            }
        }
        或 None（若生成失败）
    """
    if online_context is None:
        online_context = {}

    # ================================================================
    # 1. 提取参数
    # ================================================================
    cycle_time = signal_timing["cycle_time"]
    green_time = signal_timing["phase_time"]    # builder 中 phase_time = 绿灯时长
    red_time = signal_timing["red_time"]
    time_in_cycle = signal_timing.get("time_in_cycle", 0.0)
    current_time = signal_timing.get("current_time", 0.0)

    n_flow = pwl_config.get("n_flow_samples", _DEFAULT_N_FLOW_SAMPLES)
    n_cap = pwl_config.get("n_capacity_samples", _DEFAULT_N_CAPACITY_SAMPLES)
    startup_lost = pwl_config.get("stochastic_startup_lost_time", _DEFAULT_STARTUP_LOST_TIME)
    max_cross = pwl_config.get("stochastic_max_cross_cycles", _DEFAULT_MAX_CROSS_CYCLES)

    # 初始队列
    initial_queue_m = online_context.get("current_queue_m", 0.0)
    queue_source = "traci_current_lane_queue" if initial_queue_m > 0 else "none"
    if "queue_source" in online_context:
        queue_source = online_context["queue_source"]

    # 流量 XML
    flow_xml = online_context.get("flow_xml_path",
                                   pwl_config.get("flow_xml_path", None))

    # 能力参数
    cap_params = online_context.get("capacity_params",
                                     pwl_config.get("capacity_params", None))

    trigger_time = online_context.get("trigger_time", current_time)

    bus_id = online_context.get("bus_id", "unknown")
    signal_id = online_context.get("first_signal_id", signal_timing.get("signal_id", "?"))

    # ================================================================
    # 2. 确定时间范围和周期数
    # ================================================================
    if t_arr_lb >= t_arr_ub:
        print(f"[o_stochastic_pwl WARNING] t_arr_lb({t_arr_lb:.1f}) >= t_arr_ub({t_arr_ub:.1f})")
        return None

    # 锚定第一个周期的红灯起始
    cycle_red_start_abs = current_time - time_in_cycle

    # 找到覆盖 arrival window 的周期范围
    n_first = math.floor((t_arr_lb - cycle_red_start_abs) / cycle_time)
    n_last = math.ceil((t_arr_ub - cycle_red_start_abs) / cycle_time)

    first_cycle_start = cycle_red_start_abs + n_first * cycle_time
    base_num_cycles = max(1, n_last - n_first)
    num_cycles = base_num_cycles + max_cross

    cycle_times = []
    for i in range(num_cycles):
        c_start = first_cycle_start + i * cycle_time
        c_end = c_start + cycle_time
        cycle_times.append((c_start, c_end))

    # ================================================================
    # 3. 生成关键点
    # ================================================================
    key_points = _generate_key_points(
        t_arr_lb, t_arr_ub, cycle_time, red_time, time_in_cycle,
        current_time, startup_lost)

    if len(key_points) < 2:
        print(f"[o_stochastic_pwl WARNING] Too few key points: {len(key_points)}")
        return None

    # ================================================================
    # 4. 抽样
    # ================================================================
    # 4a. 流量 λ（按目标 lane 过滤）
    target_lane_id = online_context.get("lane_id", None)
    flow_lambdas, flow_load_info = _load_flow_lambda_from_xml(
        flow_xml, trigger_time, cycle_time, num_cycles, first_cycle_start,
        target_lane_id=target_lane_id)

    # 4b. 每周期流量样本
    flow_samples_per_cycle = []
    for i in range(num_cycles):
        samples = _sample_flows_poisson(flow_lambdas[i], cycle_time, n_flow)
        flow_samples_per_cycle.append(samples)

    # 4c. 通行能力样本
    # [V3] 将 target_lane_id 和 capacity_xml_path 注入 cap_params，
    # 使 _sample_capacities 能从 XML raw_samples 中按目标 lane 抽样
    if cap_params is None:
        cap_params = {}
    if "target_lane_id" not in cap_params and target_lane_id is not None:
        cap_params = dict(cap_params)  # 避免修改外部传入的 dict
        cap_params["target_lane_id"] = target_lane_id
    if "capacity_xml_path" not in cap_params:
        # capacity_xml_path 可从 pwl_config 或 online_context 中获取
        cap_xml_from_config = pwl_config.get("capacity_xml_path", None)
        if cap_xml_from_config is not None:
            cap_params = dict(cap_params) if not isinstance(cap_params, dict) else dict(cap_params)
            cap_params["capacity_xml_path"] = cap_xml_from_config
    capacity_samples, capacity_sample_info = _sample_capacities(n_cap, cap_params)

    # ================================================================
    # 5. 逐场景计算（串行，在线实时性要求不使用多进程）
    # ================================================================
    all_results = []
    total_scenarios = 0

    for kp_idx, virtual_arrival in enumerate(key_points):
        for cap_idx, cap_val in enumerate(capacity_samples):
            for flow_idx in range(n_flow):
                # 构建本场景的每周期流量
                flows = []
                for ci in range(num_cycles):
                    if ci < len(flow_samples_per_cycle) and len(flow_samples_per_cycle[ci]) > 0:
                        si = flow_idx % len(flow_samples_per_cycle[ci])
                        flows.append(float(flow_samples_per_cycle[ci][si]))
                    else:
                        flows.append(0.0)

                t_pass, delay, crossed = _compute_single_scenario(
                    virtual_arrival=virtual_arrival,
                    capacity=float(cap_val),
                    flows=flows,
                    cycle_times=cycle_times,
                    cycle_time=cycle_time,
                    green_time=green_time,
                    initial_queue_m=initial_queue_m,
                    startup_lost_time=startup_lost,
                    max_cross_cycles=max_cross,
                    avg_veh_len=_DEFAULT_AVG_VEHICLE_LENGTH,
                    jam_density=_DEFAULT_JAM_DENSITY,
                    free_flow_speed=_DEFAULT_FREE_FLOW_SPEED,
                )
                all_results.append((kp_idx, t_pass, delay, crossed))
                total_scenarios += 1

    # ================================================================
    # 6. 聚合 + 后处理
    # ================================================================
    breakpoints, values = _aggregate_and_correct(key_points, all_results)

    if len(breakpoints) < 2:
        print(f"[o_stochastic_pwl WARNING] Aggregation produced < 2 points")
        return None

    # ================================================================
    # 7. 组装输出
    # ================================================================
    print(f"[o_stochastic_pwl] bus={bus_id}, sig={signal_id}: "
          f"{len(breakpoints)} pts, {total_scenarios} scenarios "
          f"({n_cap}cap × {n_flow}flow × {len(key_points)}kp), "
          f"q0={initial_queue_m:.1f}m, cycles={num_cycles}, "
          f"flow_src={flow_load_info['flow_source']}, "
          f"cap_dist={capacity_sample_info['capacity_dist_type']}")

    # 将 capacity_params_used 序列化为字符串（用于 XML 日志）
    cap_params_str = str(capacity_sample_info.get("capacity_params_used", {}))

    return {
        "breakpoints": breakpoints,
        "values": values,
        "meta": {
            # === 已有字段 ===
            "generator_type": "stochastic_online",
            "base_num_cycles": base_num_cycles,
            "num_cycles": num_cycles,
            "num_keypoints_raw": len(key_points),
            "n_flow_samples": n_flow,
            "n_capacity_samples": n_cap,
            "n_total_scenarios": total_scenarios,
            "queue_source": queue_source,
            "initial_queue_m": initial_queue_m,
            # === [V2 新增] 流量来源追踪 ===
            "flow_source": flow_load_info["flow_source"],
            "target_lane_id": flow_load_info["target_lane_id"],
            "flow_fallback_used": flow_load_info["flow_fallback_used"],
            # === [V4 新增] XML 格式追踪 ===
            "xml_format_used": flow_load_info["xml_format_used"],
            "occupancy_meta": flow_load_info.get("occupancy_meta", None),
            # === [V2 新增] 能力分布追踪 ===
            "capacity_dist_type": capacity_sample_info["capacity_dist_type"],
            "capacity_params_used": cap_params_str,
        }
    }
