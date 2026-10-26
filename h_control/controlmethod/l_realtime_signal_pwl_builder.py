import math
import copy
import m_signal_geometry_mapping

# [NEW] Stochastic PWL + Physical Window 改造：在线队列读取
try:
    import p_online_queue_reader
except ImportError:
    p_online_queue_reader = None
    print("[l_realtime_signal_pwl_builder WARNING] p_online_queue_reader not found, "
          "online queue reading disabled")


# ========================================================================
# [NEW] 局部窗口辅助函数
# ========================================================================

def _get_reference_speed_for_local_window(
    bus_obj,
    last_control_result,
    lb_speed,
    up_speed,
    fallback_ref_speed=10.0,
    min_ref_speed_floor=0.5,
    geo_info=None,
    current_time=None,          # [BUGFIX Round-1] 保留此参数以兼容调用方，当前版本暂不使用
):
    """
    计算用于局部窗口定位的参考速度 v_ref。

    优先级（从高到低）：
      1. 上轮控制结果中，该公交在 first_signal_edge_id（首个目标信号所在路段）上的决策速度
         → v_ref_source = "last_control_target_edge"
         理由：局部窗口中心 t_ref = t_start + dist_to_first_signal / v_ref，
               因此 v_ref 应尽量反映"起点到首个目标信号"这段的速度，
               而非远端无关路段的速度。
      2. 公交当前瞬时速度（需 > min_ref_speed_floor）
         → v_ref_source = "current_speed"
      3. fallback_ref_speed（硬编码兜底）
         → v_ref_source = "fallback_ref_speed"

    参数:
        bus_obj              : 公交车对象
        last_control_result  : dict or None, 上轮控制结果
                               格式: {bus_id: {edge_id: speed_value, ...}, ...}
        lb_speed             : float, 最低速度 (m/s)
        up_speed             : float, 最高速度 (m/s)
        fallback_ref_speed   : float, 兜底参考速度 (m/s)
        min_ref_speed_floor  : float, 瞬时速度低于此值则视为无效 (m/s)
        geo_info             : dict or None, 几何信息
                               来自 m_signal_geometry_mapping.get_signals_between_bus_and_next_stop()
                               需包含 "first_signal_edge_id" 字段

    返回:
        tuple: (v_ref, v_ref_source, target_edge_id)
            v_ref            : float, 参考速度 (m/s)，已 clamp 到 [lb_speed+0.01, up_speed-0.01]
            v_ref_source     : str, 来源标识
            target_edge_id   : str or None, 优先级 1 中使用的目标路段 ID
    """
    bus_id = bus_obj.bus_id_s
    v_ref = None
    v_ref_source = ""

    # 从 geo_info 中提取首个信号所在路段 ID
    target_edge_id = None
    if geo_info is not None:
        target_edge_id = geo_info.get("first_signal_edge_id")

    # ---- 优先级 1：上轮控制结果中 first_signal_edge_id 对应的速度 ----
    if (v_ref is None
            and target_edge_id is not None
            and last_control_result is not None
            and bus_id in last_control_result):
        edge_speed = last_control_result[bus_id].get(target_edge_id)
        if edge_speed is not None and edge_speed > min_ref_speed_floor:
            v_ref = edge_speed
            v_ref_source = "last_control_target_edge"

    # ---- 优先级 2：当前瞬时速度 ----
    if v_ref is None:
        if (hasattr(bus_obj, 'bus_speed_n')
                and bus_obj.bus_speed_n is not None
                and bus_obj.bus_speed_n > min_ref_speed_floor):
            v_ref = bus_obj.bus_speed_n
            v_ref_source = "current_speed"

    # ---- 优先级 3：硬编码兜底 ----
    if v_ref is None:
        v_ref = fallback_ref_speed
        v_ref_source = "fallback_ref_speed"

    # clamp 到 [lb_speed + 0.01, up_speed - 0.01]
    v_ref = max(lb_speed + 0.01, min(up_speed - 0.01, v_ref))

    return v_ref, v_ref_source, target_edge_id


def _compute_local_window(t_start, dist, lb_speed, up_speed, v_ref, delta_left, delta_right):
    """
    计算两层窗口（物理窗 + 局部窗）。

    物理窗（全局速度极限决定）：
        x_phys_lb = t_start + dist / up_speed   （最快到达）
        x_phys_ub = t_start + dist / lb_speed   （最慢到达）

    局部窗（以参考到达时刻为中心，左右偏移 delta）：
        t_ref = t_start + dist / v_ref
        x_local_lb = t_ref - delta_left
        x_local_ub = t_ref + delta_right

    实际使用窗口 = 物理窗 ∩ 局部窗：
        x_used_lb = max(x_phys_lb, x_local_lb)
        x_used_ub = min(x_phys_ub, x_local_ub)

    若局部窗退化（x_used_lb >= x_used_ub），回退到物理窗。

    参数:
        t_start     : float, 公交出发时刻
        dist        : float, 到第一个信号停止线的距离 (m)
        lb_speed    : float, 最低速度 (m/s)
        up_speed    : float, 最高速度 (m/s)
        v_ref       : float, 参考速度 (m/s)
        delta_left  : float, 局部窗口左偏移 (s)
        delta_right : float, 局部窗口右偏移 (s)

    返回:
        tuple: (x_used_lb, x_used_ub, x_phys_lb, x_phys_ub, t_ref)
    """
    # 物理窗
    x_phys_lb = t_start + dist / up_speed
    x_phys_ub = t_start + dist / lb_speed

    # 参考到达时刻
    t_ref = t_start + dist / v_ref

    # 局部窗 = 物理窗 ∩ [t_ref - delta_left, t_ref + delta_right]
    x_used_lb = max(x_phys_lb, t_ref - delta_left)
    x_used_ub = min(x_phys_ub, t_ref + delta_right)

    # # 局部窗 = 物理窗   调试使用
    # x_used_lb = x_phys_lb
    # x_used_ub = x_phys_ub

    # 安全检查：若局部窗退化，回退到物理窗
    if x_used_lb >= x_used_ub:
        print(f"[Local Window] 局部窗退化: x_used_lb={x_used_lb:.2f} >= x_used_ub={x_used_ub:.2f}, "
              f"回退到物理窗 [{x_phys_lb:.2f}, {x_phys_ub:.2f}]")
        x_used_lb = x_phys_lb
        x_used_ub = x_phys_ub

    return x_used_lb, x_used_ub, x_phys_lb, x_phys_ub, t_ref


def _adaptive_num_points(x_used_lb, x_used_ub, signal_timing, pwl_config,
                         pwl_generator_meta=None):
    """
    根据局部窗口覆盖的信号周期数，自适应计算 PWL 断点数。

    计算公式：
        covered_cycles = window_width / cycle_time
        n_points = points_per_cycle * covered_cycles + cycle_boundary_bonus
        n_points = clamp(n_points, min_points, max_points)

    当外部 PWL 生成函数返回了 meta 信息且含 num_cycles 时，优先使用。

    参数:
        x_used_lb          : float, 实际使用窗口下界
        x_used_ub          : float, 实际使用窗口上界
        signal_timing      : dict, 信号配时信息（需含 "cycle_time"）
        pwl_config         : dict, PWL 配置参数
        pwl_generator_meta : dict or None, 外部 PWL 函数返回的元数据

    返回:
        int : 自适应断点数
    """
    cycle_time = signal_timing["cycle_time"]
    points_per_cycle = pwl_config.get("points_per_cycle", 8)
    min_points = pwl_config.get("min_points", 10)
    max_points = pwl_config.get("max_points", 60)
    bonus = pwl_config.get("cycle_boundary_bonus", 4)

    # 优先使用外部元数据中的周期数
    if pwl_generator_meta and "num_cycles" in pwl_generator_meta:
        covered_cycles = pwl_generator_meta["num_cycles"]
    else:
        window_width = x_used_ub - x_used_lb
        covered_cycles = window_width / cycle_time if cycle_time > 0 else 1.0

    n_points = int(points_per_cycle * covered_cycles + bonus)
    n_points = min(max(min_points, n_points), max_points)

    return n_points


# ========================================================================
# 主入口函数
# ========================================================================

def build_signal_pwl_bundles(
    current_time,
    bus_obj_l,
    line_obj_dic,
    signal_obj_dic,
    sorted_busline_edge_d,
    busline_stop_edge_d,
    station_dict,
    lb_speed,
    up_speed,
    pwl_generator_func=None,
    pwl_config=None,
    # ===== [NEW] PWL 局部窗口改造新增参数 =====
    last_control_result=None,     # 上轮控制结果 {bus_id: {edge_id: speed}}, 用于提取 v_ref
    rebuild_overrides=None,       # dict: {(bus_id, stop_id): {"v_ref":..., "delta_left":..., ...}}
                                  # 若非 None，仅对指定公交重建 bundle，其余复用 existing_bundles
    existing_bundles=None,        # dict: 上一轮的 bundle（rebuild 时复用未修改的公交）
    # ===== [NEW] Stochastic PWL + Physical Window 改造新增参数 =====
    window_mode_override=None,    # str or None
                                  # "physical_window" → 跳过局部窗口，直接使用 physical window
                                  # None → 使用正常的局部窗口逻辑
):
    """
    为本次控制周期的所有候选公交构建"下一站"信号 PWL 数据包。

    [PWL 局部窗口改造] 新增逻辑：
    - 若 rebuild_overrides 不为 None，仅对其中指定的公交重建 bundle，
      其余公交直接从 existing_bundles 中复用
    - 计算 v_ref 时使用 _get_reference_speed_for_local_window()
    - 使用 _compute_local_window() 计算局部窗口
    - 使用 _adaptive_num_points() 自适应断点数

    [Stochastic PWL + Physical Window 改造] 新增逻辑：
    - window_mode_override="physical_window" 时：
      所有公交跳过局部窗口，直接使用 physical window 生成 PWL
      bundle 中 x_lb=x_phys_lb, x_ub=x_phys_ub
    - pwl_generator_func 调用时传入 online_context 参数（第5参数）

    参数:
        current_time         : float, 当前仿真时间（触发时刻）
        bus_obj_l            : list, 公交车对象列表（已简化后的）
        line_obj_dic         : dict, 线路对象字典
        signal_obj_dic       : dict, 信号灯对象字典
        sorted_busline_edge_d: dict, 各线路排好顺序的路段元素字典
        busline_stop_edge_d  : dict, 各线路公交站对应的路段ID
        station_dict         : dict, 各线路公交站顺序
        lb_speed             : float, 最低速度 (m/s)
        up_speed             : float, 最高速度 (m/s)
        pwl_generator_func   : callable or None, 外部 PWL 生成函数
                               签名（旧）: pwl_generator_func(signal_timing, t_arr_lb, t_arr_ub, config)
                               签名（新）: pwl_generator_func(signal_timing, t_arr_lb, t_arr_ub, config,
                                                              online_context=ctx)
                               返回: (breakpoints_list, values_list) 或
                                     {"breakpoints":..., "values":..., "meta":...} 或 None
                               若为 None，则使用内置的解析式 PWL 生成方法
        pwl_config           : dict or None, PWL 生成参数配置
        last_control_result  : dict or None, 上轮控制决策速度
        rebuild_overrides    : dict or None, 重建参数覆盖
        existing_bundles     : dict or None, 上一轮的 bundle（rebuild 时复用）
        window_mode_override : str or None, 窗口模式覆盖
                               "physical_window" → 所有公交跳过局部窗口
                               None → 正常局部窗口逻辑

    返回:
        signal_pwl_bundle_d : dict
            key   = (bus_id, next_stop_id)
            value = { "case_type": str, ... }
    """

    if pwl_config is None:
        pwl_config = {
            "num_points": 20,
            "extra_cycles": 1,
            "delta_left": 30.0,
            "delta_right": 30.0,
            "fallback_ref_speed": 10.0,
            "min_ref_speed_floor": 0.5,
            "points_per_cycle": 8,
            "min_points": 10,
            "max_points": 60,
            "cycle_boundary_bonus": 4,
        }

    # ================================================================
    # Step 1: 批量获取所有公交的信号几何信息
    # ================================================================
    signal_geometry_d = m_signal_geometry_mapping.get_all_bus_signal_geometry(
        bus_obj_l=bus_obj_l,
        line_obj_dic=line_obj_dic,
        signal_obj_dic=signal_obj_dic,
        sorted_busline_edge_d=sorted_busline_edge_d,
        busline_stop_edge_d=busline_stop_edge_d,
        station_dict=station_dict,
    )

    # ================================================================
    # Step 2: 对每辆公交逐一构建 PWL 数据包
    # ================================================================
    signal_pwl_bundle_d = {}

    for bus_obj in bus_obj_l:
        bus_id = bus_obj.bus_id_s
        next_stop_id = bus_obj.next_stop_id_s
        bundle_key = (bus_id, next_stop_id)

        # ---- [NEW] 重建模式：判断是否需要重建该公交的 bundle ----
        if rebuild_overrides is not None:
            if bundle_key not in rebuild_overrides:
                # 不需要重建的公交 → 从 existing_bundles 中复用
                if existing_bundles is not None and bundle_key in existing_bundles:
                    signal_pwl_bundle_d[bundle_key] = copy.deepcopy(existing_bundles[bundle_key])
                else:
                    # existing_bundles 中也没有，回退为 no_signal
                    signal_pwl_bundle_d[bundle_key] = {"case_type": "no_signal"}
                continue
            # 需要重建的公交 → 使用 rebuild_overrides 中的参数（下面继续构建）

        # 获取几何信息
        geo_key = (bus_id, next_stop_id)
        if geo_key not in signal_geometry_d:
            signal_pwl_bundle_d[bundle_key] = {"case_type": "no_signal"}
            continue

        geo_info = signal_geometry_d[geo_key]
        num_signals = geo_info["num_signals_before_next_stop"]

        # ----------------------------------------------------------
        # Case A：下一站前没有信号
        # ----------------------------------------------------------
        if num_signals == 0:
            signal_pwl_bundle_d[bundle_key] = {"case_type": "no_signal"}
            continue

        # ----------------------------------------------------------
        # [NEW] 计算 v_ref 和获取 delta 参数 + 构建 PWL bundle
        # [BUGFIX Round-1] 将 v_ref 计算和 bundle 构建放进同一个
        #   per-bus try/except，单辆公交失败只影响该公交
        # ----------------------------------------------------------
        try:
            # 若有 rebuild_overrides 中的覆盖值则优先使用
            override = rebuild_overrides.get(bundle_key, {}) if rebuild_overrides else {}

            # v_ref
            if "v_ref" in override and override["v_ref"] is not None:
                v_ref = override["v_ref"]
            else:
                v_ref, _v_ref_src, _ref_edge = _get_reference_speed_for_local_window(
                    bus_obj=bus_obj,
                    last_control_result=last_control_result,
                    lb_speed=lb_speed,
                    up_speed=up_speed,
                    fallback_ref_speed=pwl_config.get("fallback_ref_speed", 10.0),
                    min_ref_speed_floor=pwl_config.get("min_ref_speed_floor", 0.5),
                    geo_info=geo_info,
                    current_time=current_time,
                )

            # delta
            delta_left = override.get("delta_left", pwl_config.get("delta_left", 30.0))
            delta_right = override.get("delta_right", pwl_config.get("delta_right", 30.0))
            window_mode = override.get("window_mode", "local_initial")
            rebuild_round = override.get("rebuild_round", 0)

            # [NEW] Stochastic PWL + Physical Window 改造：
            # 若 window_mode_override 被设置，覆盖 window_mode 和 rebuild_round
            if window_mode_override == "physical_window":
                window_mode = "physical_window"
                rebuild_round = 2

            # ----------------------------------------------------------
            # Case B / Case C：需要构建 PWL
            # ----------------------------------------------------------
            bundle = _build_pwl_bundle_for_first_signal(
                bus_obj=bus_obj,
                geo_info=geo_info,
                current_time=current_time,
                sorted_busline_edge_d=sorted_busline_edge_d,
                signal_obj_dic=signal_obj_dic,
                lb_speed=lb_speed,
                up_speed=up_speed,
                pwl_generator_func=pwl_generator_func,
                pwl_config=pwl_config,
                # ===== [NEW] 局部窗口参数 =====
                v_ref=v_ref,
                delta_left=delta_left,
                delta_right=delta_right,
                window_mode=window_mode,
                rebuild_round=rebuild_round,
            )

            if bundle is not None:
                signal_pwl_bundle_d[bundle_key] = bundle
            else:
                # PWL 构建失败，回退为 no_signal
                print(f"[PWL Builder WARNING] PWL build returned None for "
                      f"{bus_id}->{next_stop_id}, fallback to no_signal")
                signal_pwl_bundle_d[bundle_key] = {"case_type": "no_signal"}

        except Exception as e:
            print(f"[PWL Builder ERROR] Exception in v_ref/PWL build for "
                  f"{bus_id}->{next_stop_id}: {e}, fallback to no_signal")
            signal_pwl_bundle_d[bundle_key] = {"case_type": "no_signal"}

    # ================================================================
    # Step 3: 汇总日志
    # ================================================================
    _print_summary_log(signal_pwl_bundle_d, current_time)

    return signal_pwl_bundle_d


# ========================================================================
# 核心内部函数：为首个信号构建 PWL bundle
# ========================================================================

def _build_pwl_bundle_for_first_signal(
    bus_obj,
    geo_info,
    current_time,
    sorted_busline_edge_d,
    signal_obj_dic,
    lb_speed,
    up_speed,
    pwl_generator_func,
    pwl_config,
    # ===== [NEW] 局部窗口参数 =====
    v_ref=None,
    delta_left=30.0,
    delta_right=30.0,
    window_mode="local_initial",
    rebuild_round=0,
    last_control_result=None,
):
    """
    为公交到下一站途中的第一个信号灯构建 PWL 数据包。

    对于 Case B（1个信号）和 Case C（多个信号）均调用此函数。
    Case C 时额外计算后续信号的平均延误总和。

    [PWL 局部窗口改造] 关键改动：
    1. 不再直接用 up_speed/lb_speed 计算 t_arr_lb/t_arr_ub，改为调用 _compute_local_window()
    2. 不再用固定 num_points，改为调用 _adaptive_num_points()
    3. Bundle 中新增 x_phys_lb, x_phys_ub, t_ref, v_ref, delta_left, delta_right, window_mode, rebuild_round

    返回:
        dict (signal_pwl_bundle) 或 None（若构建失败）
    """

    bus_id = bus_obj.bus_id_s
    next_stop_id = bus_obj.next_stop_id_s
    line_id = bus_obj.belong_line_id_s
    num_signals = geo_info["num_signals_before_next_stop"]

    first_signal_id = geo_info["first_signal_id"]
    first_signal_edge_id = geo_info["first_signal_edge_id"]
    first_signal_lane_id = geo_info["first_signal_lane_id"]
    dist_start_to_first_signal = geo_info["dist_start_to_first_signal"]
    dist_first_signal_to_stop = geo_info["dist_first_signal_to_stop"]

    # ================================================================
    # 1. 确定 t_start（公交出发时刻）
    # ================================================================
    t_start = _get_bus_start_time(bus_obj, current_time)

    # ================================================================
    # 2. [MODIFIED] 计算局部 arrival-time window
    #    改为使用 _compute_local_window()，基于 v_ref 和 delta 计算
    # ================================================================
    if dist_start_to_first_signal <= 0:
        # 公交已经过了（或恰好在）第一个信号停止线，不需要 PWL
        print(f"[PWL Builder INFO] {bus_id}: dist_to_first_signal={dist_start_to_first_signal:.1f} <= 0, "
              f"skip PWL")
        return None

    # 若 v_ref 未提供，使用 fallback 计算
    # [BUGFIX Round-1] 预初始化 v_ref_source / ref_target_edge_id，
    #   避免 v_ref 已由调用方传入时下方日志和 bundle 中引用未定义变量
    v_ref_source = "caller_provided"
    ref_target_edge_id = None

    if v_ref is None:
        v_ref, v_ref_source, ref_target_edge_id = _get_reference_speed_for_local_window(
            bus_obj=bus_obj,
            last_control_result=last_control_result,
            lb_speed=lb_speed,
            up_speed=up_speed,
            fallback_ref_speed=pwl_config.get("fallback_ref_speed", 10.0),
            min_ref_speed_floor=pwl_config.get("min_ref_speed_floor", 0.5),
            geo_info=geo_info,
        )

    # 计算两层窗口
    x_used_lb, x_used_ub, x_phys_lb, x_phys_ub, t_ref = _compute_local_window(
        t_start=t_start,
        dist=dist_start_to_first_signal,
        lb_speed=lb_speed,
        up_speed=up_speed,
        v_ref=v_ref,
        delta_left=delta_left,
        delta_right=delta_right,
    )

    # [NEW] Stochastic PWL + Physical Window 改造：
    # 当 window_mode == "physical_window" 时，跳过局部窗口，直接使用 physical window
    if window_mode == "physical_window":
        t_arr_lb = x_phys_lb
        t_arr_ub = x_phys_ub
    else:
        # 使用局部窗口作为 PWL 的 arrival window
        t_arr_lb = x_used_lb
        t_arr_ub = x_used_ub

    if t_arr_lb >= t_arr_ub:
        print(f"[PWL Builder WARNING] {bus_id}: t_arr_lb({t_arr_lb:.1f}) >= "
              f"t_arr_ub({t_arr_ub:.1f}), skip PWL")
        return None

    # [V2 修正] 窗口日志：明确区分 physical / local_candidate / used
    # used_window_type 用于标记实际使用了哪种窗口，避免与 window_mode（承担 round 语义）混淆
    used_window_type = "physical" if window_mode == "physical_window" else "local"
    print(f"[PWL Builder Window] bus={bus_id}, stop={next_stop_id}: "
          f"v_ref={v_ref:.2f}({v_ref_source}), t_ref={t_ref:.1f}, "
          f"phys=[{x_phys_lb:.1f}, {x_phys_ub:.1f}], "
          f"local_candidate=[{x_used_lb:.1f}, {x_used_ub:.1f}], "
          f"used=[{t_arr_lb:.1f}, {t_arr_ub:.1f}], "
          f"used_type={used_window_type}, "
          f"delta_L={delta_left:.1f}, delta_R={delta_right:.1f}, "
          f"mode={window_mode}, round={rebuild_round}")

    # ================================================================
    # 3. 获取信号灯配时信息
    # ================================================================
    signal_timing = _get_signal_timing(
        first_signal_id=first_signal_id,
        first_signal_edge_id=first_signal_edge_id,
        line_id=line_id,
        sorted_busline_edge_d=sorted_busline_edge_d,
        signal_obj_dic=signal_obj_dic,
        bus_obj=bus_obj,
        current_time=current_time,
    )

    if signal_timing is None:
        print(f"[PWL Builder WARNING] {bus_id}: cannot get signal timing for "
              f"signal={first_signal_id}, skip PWL")
        return None

    # ================================================================
    # 4. [MODIFIED] 自适应断点数
    # ================================================================
    num_points = _adaptive_num_points(
        x_used_lb=t_arr_lb,
        x_used_ub=t_arr_ub,
        signal_timing=signal_timing,
        pwl_config=pwl_config,
    )

    # ================================================================
    # 5. 生成 PWL 断点和值
    # ================================================================
    pwl_generator_meta = None  # 用于接收外部生成函数可能返回的元数据

    if pwl_generator_func is not None:
        # 使用外部 PWL 生成函数
        try:
            # [NEW] Stochastic PWL + Physical Window 改造：
            # 构建 online_context 并尝试传给 pwl_generator_func
            # 包含在线队列、流量/能力参数等上下文信息
            online_context = _build_online_context_for_generator(
                bus_obj=bus_obj,
                geo_info=geo_info,
                current_time=current_time,
                first_signal_id=first_signal_id,
                first_signal_lane_id=first_signal_lane_id,
                pwl_config=pwl_config,
            )

            # 尝试新签名 (5 参数，含 online_context)
            try:
                result = pwl_generator_func(signal_timing, t_arr_lb, t_arr_ub,
                                            pwl_config, online_context=online_context)
            except TypeError:
                # 旧签名 (4 参数) 兼容
                result = pwl_generator_func(signal_timing, t_arr_lb, t_arr_ub, pwl_config)

            if result is not None:
                # [NEW] 兼容新返回格式 {"breakpoints":..., "values":..., "meta":...}
                if isinstance(result, dict):
                    breakpoints = result.get("breakpoints")
                    values = result.get("values")
                    pwl_generator_meta = result.get("meta")
                elif isinstance(result, (tuple, list)) and len(result) >= 2:
                    breakpoints, values = result[0], result[1]
                else:
                    breakpoints, values = None, None
            else:
                breakpoints, values = None, None
        except Exception as e:
            print(f"[PWL Builder ERROR] External PWL generator failed for "
                  f"{bus_id}: {e}")
            breakpoints, values = None, None

        # [NEW] 若外部 generator 失败，回退到内置解析式（仅作 fallback）
        if breakpoints is None or values is None:
            print(f"[PWL Builder INFO] {bus_id}: external generator returned None, "
                  f"falling back to analytical PWL")
            breakpoints, values = _generate_through_time_pwl_analytical(
                signal_timing=signal_timing,
                t_arr_lb=t_arr_lb,
                t_arr_ub=t_arr_ub,
                num_points=num_points,
            )
            # [V2] 标记 fallback 来源
            pwl_generator_meta = {"generator_type": "analytical_fallback"}
    else:
        # 使用内置解析式 PWL 生成方法，传入自适应断点数
        breakpoints, values = _generate_through_time_pwl_analytical(
            signal_timing=signal_timing,
            t_arr_lb=t_arr_lb,
            t_arr_ub=t_arr_ub,
            num_points=num_points,
        )
        # [V2] 无外部 generator 时标记
        pwl_generator_meta = {"generator_type": "analytical_builtin"}

    # [NEW] 若外部函数返回了 meta 且含 num_cycles，可用于重新计算断点数
    # 当外部函数已经生成了足够的断点时，不再重算
    if (pwl_generator_meta is not None
            and breakpoints is not None
            and len(breakpoints) >= 2):
        adaptive_pts = _adaptive_num_points(
            x_used_lb=t_arr_lb,
            x_used_ub=t_arr_ub,
            signal_timing=signal_timing,
            pwl_config=pwl_config,
            pwl_generator_meta=pwl_generator_meta,
        )
        # 如果外部函数生成的断点数少于自适应建议值，用内置方法重新生成
        if len(breakpoints) < adaptive_pts * 0.5:
            print(f"[PWL Builder INFO] {bus_id}: external generator produced "
                  f"{len(breakpoints)} points < adaptive suggestion {adaptive_pts}, "
                  f"regenerating with built-in method")
            breakpoints, values = _generate_through_time_pwl_analytical(
                signal_timing=signal_timing,
                t_arr_lb=t_arr_lb,
                t_arr_ub=t_arr_ub,
                num_points=adaptive_pts,
            )

    # ================================================================
    # 6. 校验 PWL 数据
    # ================================================================
    if not _validate_pwl_data(breakpoints, values, bus_id, next_stop_id):
        return None

    # ================================================================
    # 7. Case C：计算后续信号的平均延误总和
    # ================================================================
    extra_avg_delay = 0.0
    num_remaining_signals = 0
    case_type = "one_signal"

    if num_signals > 1:
        case_type = "multi_signal_first_pwl"
        remaining_sigs = geo_info.get("remaining_signals_after_first", [])
        num_remaining_signals = len(remaining_sigs)
        for sig_info in remaining_sigs:
            extra_avg_delay += sig_info.get("avg_delay", 0.0)

    # ================================================================
    # 8. [MODIFIED] 组装 bundle —— 新增窗口元数据字段
    # ================================================================
    bundle = {
        # === 原有字段（保留） ===
        "case_type": case_type,
        "first_signal_id": first_signal_id,
        "first_signal_edge_id": first_signal_edge_id,
        "lane_id": first_signal_lane_id,
        "dist_start_to_first_signal": dist_start_to_first_signal,
        "dist_first_signal_to_stop": dist_first_signal_to_stop,
        "breakpoints": breakpoints,
        "values": values,
        "x_lb": t_arr_lb,                    # 实际使用的 arrival window 下界
        "x_ub": t_arr_ub,                    # 实际使用的 arrival window 上界
                                              # physical_window 模式下 = x_phys_lb/x_phys_ub
                                              # local 模式下 = local window ∩ physical window
        "extra_avg_delay_after_first_signal": extra_avg_delay,
        "num_remaining_signals": num_remaining_signals,
        "y_axis_meaning": "crossing_time",

        # === [NEW] 窗口元数据字段 ===
        "x_phys_lb": x_phys_lb,              # 全局物理窗下界
        "x_phys_ub": x_phys_ub,              # 全局物理窗上界
        "t_ref": t_ref,                       # 参考到达时刻
        "v_ref": v_ref,                       # 参考速度 (m/s)
        "v_ref_source": v_ref_source,
        "ref_target_edge_id": ref_target_edge_id,
        "delta_left": delta_left,             # 局部窗口左偏移 (s)
        "delta_right": delta_right,           # 局部窗口右偏移 (s)
        "window_mode": window_mode,           # 窗口模式标签 (round 语义)
        "rebuild_round": rebuild_round,       # 当前重建轮次

        # === [V2 新增] 窗口类型与 generator 元信息 ===
        "used_window_type": used_window_type,  # "local" | "physical"
                                                # 明确标记实际使用了哪种窗口
        "pwl_generator_meta": pwl_generator_meta,  # dict or None
                                                     # stochastic_online 时含完整 meta
                                                     # analytical_fallback / analytical_builtin 时含类型标记
    }

    return bundle


# ========================================================================
# [NEW] 在线上下文构建辅助函数
# ========================================================================

def _build_online_context_for_generator(
    bus_obj, geo_info, current_time,
    first_signal_id, first_signal_lane_id,
    pwl_config,
):
    """
    为 pwl_generator_func（如 o_stochastic_signal_pwl_generator.generate_stochastic_pwl）
    构建在线上下文 dict。

    包含：
    - 公交和信号标识信息
    - 在线队列（通过 p_online_queue_reader 即时读取）
    - 流量 XML 路径和能力分布参数（从 pwl_config 中提取）

    参数:
        bus_obj              : 公交车对象
        geo_info             : dict, 几何信息
        current_time         : float, 触发时刻
        first_signal_id      : str, 首个目标信号 ID
        first_signal_lane_id : str, 首个目标信号的进口车道 ID
        pwl_config           : dict, PWL 配置参数

    返回:
        dict: online_context
    """
    bus_id = bus_obj.bus_id_s
    next_stop_id = bus_obj.next_stop_id_s

    # ---- 在线队列读取 ----
    current_queue_m = 0.0
    queue_source = "none"
    if p_online_queue_reader is not None and first_signal_lane_id is not None:
        try:
            current_queue_m = p_online_queue_reader.read_queue_for_signal_lane(
                first_signal_lane_id, method="e2")
            queue_source = "traci_current_lane_queue"
        except Exception as e:
            print(f"[PWL Builder WARNING] Queue read failed for "
                  f"{bus_id}->{next_stop_id}, lane={first_signal_lane_id}: {e}")
            current_queue_m = 0.0
            queue_source = "read_failed"

    context = {
        "bus_id": bus_id,
        "next_stop_id": next_stop_id,
        "first_signal_id": first_signal_id,
        "lane_id": first_signal_lane_id,
        "trigger_time": current_time,
        "current_queue_m": current_queue_m,
        "queue_source": queue_source,
        "flow_xml_path": pwl_config.get("flow_xml_path", None),
        "capacity_params": pwl_config.get("capacity_params", None),
        "geo_info": geo_info,
    }

    return context


# ========================================================================
# 辅助函数：获取公交出发时刻
# ========================================================================

def _get_bus_start_time(bus_obj, current_time):
    """
    确定公交车的出发时刻 t_start。

    语义与当前 define_next_stop_arrival_time_con 中完全一致：
    - 若公交在站间（bus_state_s != "Stop"）：t_start = current_time（触发时刻）
    - 若公交在站内（bus_state_s == "Stop"）：t_start = 离站时刻
      离站时刻来自 just_server_stop_data_d[last_stop][1]
    """
    if bus_obj.bus_state_s != "Stop":
        return current_time
    else:
        just_server_stop_data_d = bus_obj.just_server_stop_data_d
        last_served_stop = next(reversed(just_server_stop_data_d), None)
        if last_served_stop is not None:
            return just_server_stop_data_d[last_served_stop][1]
        else:
            # 回退：使用当前时间
            return current_time


# ========================================================================
# 辅助函数：获取信号灯配时信息
# ========================================================================

def _get_signal_timing(
    first_signal_id,
    first_signal_edge_id,
    line_id,
    sorted_busline_edge_d,
    signal_obj_dic,
    bus_obj,
    current_time,
):
    """
    获取第一个信号灯的完整配时信息，供 PWL 生成使用。

    返回:
        dict: {
            "signal_id":      str,   # 信号灯 tl ID
            "phase_time":     float, # 绿灯相位时长 (s)
            "cycle_time":     float, # 信号周期时长 (s)
            "red_time":       float, # 红灯时长 = cycle_time - phase_time
            "from_lane":      str,   # 进口车道 ID
            "current_time":   float, # 当前仿真时间
            "time_in_cycle":  float, # 触发时刻在当前信号周期中的已过时间
                                     # （从该周期红灯起始算起的秒数）
            "avg_delay":      float, # 平均延误（与 subsequent_signal_info_d 一致）
        }
        或 None（若无法获取）
    """

    # 从 sorted_busline_edge_d 中获取配时信息
    # "i" 类型路段: [type, length, dedicated_flag, tl, phase_time, cycle_time, from_lane, dir]
    #                [0]    [1]      [2]           [3]    [4]         [5]        [6]       [7]
    edge_info = None
    if (line_id in sorted_busline_edge_d
            and first_signal_edge_id in sorted_busline_edge_d[line_id]):
        edge_info = sorted_busline_edge_d[line_id][first_signal_edge_id]

    if edge_info is None or edge_info[0] != "i":
        return None

    try:
        phase_time = float(edge_info[4])    # 绿灯相位时长
        cycle_time = float(edge_info[5])    # 信号周期时长
    except (ValueError, TypeError, IndexError):
        return None

    if cycle_time <= 0 or phase_time <= 0 or phase_time >= cycle_time:
        return None

    red_time = cycle_time - phase_time
    from_lane = edge_info[6] if len(edge_info) > 6 else None

    # 从 subsequent_signal_info_d 获取时间在周期中的偏移以及平均延误
    time_in_cycle = 0.0
    avg_delay = 0.0
    if (hasattr(bus_obj, 'subsequent_signal_info_d')
            and bus_obj.subsequent_signal_info_d
            and first_signal_id in bus_obj.subsequent_signal_info_d):
        sig_info = bus_obj.subsequent_signal_info_d[first_signal_id]
        # subsequent_signal_info_d[signal_id] 的结构：
        # 根据现有代码 tl_info_l[5] 用于 "time elapsed since last red start"
        # tl_info_l[-1] 或 tl_info_l[7] 是平均延误
        # 这里我们从 sig_info 中按相同索引提取
        if sig_info and len(sig_info) > 5:
            try:
                time_in_cycle = float(sig_info[5])  # 触发时刻距上次红灯起始的时间
            except (ValueError, TypeError):
                time_in_cycle = 0.0
        if sig_info and len(sig_info) > 0:
            try:
                avg_delay = float(sig_info[-1])
            except (ValueError, TypeError):
                avg_delay = 0.0

    return {
        "signal_id": first_signal_id,
        "phase_time": phase_time,
        "cycle_time": cycle_time,
        "red_time": red_time,
        "from_lane": from_lane,
        "current_time": current_time,
        "time_in_cycle": time_in_cycle,
        "avg_delay": avg_delay,
    }


# ========================================================================
# 内置解析式 PWL 生成方法
# ========================================================================

def _generate_through_time_pwl_analytical(
    signal_timing,
    t_arr_lb,
    t_arr_ub,
    num_points=20,
):
    """
    基于信号配时参数，解析式地生成 through-time PWL 曲线。

    物理模型（简化版，忽略排队长度效应）：
    对于虚拟到达停止线时刻 t_arr，实际通过停止线时刻 T_pass(t_arr) 按以下逻辑：
    
    1. 将 t_arr 映射到信号周期中的相对位置
    2. 若 t_arr 落在绿灯期间（可通行）：T_pass = t_arr（零延误）
    3. 若 t_arr 落在红灯期间（需等待）：T_pass = 下一个绿灯起始时刻
       （简化：不考虑排队消散时间，即假设通行能力足够大；
         若需要更精确的排队消散模型，请通过 pwl_generator_func 传入外部函数）

    through-time 物理约束保证：
    - T_pass(t_arr) >= t_arr       （不能"穿越"回过去）
    - T_pass 单调非递减             （后到的不能先过）

    参数:
        signal_timing : dict, 信号配时信息（来自 _get_signal_timing）
        t_arr_lb      : float, 到达时刻下界
        t_arr_ub      : float, 到达时刻上界
        num_points    : int, 采样点数（不含关键跳变点）

    返回:
        (breakpoints, values) : (list[float], list[float])
        或 (None, None) 若生成失败
    """

    cycle_time = signal_timing["cycle_time"]
    phase_time = signal_timing["phase_time"]       # 绿灯时长
    red_time = signal_timing["red_time"]             # 红灯时长
    time_in_cycle = signal_timing["time_in_cycle"]   # 触发时刻在周期中的偏移
    current_time = signal_timing["current_time"]

    if cycle_time <= 0 or phase_time <= 0:
        return None, None

    # ================================================================
    # 计算信号周期的绝对时间锚点
    # ================================================================
    # time_in_cycle 是触发时刻距其所在周期红灯起始的时间偏移
    # 当前周期的红灯起始绝对时刻：
    cycle_red_start_abs = current_time - time_in_cycle

    # ================================================================
    # 生成覆盖 [t_arr_lb, t_arr_ub] 的关键时刻点
    # ================================================================
    # 在 arrival window 内，我们需要找到所有红绿灯切换时刻，
    # 这些是 PWL 曲线的关键跳变点。
    # 然后在相邻跳变点之间均匀插值。

    # 找到覆盖 arrival window 的所有周期边界
    key_times = set()
    key_times.add(t_arr_lb)
    key_times.add(t_arr_ub)

    # 从红灯起始绝对时刻开始，生成覆盖范围内的所有信号切换时刻
    # 需要找到第一个 <= t_arr_lb 的周期起始
    n_start = math.floor((t_arr_lb - cycle_red_start_abs) / cycle_time)
    n_end = math.ceil((t_arr_ub - cycle_red_start_abs) / cycle_time) + 1

    for n in range(n_start, n_end + 1):
        # 第 n 个周期的红灯起始
        t_red_start = cycle_red_start_abs + n * cycle_time
        # 第 n 个周期的绿灯起始 = 红灯起始 + 红灯时长
        t_green_start = t_red_start + red_time

        # 只保留在 arrival window 内或附近的关键点
        for t_key in [t_red_start, t_green_start]:
            if t_arr_lb - cycle_time <= t_key <= t_arr_ub + cycle_time:
                key_times.add(t_key)

    # 排序关键时刻
    key_times_sorted = sorted(key_times)

    # ================================================================
    # 在关键时刻之间插入均匀采样点
    # ================================================================
    all_sample_points = set()
    for t_key in key_times_sorted:
        if t_arr_lb <= t_key <= t_arr_ub:
            all_sample_points.add(t_key)

    # 在 [t_arr_lb, t_arr_ub] 上均匀采样
    if num_points > 2:
        step = (t_arr_ub - t_arr_lb) / (num_points - 1)
        for i in range(num_points):
            t_sample = t_arr_lb + i * step
            all_sample_points.add(round(t_sample, 6))

    # 在关键跳变点附近加密采样（红灯起始前后加 ±0.5s 以捕获跳变）
    epsilon = 0.5
    extra_points = set()
    for t_key in key_times_sorted:
        if t_arr_lb <= t_key <= t_arr_ub:
            for offset in [-epsilon, epsilon]:
                t_extra = t_key + offset
                if t_arr_lb <= t_extra <= t_arr_ub:
                    extra_points.add(round(t_extra, 6))
    all_sample_points.update(extra_points)

    # 排序去重
    breakpoints = sorted(all_sample_points)

    if len(breakpoints) < 2:
        return None, None

    # ================================================================
    # 计算每个断点的 T_pass 值
    # ================================================================
    values = []
    for t_arr in breakpoints:
        t_pass = _compute_t_pass_analytical(
            t_arr=t_arr,
            cycle_red_start_abs=cycle_red_start_abs,
            cycle_time=cycle_time,
            red_time=red_time,
            phase_time=phase_time,
        )
        values.append(t_pass)

    # ================================================================
    # 后处理：确保 through-time 物理约束
    # ================================================================
    breakpoints, values = _enforce_through_time_constraints(breakpoints, values)

    return breakpoints, values


def _compute_t_pass_analytical(t_arr, cycle_red_start_abs, cycle_time, red_time, phase_time):
    """
    解析式计算单个到达时刻的通过时刻 T_pass。

    信号周期结构（从红灯起始算起）：
        [0, red_time)          → 红灯（等待至绿灯起始才能通过）
        [red_time, cycle_time) → 绿灯（直接通过）

    参数:
        t_arr               : float, 虚拟到达停止线时刻
        cycle_red_start_abs : float, 参考周期的红灯起始绝对时刻
        cycle_time          : float, 信号周期时长
        red_time            : float, 红灯时长
        phase_time          : float, 绿灯时长

    返回:
        float: T_pass，实际通过停止线时刻
    """
    # 计算 t_arr 在信号周期中的相对位置
    t_rel = (t_arr - cycle_red_start_abs) % cycle_time

    if t_rel < 0:
        t_rel += cycle_time

    if t_rel >= red_time:
        # 到达时处于绿灯相位 → 可直接通过
        return t_arr
    else:
        # 到达时处于红灯相位 → 需要等到本周期绿灯起始
        wait_time = red_time - t_rel
        return t_arr + wait_time


def _enforce_through_time_constraints(breakpoints, values):
    """
    后处理：确保 PWL 数据满足 through-time 物理约束：
    1. y >= x    （T_pass >= t_arr）
    2. y 单调非递减

    若有违反，进行修正。
    """
    n = len(breakpoints)
    if n == 0:
        return breakpoints, values

    # 约束 1: y >= x
    for i in range(n):
        if values[i] < breakpoints[i]:
            values[i] = breakpoints[i]

    # 约束 2: y 单调非递减
    for i in range(1, n):
        if values[i] < values[i - 1]:
            values[i] = values[i - 1]

    return breakpoints, values


# ========================================================================
# PWL 数据校验
# ========================================================================

def _validate_pwl_data(breakpoints, values, bus_id, next_stop_id):
    """
    校验 PWL 数据的合法性。

    检查项：
    1. breakpoints 和 values 非空且长度一致
    2. 长度 >= 2
    3. breakpoints 严格递增
    4. values 满足 y >= x
    5. values 单调非递减

    返回: True (通过) / False (不通过)
    """
    tag = f"[PWL Validate] {bus_id}->{next_stop_id}"

    # 检查 1: 非空且长度一致
    if breakpoints is None or values is None:
        print(f"{tag}: breakpoints or values is None")
        return False

    if len(breakpoints) != len(values):
        print(f"{tag}: len mismatch: breakpoints={len(breakpoints)}, values={len(values)}")
        return False

    # 检查 2: 长度 >= 2
    if len(breakpoints) < 2:
        print(f"{tag}: too few points: {len(breakpoints)}")
        return False

    # 检查 3: breakpoints 严格递增
    for i in range(1, len(breakpoints)):
        if breakpoints[i] <= breakpoints[i - 1]:
            print(f"{tag}: breakpoints not strictly increasing at index {i}: "
                  f"{breakpoints[i-1]} >= {breakpoints[i]}")
            return False

    # 检查 4: y >= x
    violations_yx = 0
    for i in range(len(breakpoints)):
        if values[i] < breakpoints[i] - 1e-6:  # 允许微小数值误差
            violations_yx += 1

    if violations_yx > 0:
        print(f"{tag}: {violations_yx} points violate y >= x constraint "
              f"(after enforce should be 0, check enforce logic)")
        return False

    # 检查 5: values 单调非递减
    violations_mono = 0
    for i in range(1, len(values)):
        if values[i] < values[i - 1] - 1e-6:
            violations_mono += 1

    if violations_mono > 0:
        print(f"{tag}: {violations_mono} points violate monotonicity "
              f"(after enforce should be 0, check enforce logic)")
        return False

    return True


# ========================================================================
# 汇总日志
# ========================================================================

def _print_summary_log(signal_pwl_bundle_d, current_time):
    """输出本次控制周期 PWL 构建汇总日志。"""

    total = len(signal_pwl_bundle_d)
    case_a = 0
    case_b = 0
    case_c = 0
    fallback = 0

    case_b_details = []
    case_c_details = []

    for key, bundle in signal_pwl_bundle_d.items():
        bus_id, stop_id = key
        ct = bundle["case_type"]

        if ct == "no_signal":
            case_a += 1
        elif ct == "one_signal":
            case_b += 1
            # [NEW] 增加窗口元数据输出
            case_b_details.append(
                f"    {bus_id}->{stop_id}: sig={bundle.get('first_signal_id','?')}, "
                f"pts={len(bundle.get('breakpoints',[]))}, "
                f"x=[{bundle.get('x_lb',0):.1f}, {bundle.get('x_ub',0):.1f}], "
                f"phys=[{bundle.get('x_phys_lb',0):.1f}, {bundle.get('x_phys_ub',0):.1f}], "
                f"v_ref={bundle.get('v_ref',0):.2f}, "
                f"mode={bundle.get('window_mode','?')}, "
                f"round={bundle.get('rebuild_round',0)}"
            )
        elif ct == "multi_signal_first_pwl":
            case_c += 1
            case_c_details.append(
                f"    {bus_id}->{stop_id}: sig={bundle.get('first_signal_id','?')}(PWL)"
                f"+{bundle.get('num_remaining_signals',0)} avg_delay, "
                f"pts={len(bundle.get('breakpoints',[]))}, "
                f"x=[{bundle.get('x_lb',0):.1f}, {bundle.get('x_ub',0):.1f}], "
                f"phys=[{bundle.get('x_phys_lb',0):.1f}, {bundle.get('x_phys_ub',0):.1f}], "
                f"extra_delay={bundle.get('extra_avg_delay_after_first_signal',0):.1f}s, "
                f"v_ref={bundle.get('v_ref',0):.2f}, "
                f"mode={bundle.get('window_mode','?')}, "
                f"round={bundle.get('rebuild_round',0)}"
            )
        else:
            fallback += 1

    print(f"\n====== [Signal PWL Builder] Control cycle at t={current_time:.1f} ======")
    print(f"  Total buses:                  {total}")
    print(f"  Case A (no signal):           {case_a}")
    print(f"  Case B (one signal PWL):      {case_b}")
    print(f"  Case C (first-sig PWL + avg): {case_c}")
    if fallback > 0:
        print(f"  Fallback to no_signal:        {fallback}")

    for detail in case_b_details:
        print(detail)
    for detail in case_c_details:
        print(detail)

    print(f"{'=' * 60}")
