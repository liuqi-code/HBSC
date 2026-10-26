import copy
import time as time_module   # 用于测量求解用时，与 traci 的 simulation time 区分
import random
import gurobipy
import h_subfunction_for_gurobi
import l_realtime_signal_pwl_builder

# 设置种子为41、38
random.seed(41)


# ============================================================================
# PWL 局部窗口相关参数集中定义
# 所有 PWL 相关参数在此处统一管理，便于调试和调参。
# ============================================================================
PWL_CONFIG = {
    # --- 局部窗口 ---
    "delta_left": 130.0,                       # 局部窗口左偏移 (s)
    "delta_right": 50.0,                      # 局部窗口右偏移 (s)
    "fallback_ref_speed": 12.0,               # v_ref 回退速度 (m/s)
                                              # [注] 仍用于 _get_reference_speed_for_local_window 优先级3兜底；
                                              #      不再用于 infeasible 主重建分支
    "min_ref_speed_floor": 0.5,               # 瞬时速度低于此值则使用 fallback (m/s)

    # --- 重建参数 ---
    "rebuild_expand_factor_hit_boundary": 1.3, # 命中边界时 delta 扩张因子 (Round 1 使用)
    "rebuild_expand_factor_infeasible": 1.7,   # [兼容保留] 不再参与当前主流程 Round 2
                                               # 原用于 rebuild_from_fallback，现已被 pwl_physical_window 替代
    "max_rebuild_round": 3,                    # 最大 round 数 (含 avg_delay_only)

    # --- 边界检查 ---
    "boundary_hit_eps_abs": 0.5,              # 绝对 eps (s)
    "boundary_hit_eps_rel": 0.02,             # 相对 eps (相对于窗口宽度)

    # --- PWL 断点自适应 ---
    "points_per_cycle": 8,                    # 每个信号周期的断点数
    "min_points": 10,                         # 最少断点数
    "max_points": 60,                         # 最多断点数
    "cycle_boundary_bonus": 4,                # 周期边界额外加点数
    "extra_cycles": 1,                        # arrival window 额外覆盖周期数

    # --- PWL 生成（兼容旧配置） ---
    "num_points": 20,                         # 旧固定断点数（兼容用）

    # --- [NEW] Stochastic PWL 生成参数（为 o_stochastic_signal_pwl_generator 准备） ---
    "n_flow_samples": 5,                      # 流量蒙特卡洛抽样数
    "n_capacity_samples": 5,                  # 通行能力蒙特卡洛抽样数
    "stochastic_startup_lost_time": 2.0,      # 冲击波模型启动损失时间 τ (s)
    "stochastic_max_cross_cycles": 2,         # 最大跨周期数
    "flow_xml_path": None,                    # 离线流量 XML 文件路径（由 control_sim 传入）
    "capacity_params": None,                  # 能力分布参数 dict: {mean, std, dist_type}（由 control_sim 传入）
}


# ===========================================================================
# 全局控制周期计数器（用于 solve_history 中的 control_cycle_id）
# ===========================================================================
_control_cycle_counter = 0


def _get_next_control_cycle_id():
    """获取下一个控制周期 ID（自增计数器）"""
    global _control_cycle_counter
    _control_cycle_counter += 1
    return _control_cycle_counter


def reset_control_cycle_counter():
    """重置控制周期计数器为 0。

    在每次仿真开始前调用，避免同一 Python 进程中多次运行仿真时
    （如参数扫描、批量实验）计数器持续累加导致 control_cycle_id 不从 1 开始。
    """
    global _control_cycle_counter
    _control_cycle_counter = 0


# ============================================================================
# 回调函数
# ============================================================================

def my_callback(model, where):
    """Gurobi 求解过程回调：当 MIP Gap 小于设定阈值时提前终止"""
    if where == gurobipy.GRB.Callback.MIP:
        best_obj = model.cbGet(gurobipy.GRB.Callback.MIP_OBJBST)   # 当前最优整数解
        best_bound = model.cbGet(gurobipy.GRB.Callback.MIP_OBJBND)  # 当前最优界
        if best_obj != gurobipy.GRB.INFINITY and best_bound != -gurobipy.GRB.INFINITY:
            gap = abs(best_bound - best_obj) / max(abs(best_obj), 1e-10)
            # print(f"当前 GAP: {gap:.5f}")
            if gap < 0.01:
                print("GAP 小于 0.01，终止求解！")
                model.terminate()


# ============================================================================
# 辅助函数（与原版一致，不改动）
# ============================================================================

def judge_stop_chuanche(stop_obj_d, specified_headway, trigger_time):
    """判断某个车站是否发生串车"""
    for stop_id in stop_obj_d:
        service_data_l = stop_obj_d[stop_id].service_data_l
        scaled_service_data_l = []
        for service_data in service_data_l:
            if service_data[2] < trigger_time - 300:
                scaled_service_data_l.append(service_data)
        scaled_busline_service_data_d = {}
        for service_data in scaled_service_data_l:
            if service_data[1] not in scaled_busline_service_data_d:
                scaled_busline_service_data_d[service_data[1]] = [service_data]
            else:
                scaled_busline_service_data_d[service_data[1]].append(service_data)
        for busline in scaled_busline_service_data_d:
            busline_service_data_l = scaled_busline_service_data_d[busline]
            for index, service_data in enumerate(busline_service_data_l):
                if index < len(busline_service_data_l) - 1:
                    if (busline_service_data_l[index + 1][2] - service_data[2] < specified_headway
                            and service_data[0].split("_")[1] != "0"):
                        print(f"{service_data[0]}与{busline_service_data_l[index+1][0]}在{stop_id}发生串车！！！")


def Simplified_bus_and_stop_dict(origin_bus_obj_dic, origin_stop_obj_dic, first_stop_num,
                                 scaled_line_station_od_otd_dict):
    """补充好各公交站的刚服务完的路段"""
    bus_obj_dic = copy.deepcopy(origin_bus_obj_dic)
    stop_obj_dic = copy.deepcopy(origin_stop_obj_dic)

    for bus_obj_id in bus_obj_dic:
        bus_obj_dic[bus_obj_id].unserved_stop_l = bus_obj_dic[bus_obj_id].unserved_stop_l[
            :min(first_stop_num, len(bus_obj_dic[bus_obj_id].unserved_stop_l))]

    scaled_stop_od = {}
    for stop_obj_id in stop_obj_dic:
        stop_obj_dic[stop_obj_id].unserved_bus_l = []
        stop_obj_dic[stop_obj_id].service_line_l = []
        scaled_od = {}
        if stop_obj_id in scaled_stop_od:
            stop_obj_dic[stop_obj_id].just_leave_data_l[-1] = copy.deepcopy(scaled_stop_od[stop_obj_id])
        else:
            for od_otd in stop_obj_dic[stop_obj_id].just_leave_data_l[-1]:
                if od_otd in scaled_line_station_od_otd_dict[
                    stop_obj_dic[stop_obj_id].just_leave_data_l[1]][stop_obj_id]:
                    scaled_od[od_otd] = copy.deepcopy(stop_obj_dic[stop_obj_id].just_leave_data_l[-1][od_otd])
            stop_obj_dic[stop_obj_id].just_leave_data_l[-1] = copy.deepcopy(scaled_od)
            scaled_stop_od[stop_obj_id] = copy.deepcopy(scaled_od)

    for bus_obj_id in bus_obj_dic:
        bus_obj_unserved_stop_l = bus_obj_dic[bus_obj_id].unserved_stop_l
        for unserved_stop_id in bus_obj_unserved_stop_l:
            stop_obj_dic[unserved_stop_id].unserved_bus_l.append(bus_obj_id)
            if bus_obj_id.split("_")[0] not in stop_obj_dic[unserved_stop_id].service_line_l:
                stop_obj_dic[unserved_stop_id].service_line_l.append(bus_obj_id.split("_")[0])

    return bus_obj_dic, stop_obj_dic


def get_bus_stop_obj_l(simplified_bus_obj_dic, simplified_stop_obj_dic):
    """获取已激活的公交车和有未服务公交车的公交站列表"""
    bus_obj_l = []
    stop_obj_l = []
    bus_obj_id_l = []
    stop_obj_id_l = []
    for bus_id in simplified_bus_obj_dic:
        if (simplified_bus_obj_dic[bus_id].bus_state_s != "No"
                and simplified_bus_obj_dic[bus_id].unserved_stop_l):
            bus_obj_l.append(simplified_bus_obj_dic[bus_id])
            bus_obj_id_l.append(simplified_bus_obj_dic[bus_id].bus_id_s)
    for stop_id in simplified_stop_obj_dic:
        if simplified_stop_obj_dic[stop_id].unserved_bus_l:
            stop_obj_l.append(simplified_stop_obj_dic[stop_id])
            stop_obj_id_l.append(simplified_stop_obj_dic[stop_id].stop_id_s)
    return bus_obj_l, stop_obj_l, bus_obj_id_l, stop_obj_id_l


# ============================================================================
# [NEW] PWL 边界检查与重建相关函数
# ============================================================================

def _extract_pwl_solve_info(model_aux_var_d, signal_pwl_bundle_d, bus_obj_l,
                            decision_variable_inv_d,
                            boundary_hit_eps_abs=0.5, boundary_hit_eps_rel=0.02):
    """
    从已求解的 Gurobi 模型中提取 PWL 相关求解信息，并判断是否命中 arrival window 边界。

    参数:
        model_aux_var_d        : dict, PWL 辅助变量字典
                                 key = (bus_id, next_stop_id, "t_arr_sig") 或 (bus_id, next_stop_id, "t_cross_sig")
                                 value = Gurobi 变量对象
        signal_pwl_bundle_d    : dict or None, PWL 数据包
        bus_obj_l              : list, 公交车对象列表
        decision_variable_inv_d: dict, 速度倒数决策变量字典
        boundary_hit_eps_abs   : float, 边界命中判定绝对阈值 (s)
        boundary_hit_eps_rel   : float, 边界命中判定相对阈值 (占窗口宽度比例)

    返回:
        pwl_solve_info : dict
            key = (bus_id, next_stop_id)
            value = {
                "used_pwl": bool,
                "case_type": str,
                "window_mode": str,
                "rebuild_round": int,
                "t_arr_sig_sol": float or None,
                "t_cross_sig_sol": float or None,
                "x_lb": float,
                "x_ub": float,
                "x_phys_lb": float,
                "x_phys_ub": float,
                "dist_to_lb": float,
                "dist_to_ub": float,
                "hit_lower_boundary": bool,
                "hit_upper_boundary": bool,
                "hit_any_boundary": bool,
                "rebuild_suggested": bool,
                "v_ref": float,
                "delta_left": float,
                "delta_right": float,
                "first_signal_id": str,
                "decision_speed_sol": float or None,
            }
    """
    pwl_solve_info = {}

    if signal_pwl_bundle_d is None:
        return pwl_solve_info

    for bus_obj in bus_obj_l:
        bus_id = bus_obj.bus_id_s
        next_stop_id = bus_obj.next_stop_id_s
        bundle_key = (bus_id, next_stop_id)

        if bundle_key not in signal_pwl_bundle_d:
            continue

        bundle = signal_pwl_bundle_d[bundle_key]
        case_type = bundle.get("case_type", "no_signal")

        # 只关注使用了 PWL 的公交
        if case_type not in ("one_signal", "multi_signal_first_pwl"):
            pwl_solve_info[bundle_key] = {
                "used_pwl": False,
                "case_type": case_type,
                "window_mode": bundle.get("window_mode", ""),
                "rebuild_round": bundle.get("rebuild_round", 0),
                "t_arr_sig_sol": None,
                "t_cross_sig_sol": None,
                "x_lb": 0.0, "x_ub": 0.0,
                "x_phys_lb": 0.0, "x_phys_ub": 0.0,
                "dist_to_lb": 0.0, "dist_to_ub": 0.0,
                "hit_lower_boundary": False,
                "hit_upper_boundary": False,
                "hit_any_boundary": False,
                "rebuild_suggested": False,
                "v_ref": bundle.get("v_ref", 0.0),
                "delta_left": bundle.get("delta_left", 0.0),
                "delta_right": bundle.get("delta_right", 0.0),
                "first_signal_id": bundle.get("first_signal_id", ""),
                "v_ref_source": bundle.get("v_ref_source", ""),
                "ref_target_edge_id": bundle.get("ref_target_edge_id", ""),
                "decision_speed_sol": None,
            }
            continue

        # 提取 t_arr_sig 和 t_cross_sig 的求解值
        t_arr_key = (bus_id, next_stop_id, "t_arr_sig")
        t_cross_key = (bus_id, next_stop_id, "t_cross_sig")

        t_arr_sig_sol = None
        t_cross_sig_sol = None
        if t_arr_key in model_aux_var_d:
            try:
                t_arr_sig_sol = model_aux_var_d[t_arr_key].X
            except Exception:
                pass
        if t_cross_key in model_aux_var_d:
            try:
                t_cross_sig_sol = model_aux_var_d[t_cross_key].X
            except Exception:
                pass

        # 窗口边界
        x_lb = bundle.get("x_lb", 0.0)
        x_ub = bundle.get("x_ub", 0.0)
        x_phys_lb = bundle.get("x_phys_lb", x_lb)
        x_phys_ub = bundle.get("x_phys_ub", x_ub)

        # 提取决策速度（速度倒数的倒数）
        decision_speed_sol = None
        if bundle_key in decision_variable_inv_d:
            try:
                inv_speed_sol = decision_variable_inv_d[bundle_key].X
                if inv_speed_sol > 1e-9:
                    decision_speed_sol = 1.0 / inv_speed_sol
            except Exception:
                pass

        # 边界判定
        hit_lower = False
        hit_upper = False
        dist_to_lb = 0.0
        dist_to_ub = 0.0

        if t_arr_sig_sol is not None and (x_ub - x_lb) > 1e-6:
            window_width = x_ub - x_lb
            eps = max(boundary_hit_eps_abs, boundary_hit_eps_rel * window_width)

            dist_to_lb = t_arr_sig_sol - x_lb
            dist_to_ub = x_ub - t_arr_sig_sol

            hit_lower = (dist_to_lb <= eps)
            hit_upper = (dist_to_ub <= eps)

        hit_any = hit_lower or hit_upper
        rebuild_suggested = hit_any

        pwl_solve_info[bundle_key] = {
            "used_pwl": True,
            "case_type": case_type,
            "window_mode": bundle.get("window_mode", "local_initial"),
            "rebuild_round": bundle.get("rebuild_round", 0),
            "t_arr_sig_sol": t_arr_sig_sol,
            "t_cross_sig_sol": t_cross_sig_sol,
            "x_lb": x_lb,
            "x_ub": x_ub,
            "x_phys_lb": x_phys_lb,
            "x_phys_ub": x_phys_ub,
            "dist_to_lb": dist_to_lb,
            "dist_to_ub": dist_to_ub,
            "hit_lower_boundary": hit_lower,
            "hit_upper_boundary": hit_upper,
            "hit_any_boundary": hit_any,
            "rebuild_suggested": rebuild_suggested,
            "v_ref": bundle.get("v_ref", 0.0),
            "delta_left": bundle.get("delta_left", 0.0),
            "delta_right": bundle.get("delta_right", 0.0),
            "first_signal_id": bundle.get("first_signal_id", ""),
            "v_ref_source": bundle.get("v_ref_source", ""),
            "ref_target_edge_id": bundle.get("ref_target_edge_id", ""),
            "decision_speed_sol": decision_speed_sol,
        }

        # 日志输出
        if hit_any:
            print(f"[PWL Boundary Check] bus={bus_id}, stop={next_stop_id}: "
                  f"t_arr_sol={t_arr_sig_sol:.2f}, "
                  f"window=[{x_lb:.2f}, {x_ub:.2f}], "
                  f"dist_to_lb={dist_to_lb:.2f}, dist_to_ub={dist_to_ub:.2f}, "
                  f"hit_lower={hit_lower}, hit_upper={hit_upper}")

    return pwl_solve_info


def _check_boundary_hits(pwl_solve_info):
    """
    从 pwl_solve_info 中筛选出命中 arrival window 边界的公交列表。

    参数:
        pwl_solve_info : dict, _extract_pwl_solve_info() 的返回值

    返回:
        hit_list : list of (bus_id, next_stop_id) tuples
    """
    hit_list = []
    for key, info in pwl_solve_info.items():
        if info.get("used_pwl", False) and info.get("hit_any_boundary", False):
            hit_list.append(key)
    return hit_list


def _build_rebuild_overrides_from_solution(pwl_solve_info, hit_list, signal_pwl_bundle_d,
                                           expand_factor=1.5):
    """
    根据 Round 0 的求解结果，为命中边界的公交计算新的重建参数。

    策略：
    - v_ref 使用该公交在 Round 0 中的求解速度（decision_speed_sol）
    - delta_left/delta_right 按 expand_factor 扩张
    - window_mode 设为 "rebuild_from_solution"
    - rebuild_round 设为 1

    参数:
        pwl_solve_info      : dict, Round 0 的 PWL 求解信息
        hit_list            : list, 命中边界的 (bus_id, next_stop_id) 列表
        signal_pwl_bundle_d : dict, Round 0 的 PWL bundle
        expand_factor       : float, delta 扩张因子

    返回:
        rebuild_overrides : dict
            key = (bus_id, next_stop_id)
            value = {"v_ref": float, "delta_left": float, "delta_right": float,
                     "window_mode": str, "rebuild_round": int}
    """
    rebuild_overrides = {}

    for key in hit_list:
        info = pwl_solve_info.get(key, {})
        bundle = signal_pwl_bundle_d.get(key, {})

        # 从求解速度获取 v_ref；若不可用则使用原 v_ref
        v_ref = info.get("decision_speed_sol")
        if v_ref is None or v_ref <= 0:
            v_ref = info.get("v_ref", PWL_CONFIG["fallback_ref_speed"])

        # 取原 delta 进行扩张
        old_delta_left = bundle.get("delta_left", PWL_CONFIG["delta_left"])
        old_delta_right = bundle.get("delta_right", PWL_CONFIG["delta_right"])

        rebuild_overrides[key] = {
            "v_ref": v_ref,
            "delta_left": expand_factor * old_delta_left,
            "delta_right": expand_factor * old_delta_right,
            "window_mode": "rebuild_from_solution",
            "rebuild_round": 1,
        }

        print(f"[Rebuild Override (from solution)] {key}: "
              f"v_ref={v_ref:.2f}, "
              f"delta_left={expand_factor * old_delta_left:.1f}, "
              f"delta_right={expand_factor * old_delta_right:.1f}")

    return rebuild_overrides


def _build_rebuild_overrides_from_fallback(signal_pwl_bundle_d, fallback_speed=10.0,
                                           expand_factor=2.0):
    """
    [兼容保留 — 不再被当前主流程 Round 2 调用]

    为所有使用了 PWL 的公交生成 fallback 重建参数。
    原用于 Round 2 (rebuild_from_fallback)，现已被 pwl_physical_window 替代。
    保留此函数以兼容可能的外部调用或回退测试。

    策略：
    - 所有 PWL 公交都重建
    - v_ref 使用 fallback_speed
    - delta_left/delta_right 按 expand_factor 扩张
    - window_mode 设为 "rebuild_from_fallback"
    - rebuild_round 设为 2

    参数:
        signal_pwl_bundle_d : dict, 上一轮的 PWL bundle
        fallback_speed      : float, fallback 参考速度 (m/s)
        expand_factor       : float, delta 扩张因子

    返回:
        rebuild_overrides : dict
    """
    rebuild_overrides = {}

    if signal_pwl_bundle_d is None:
        return rebuild_overrides

    for key, bundle in signal_pwl_bundle_d.items():
        case_type = bundle.get("case_type", "no_signal")
        if case_type not in ("one_signal", "multi_signal_first_pwl"):
            continue  # 只对使用了 PWL 的公交进行重建

        old_delta_left = bundle.get("delta_left", PWL_CONFIG["delta_left"])
        old_delta_right = bundle.get("delta_right", PWL_CONFIG["delta_right"])

        rebuild_overrides[key] = {
            "v_ref": fallback_speed,
            "delta_left": expand_factor * old_delta_left,
            "delta_right": expand_factor * old_delta_right,
            "window_mode": "rebuild_from_fallback",
            "rebuild_round": 2,
        }

    if rebuild_overrides:
        print(f"[Rebuild Override (from fallback)] {len(rebuild_overrides)} buses "
              f"with v_ref={fallback_speed:.1f}, expand_factor={expand_factor:.1f}")

    return rebuild_overrides


def _record_solve_round(solve_history, round_id, solve_mode, GurobiModel,
                        objective_value, pwl_solve_info, signal_pwl_bundle_d,
                        solve_time_sec, simulation_time, control_cycle_id):
    """
    向 solve_history 列表追加一条 round 记录。

    参数:
        solve_history      : list, 求解历史列表（会被原地修改）
        round_id           : int, 当前 round 编号 (0/1/2/3)
        solve_mode         : str, 求解模式
        GurobiModel        : gurobipy.Model, 已求解的模型
        objective_value    : float or None, 目标函数值
        pwl_solve_info     : dict or None, PWL 求解信息
        signal_pwl_bundle_d: dict or None, 本轮使用的 PWL bundle
        solve_time_sec     : float, 本轮求解用时 (s)
        simulation_time    : float, 触发仿真时刻
        control_cycle_id   : int, 控制周期 ID
    """
    # 获取模型状态
    model_status = GurobiModel.status if GurobiModel is not None else -1
    status_map = {
        1: "LOADED", 2: "OPTIMAL", 3: "INFEASIBLE", 4: "INF_OR_UNBD",
        5: "UNBOUNDED", 6: "CUTOFF", 7: "ITERATION_LIMIT", 8: "NODE_LIMIT",
        9: "TIME_LIMIT", 10: "SOLUTION_LIMIT", 11: "INTERRUPTED",
        12: "NUMERIC", 13: "SUBOPTIMAL",
    }
    model_status_str = status_map.get(model_status, f"UNKNOWN_{model_status}")

    # 判断是否使用了 PWL
    used_pwl = (signal_pwl_bundle_d is not None
                and solve_mode != "avg_delay_only")
    fallback_to_avg_delay = (solve_mode == "avg_delay_only")

    # 构建公交级信息
    bus_info = {}
    if pwl_solve_info is not None:
        for key, info in pwl_solve_info.items():
            # --- 基础字段（来自 pwl_solve_info，即 _extract_pwl_solve_info 的输出）---
            bus_info_entry = {
                "case_type": info.get("case_type", ""),
                "used_pwl": info.get("used_pwl", False),
                "window_mode": info.get("window_mode", ""),
                "rebuild_round": info.get("rebuild_round", 0),
                "v_ref": info.get("v_ref", 0.0),
                "v_ref_source": info.get("v_ref_source", ""),
                "ref_target_edge_id": info.get("ref_target_edge_id", ""),
                "delta_left": info.get("delta_left", 0.0),
                "delta_right": info.get("delta_right", 0.0),
                "x_lb": info.get("x_lb", 0.0),
                "x_ub": info.get("x_ub", 0.0),
                "x_phys_lb": info.get("x_phys_lb", 0.0),
                "x_phys_ub": info.get("x_phys_ub", 0.0),
                "t_arr_sig_sol": info.get("t_arr_sig_sol"),
                "t_cross_sig_sol": info.get("t_cross_sig_sol"),
                "hit_lower_boundary": info.get("hit_lower_boundary", False),
                "hit_upper_boundary": info.get("hit_upper_boundary", False),
                "rebuild_suggested": info.get("rebuild_suggested", False),
                "fallback_reason": None,
                "avg_delay_total": None,
            }

            # --- [V2 新增] 从 signal_pwl_bundle_d 中读取 stochastic meta ---
            # bundle 中的 pwl_generator_meta 字段由 builder 在组装 bundle 时写入，
            # 包含 generator_type, flow_source, capacity_dist_type 等追踪信息。
            # bundle 中的 used_window_type 字段标记实际使用了 local 还是 physical 窗口。
            if signal_pwl_bundle_d is not None and key in signal_pwl_bundle_d:
                bundle = signal_pwl_bundle_d[key]
                gen_meta = bundle.get("pwl_generator_meta") or {}
                bus_info_entry["generator_type"] = gen_meta.get("generator_type", "")
                bus_info_entry["queue_source"] = gen_meta.get("queue_source", "")
                bus_info_entry["initial_queue_m"] = gen_meta.get("initial_queue_m")
                bus_info_entry["n_flow_samples"] = gen_meta.get("n_flow_samples")
                bus_info_entry["n_capacity_samples"] = gen_meta.get("n_capacity_samples")
                bus_info_entry["flow_source"] = gen_meta.get("flow_source", "")
                bus_info_entry["target_lane_id"] = gen_meta.get("target_lane_id", "")
                bus_info_entry["flow_fallback_used"] = gen_meta.get("flow_fallback_used", False)
                bus_info_entry["capacity_dist_type"] = gen_meta.get("capacity_dist_type", "")
                bus_info_entry["capacity_params_used"] = str(gen_meta.get("capacity_params_used", ""))
                bus_info_entry["used_window_type"] = bundle.get("used_window_type", "")

            bus_info[key] = bus_info_entry

    round_record = {
        "control_cycle_id": control_cycle_id,
        "simulation_time": simulation_time,
        "solve_round_id": round_id,
        "solve_mode": solve_mode,
        "model_status": model_status,
        "model_status_str": model_status_str,
        "objective_value": objective_value,
        "used_pwl": used_pwl,
        "fallback_to_avg_delay": fallback_to_avg_delay,
        "solve_time_sec": solve_time_sec,
        "bus_info": bus_info,
    }

    solve_history.append(round_record)


# ============================================================================
# [BUGFIX Round-1] IIS 诊断辅助函数 —— 输出线性约束 + 一般约束 + 变量边界
# ============================================================================

def _print_iis_details(model):
    """
    在 computeIIS() 之后调用，输出完整的 IIS 成员信息。

    输出三类对象：
      1. 线性约束  （IISConstr）
      2. 一般约束  （IISGenConstr） —— 包括 ABS、PWL 等 general constraints
      3. 变量边界  （IISLB / IISUB）

    当前轮次不做"按公交归因"或"自动打分"，只把可见信息补全，
    为后续定位 infeasible 来源打基础。
    """
    separator = "-" * 70

    # ---- 1. 线性约束 ----
    iis_constrs = []
    for c in model.getConstrs():
        try:
            if c.IISConstr:
                iis_constrs.append(c.ConstrName)
        except Exception:
            pass

    print(f"\n{separator}")
    print(f"[IIS Diagnostics] 线性约束 (IISConstr): {len(iis_constrs)} 条")
    print(separator)
    for name in iis_constrs:
        print(f"  [LinConstr] {name}")

    # ---- 2. 一般约束（General Constraints） ----
    iis_gen_constrs = []
    try:
        for gc in model.getGenConstrs():
            try:
                if gc.IISGenConstr:
                    iis_gen_constrs.append(gc.GenConstrName)
            except Exception:
                pass
    except Exception:
        pass  # 模型可能没有 general constraints

    print(f"\n{separator}")
    print(f"[IIS Diagnostics] 一般约束 (IISGenConstr): {len(iis_gen_constrs)} 条")
    print(separator)
    for name in iis_gen_constrs:
        print(f"  [GenConstr] {name}")

    # ---- 3. 变量边界 ----
    iis_lb_vars = []
    iis_ub_vars = []
    for v in model.getVars():
        try:
            if v.IISLB:
                iis_lb_vars.append((v.VarName, v.LB))
        except Exception:
            pass
        try:
            if v.IISUB:
                iis_ub_vars.append((v.VarName, v.UB))
        except Exception:
            pass

    print(f"\n{separator}")
    print(f"[IIS Diagnostics] 变量下界 (IISLB): {len(iis_lb_vars)} 个")
    print(separator)
    for var_name, lb in iis_lb_vars:
        print(f"  [VarLB] {var_name}  LB={lb}")

    print(f"\n{separator}")
    print(f"[IIS Diagnostics] 变量上界 (IISUB): {len(iis_ub_vars)} 个")
    print(separator)
    for var_name, ub in iis_ub_vars:
        print(f"  [VarUB] {var_name}  UB={ub}")

    total_iis = len(iis_constrs) + len(iis_gen_constrs) + len(iis_lb_vars) + len(iis_ub_vars)
    print(f"\n{separator}")
    print(f"[IIS Diagnostics] IIS 成员总计: {total_iis} "
          f"(LinConstr={len(iis_constrs)}, GenConstr={len(iis_gen_constrs)}, "
          f"VarLB={len(iis_lb_vars)}, VarUB={len(iis_ub_vars)})")
    print(f"{separator}\n")


# ============================================================================
# [NEW] 单次"建模 + 求解"封装函数
# ============================================================================

def _build_and_solve_gurobi_model(
    env, bus_obj_l, stop_obj_l, bus_obj_id_l, stop_obj_id_l,
    trigger_time, station_dict, lb_speed, up_speed, station_interval_dis_dict,
    scaled_bus_arrstation_od_otd_dict, scaled_line_station_od_otd_dict,
    static_busCap_alight_board_l, od_otd_arr_rate_dict,
    sorted_busline_edge_d, line_obj_dic, interStop_trafficFlow_d,
    signal_pwl_bundle_d, model_aux_var_d,
    solve_mode="pwl_local_initial",
    time_limit=180,
):
    """
    单次完整建模 + 求解。

    这是对原 Gurobimodel() 中从 "GurobiModel = gurobipy.Model(env=env)"
    到 "GurobiModel.optimize(my_callback)" 以及解提取逻辑的封装。
    不改变建模逻辑本身，仅使其可被多轮调用。

    参数:
        env                : gurobipy.Env, Gurobi 环境
        bus_obj_l          : list, 公交车对象列表
        stop_obj_l         : list, 公交站对象列表
        bus_obj_id_l       : list, 公交车 ID 列表
        stop_obj_id_l      : list, 公交站 ID 列表
        trigger_time       : float, 触发时刻
        station_dict       : dict, 各线路公交站顺序
        station_interval_dis_dict : dict, 站间距离
        scaled_bus_arrstation_od_otd_dict : dict
        scaled_line_station_od_otd_dict   : dict
        static_busCap_alight_board_l : list, [BusCap, AveAlightingTime, AveBoardingTime]
        od_otd_arr_rate_dict : dict
        sorted_busline_edge_d : dict
        line_obj_dic       : dict
        interStop_trafficFlow_d : dict
        signal_pwl_bundle_d : dict or None, PWL 数据包（None 表示不使用 PWL）
        model_aux_var_d    : dict, PWL 辅助变量（会被原地写入新变量）
        solve_mode         : str, 求解模式
                             "pwl_local_initial" / "pwl_rebuild_from_solution" /
                             "pwl_physical_window" / "avg_delay_only"
                             注："pwl_rebuild_from_fallback" 为兼容保留值，不再由主流程使用
        time_limit         : int, Gurobi 求解时间限制 (s)

    返回:
        tuple: (GurobiModel, is_feasible, bus_speed_curve, controlled_bus_trajectory_data,
                objective_value, cur_bus_info, cur_stop_info, decision_variable_inv_d)
    """

    BusCap = static_busCap_alight_board_l[0]
    AveAlightingTime = static_busCap_alight_board_l[1]
    AveBoardingTime = static_busCap_alight_board_l[2]
    AH = 200
    neighborBusDepartInterval = 1
    MaxBusPassenger = BusCap
    MaxStopPassenger = 100
    # lb_speed = 1.00
    # up_speed = 13.89

    # ========== 创建 Gurobi 模型 ==========
    GurobiModel = gurobipy.Model(env=env)

    # ========== 决策变量 ==========
    decision_variable_inv_d = {}
    for bus_obj in bus_obj_l:
        for stop in bus_obj.unserved_stop_l:
            decision_variable_inv_d[bus_obj.bus_id_s, stop] = GurobiModel.addVar(
                vtype=gurobipy.GRB.CONTINUOUS,
                lb=1 / up_speed, ub=1 / lb_speed,
                name=f"decision_variable_inv_{bus_obj.bus_id_s}+{stop}")

    # ========== 公交车相关辅助变量 ==========
    depart_time_variable_d = {}
    arrive_time_variable_d = {}
    service_time_variable_d = {}
    bus_arrstation_passenger_number_variable_d = {}
    bus_arrstation_OD_OTD_number_variable_d = {}

    for bus_obj in bus_obj_l:
        unserved_stop_list = bus_obj.unserved_stop_l
        for unserved_stop in unserved_stop_list:
            depart_time_variable_d[bus_obj.bus_id_s, unserved_stop] = GurobiModel.addVar(
                vtype=gurobipy.GRB.CONTINUOUS, lb=0,
                name=f"depart_time_{bus_obj.bus_id_s}+{unserved_stop}")
            arrive_time_variable_d[bus_obj.bus_id_s, unserved_stop] = GurobiModel.addVar(
                vtype=gurobipy.GRB.CONTINUOUS, lb=0,
                name=f"arrive_time_{bus_obj.bus_id_s}+{unserved_stop}")
            service_time_variable_d[bus_obj.bus_id_s, unserved_stop] = GurobiModel.addVar(
                vtype=gurobipy.GRB.CONTINUOUS, lb=0,
                name=f"service_time_{bus_obj.bus_id_s}+{unserved_stop}")
            bus_arrstation_passenger_number_variable_d[bus_obj.bus_id_s, unserved_stop] = GurobiModel.addVar(
                vtype=gurobipy.GRB.CONTINUOUS, lb=0, ub=MaxBusPassenger,
                name=f"bus_arrstation_passenger_number_{bus_obj.bus_id_s}+{unserved_stop}")

            for od_otd in scaled_bus_arrstation_od_otd_dict[bus_obj.belong_line_id_s][unserved_stop]:
                bus_arrstation_OD_OTD_number_variable_d[bus_obj.bus_id_s, unserved_stop, od_otd] = \
                    GurobiModel.addVar(
                        vtype=gurobipy.GRB.CONTINUOUS, lb=0, ub=MaxBusPassenger,
                        name=f"bus_arrstation_OD_OTD_number_{bus_obj.bus_id_s}+{unserved_stop}+{od_otd}")

    # ========== 公交站相关辅助变量 ==========
    alighting_passenger_number_variable_d = {}
    boarding_passenger_number_variable_d = {}
    stop_busdepart_passenger_number_variable_d = {}
    want_boarding_passenger_number_variable_d = {}
    boarding_multiply_want_boarding_passenger_number_inv_variable_d = {}

    stop_lastbus_arrival_time_variable_d = {}
    stop_lastbus_depart_time_variable_d = {}
    stop_lastbusdepart_passenger_number_variable_d = {}

    stop_busarrive_OD_OTD_number_variable_d = {}
    stop_busdepart_OD_OTD_number_variable_d = {}
    alighting_OD_OTD_number_variable_d = {}
    boarding_OD_OTD_number_variable_d = {}
    want_boarding_OD_OTD_number_variable_d = {}
    stop_lastbusdepart_OD_OTD_number_variable_d = {}

    for stop_obj in stop_obj_l:
        unserved_bus_l = stop_obj.unserved_bus_l
        for unserved_bus in unserved_bus_l:
            stop_lastbus_arrival_time_variable_d[unserved_bus, stop_obj.stop_id_s] = GurobiModel.addVar(
                vtype=gurobipy.GRB.CONTINUOUS, lb=0,
                name=f"stop_lastbus_arrival_time_{unserved_bus}+{stop_obj.stop_id_s}")
            stop_lastbus_depart_time_variable_d[unserved_bus, stop_obj.stop_id_s] = GurobiModel.addVar(
                vtype=gurobipy.GRB.CONTINUOUS, lb=0,
                name=f"stop_lastbus_depart_time_{unserved_bus}+{stop_obj.stop_id_s}")
            stop_lastbusdepart_passenger_number_variable_d[unserved_bus, stop_obj.stop_id_s] = GurobiModel.addVar(
                vtype=gurobipy.GRB.CONTINUOUS, lb=0, ub=MaxStopPassenger,
                name=f"stop_lastbusdepart_passenger_number_{unserved_bus}+{stop_obj.stop_id_s}")

            alighting_passenger_number_variable_d[unserved_bus, stop_obj.stop_id_s] = GurobiModel.addVar(
                vtype=gurobipy.GRB.CONTINUOUS, lb=0, ub=MaxBusPassenger,
                name=f"alighting_passenger_number_{unserved_bus}+{stop_obj.stop_id_s}")
            boarding_passenger_number_variable_d[unserved_bus, stop_obj.stop_id_s] = GurobiModel.addVar(
                vtype=gurobipy.GRB.CONTINUOUS, lb=0, ub=MaxBusPassenger,
                name=f"boarding_passenger_number_{unserved_bus}+{stop_obj.stop_id_s}")
            stop_busdepart_passenger_number_variable_d[unserved_bus, stop_obj.stop_id_s] = GurobiModel.addVar(
                vtype=gurobipy.GRB.CONTINUOUS, lb=0, ub=MaxStopPassenger,
                name=f"stop_busdepart_passenger_number_{unserved_bus}+{stop_obj.stop_id_s}")
            want_boarding_passenger_number_variable_d[unserved_bus, stop_obj.stop_id_s] = GurobiModel.addVar(
                vtype=gurobipy.GRB.CONTINUOUS, lb=0, ub=MaxStopPassenger,
                name=f"want_boarding_passenger_number_{unserved_bus}+{stop_obj.stop_id_s}")
            boarding_multiply_want_boarding_passenger_number_inv_variable_d[unserved_bus, stop_obj.stop_id_s] = \
                GurobiModel.addVar(vtype=gurobipy.GRB.CONTINUOUS, lb=0, ub=1,
                                   name=f"boarding_multiply_want_boarding_passenger_number_inv_{unserved_bus}+{stop_obj.stop_id_s}")

            unserved_bus_belong_lines = unserved_bus.split("_")[0]
            if stop_obj.stop_id_s in scaled_line_station_od_otd_dict[unserved_bus_belong_lines]:
                for od_otd in scaled_line_station_od_otd_dict[unserved_bus_belong_lines][stop_obj.stop_id_s]:
                    stop_busarrive_OD_OTD_number_variable_d[unserved_bus, stop_obj.stop_id_s, od_otd] = \
                        GurobiModel.addVar(
                            vtype=gurobipy.GRB.CONTINUOUS, lb=0, ub=MaxStopPassenger,
                            name=f"stop_busarrive_OD_OTD_number_{unserved_bus}+{stop_obj.stop_id_s}+{od_otd}")
                    stop_busdepart_OD_OTD_number_variable_d[unserved_bus, stop_obj.stop_id_s, od_otd] = \
                        GurobiModel.addVar(
                            vtype=gurobipy.GRB.CONTINUOUS, lb=0, ub=MaxStopPassenger,
                            name=f"stop_busdepart_OD_OTD_number_{unserved_bus}+{stop_obj.stop_id_s}+{od_otd}")
                    stop_lastbusdepart_OD_OTD_number_variable_d[unserved_bus, stop_obj.stop_id_s, od_otd] = \
                        GurobiModel.addVar(
                            vtype=gurobipy.GRB.CONTINUOUS, lb=0, ub=MaxStopPassenger,
                            name=f"stop_lastbusdepart_OD_OTD_number_{unserved_bus}+{stop_obj.stop_id_s}+{od_otd}")

            stop_obj_index = station_dict[unserved_bus_belong_lines].index(stop_obj.stop_id_s)
            if stop_obj_index < len(station_dict[unserved_bus_belong_lines]) - 1:
                next_stop = station_dict[unserved_bus_belong_lines][stop_obj_index + 1]
                boarding_od_otd_list = list(
                    set(scaled_bus_arrstation_od_otd_dict[unserved_bus_belong_lines][next_stop]) -
                    set(scaled_bus_arrstation_od_otd_dict[unserved_bus_belong_lines][stop_obj.stop_id_s]))
                alighting_od_otd_list = list(
                    set(scaled_bus_arrstation_od_otd_dict[unserved_bus_belong_lines][stop_obj.stop_id_s]) -
                    set(scaled_bus_arrstation_od_otd_dict[unserved_bus_belong_lines][next_stop]))
            else:
                boarding_od_otd_list = {}
                alighting_od_otd_list = scaled_bus_arrstation_od_otd_dict[unserved_bus_belong_lines][stop_obj.stop_id_s]

            for od_otd in boarding_od_otd_list:
                boarding_OD_OTD_number_variable_d[unserved_bus, stop_obj.stop_id_s, od_otd] = GurobiModel.addVar(
                    vtype=gurobipy.GRB.CONTINUOUS, lb=0, ub=MaxBusPassenger,
                    name=f"boarding_OD_OTD_number_{unserved_bus}+{stop_obj.stop_id_s}+{od_otd}")
                want_boarding_OD_OTD_number_variable_d[unserved_bus, stop_obj.stop_id_s, od_otd] = GurobiModel.addVar(
                    vtype=gurobipy.GRB.CONTINUOUS, lb=0, ub=MaxStopPassenger,
                    name=f"want_boarding_OD_OTD_number_{unserved_bus}+{stop_obj.stop_id_s}+{od_otd}")
            for od_otd in alighting_od_otd_list:
                alighting_OD_OTD_number_variable_d[unserved_bus, stop_obj.stop_id_s, od_otd] = GurobiModel.addVar(
                    vtype=gurobipy.GRB.CONTINUOUS, lb=0, ub=MaxBusPassenger,
                    name=f"alighting_OD_OTD_number_{unserved_bus}+{stop_obj.stop_id_s}+{od_otd}")

        # 触发时刻各公交车刚经过的非终点站更新的乘客信息
        if stop_obj.just_leave_data_l[-1] and stop_obj.stop_id_s != station_dict[stop_obj.just_leave_data_l[1]][-1]:
            stop_busdepart_passenger_number_variable_d[stop_obj.just_leave_data_l[0], stop_obj.stop_id_s] = \
                GurobiModel.addVar(
                    vtype=gurobipy.GRB.CONTINUOUS, lb=0, ub=MaxStopPassenger,
                    name=f"stop_busdepart_passenger_number_{stop_obj.just_leave_data_l[0]}+{stop_obj.stop_id_s}")
            for od_otd in scaled_line_station_od_otd_dict[stop_obj.just_leave_data_l[1]][stop_obj.stop_id_s]:
                stop_busdepart_OD_OTD_number_variable_d[stop_obj.just_leave_data_l[0], stop_obj.stop_id_s, od_otd] = \
                    GurobiModel.addVar(
                        vtype=gurobipy.GRB.CONTINUOUS, lb=0, ub=MaxStopPassenger,
                        name=f"stop_busdepart_OD_OTD_number_{stop_obj.just_leave_data_l[0]}+{stop_obj.stop_id_s}+{od_otd}")

        depart_time_variable_d[stop_obj.just_leave_data_l[0], stop_obj.stop_id_s] = GurobiModel.addVar(
            vtype=gurobipy.GRB.CONTINUOUS, lb=0,
            name=f"depart_time_{stop_obj.just_leave_data_l[0]}+{stop_obj.stop_id_s}")

        arrive_time_variable_d[stop_obj.just_leave_data_l[0], stop_obj.stop_id_s] = GurobiModel.addVar(
            vtype=gurobipy.GRB.CONTINUOUS, lb=0,
            name=f"arrival_time_{stop_obj.just_leave_data_l[0]}+{stop_obj.stop_id_s}")

    # ========== 初始化触发时刻状态信息 ==========
    h_subfunction_for_gurobi.initial_bus_arrstation_OD_OTD_number_variable(
        bus_obj_l, stop_obj_l, bus_arrstation_OD_OTD_number_variable_d,
        bus_arrstation_passenger_number_variable_d,
        stop_busdepart_OD_OTD_number_variable_d,
        stop_busdepart_passenger_number_variable_d, depart_time_variable_d,
        arrive_time_variable_d, scaled_bus_arrstation_od_otd_dict,
        scaled_line_station_od_otd_dict,
        GurobiModel)

    # ========== 下一站到站时间约束（含 PWL / avg_delay_only 分支）==========
    # 当 solve_mode == "avg_delay_only" 时，signal_pwl_bundle_d 传 None 以跳过 PWL
    effective_pwl_bundle = signal_pwl_bundle_d if solve_mode != "avg_delay_only" else None
    h_subfunction_for_gurobi.define_next_stop_arrival_time_con(
        bus_obj_l, trigger_time, arrive_time_variable_d,
        decision_variable_inv_d, GurobiModel,
        signal_pwl_bundle_d=effective_pwl_bundle,
        model_aux_var_d=model_aux_var_d,
        solve_mode=solve_mode,
        line_obj_dic=line_obj_dic)

    # ========== 除下一站外的到站时间约束 ==========
    h_subfunction_for_gurobi.define_unserved_without_next_stop_arrival_time_with_tl_delay_con(
        bus_obj_l, arrive_time_variable_d, depart_time_variable_d, station_dict,
        decision_variable_inv_d, station_interval_dis_dict,
        sorted_busline_edge_d, line_obj_dic, GurobiModel)

    # ========== 离站时间约束 ==========
    h_subfunction_for_gurobi.define_unserved_stop_depart_time_con(
        arrive_time_variable_d, depart_time_variable_d, service_time_variable_d,
        GurobiModel)

    # ========== 服务时间约束 ==========
    h_subfunction_for_gurobi.define_unserved_stop_serve_time_con(
        AveAlightingTime, AveBoardingTime, service_time_variable_d, GurobiModel,
        boarding_passenger_number_variable_d, alighting_passenger_number_variable_d)

    # ========== 下车乘客约束 ==========
    h_subfunction_for_gurobi.define_alighting_passenger_number_con(
        alighting_passenger_number_variable_d, alighting_OD_OTD_number_variable_d,
        bus_arrstation_OD_OTD_number_variable_d, scaled_bus_arrstation_od_otd_dict,
        bus_obj_l, station_dict, GurobiModel)

    # ========== 上车乘客约束 ==========
    h_subfunction_for_gurobi.define_boarding_passenger_number_con(
        want_boarding_passenger_number_variable_d, boarding_passenger_number_variable_d,
        stop_busarrive_OD_OTD_number_variable_d, bus_arrstation_passenger_number_variable_d,
        alighting_passenger_number_variable_d, scaled_bus_arrstation_od_otd_dict,
        GurobiModel, bus_obj_l, station_dict, BusCap)

    # ========== 站内乘客约束 ==========
    h_subfunction_for_gurobi.define_stop_busarr_OD_OTD_passenger_number_con(
        stop_busarrive_OD_OTD_number_variable_d,
        stop_lastbusdepart_OD_OTD_number_variable_d,
        stop_lastbus_depart_time_variable_d, depart_time_variable_d,
        od_otd_arr_rate_dict, scaled_line_station_od_otd_dict, stop_obj_l,
        GurobiModel)

    # ========== 到站顺序约束 ==========
    bus_sequence_on_stops_d = h_subfunction_for_gurobi.get_bus_sequence_on_stops(
        stop_obj_l, bus_obj_l, station_interval_dis_dict, stop_obj_id_l,
        arrive_time_variable_d, GurobiModel)
    h_subfunction_for_gurobi.simplified_define_stop_lastbus_depart_time_OD_OTD_number_con(
        depart_time_variable_d, stop_lastbus_depart_time_variable_d,
        stop_busdepart_OD_OTD_number_variable_d,
        stop_lastbusdepart_OD_OTD_number_variable_d,
        stop_lastbusdepart_passenger_number_variable_d,
        scaled_line_station_od_otd_dict, stop_obj_l,
        GurobiModel, bus_sequence_on_stops_d, neighborBusDepartInterval)
    h_subfunction_for_gurobi.define_arrival_minheadway_with_busseq_con(
        stop_obj_l, arrive_time_variable_d, stop_lastbus_arrival_time_variable_d,
        AH, GurobiModel, bus_sequence_on_stops_d, scaled_line_station_od_otd_dict)

    # ========== 最大站间运行时间约束 ==========
    MaxIntervalRuningTime = 1000
    h_subfunction_for_gurobi.define_bus_max_interval_running_time_con(
        bus_obj_l, arrive_time_variable_d, MaxIntervalRuningTime, GurobiModel)

    # ========== 中间变量约束 ==========
    h_subfunction_for_gurobi.define_boarding_multiply_want_boarding_passenger_number_con(
        want_boarding_passenger_number_variable_d,
        boarding_passenger_number_variable_d,
        boarding_multiply_want_boarding_passenger_number_inv_variable_d,
        GurobiModel)

    # ========== 车上乘客约束 ==========
    h_subfunction_for_gurobi.define_bus_arrstation_passenger_number_con(
        bus_arrstation_OD_OTD_number_variable_d,
        boarding_multiply_want_boarding_passenger_number_inv_variable_d,
        stop_busarrive_OD_OTD_number_variable_d,
        bus_arrstation_passenger_number_variable_d, scaled_bus_arrstation_od_otd_dict,
        station_dict, bus_obj_l, GurobiModel)

    # ========== 站内离站乘客约束 ==========
    h_subfunction_for_gurobi.define_stop_busdepart_OD_OTD_number_con(
        stop_busdepart_OD_OTD_number_variable_d,
        stop_busarrive_OD_OTD_number_variable_d,
        boarding_multiply_want_boarding_passenger_number_inv_variable_d, station_dict,
        bus_obj_l, scaled_bus_arrstation_od_otd_dict,
        bus_arrstation_OD_OTD_number_variable_d, scaled_line_station_od_otd_dict,
        GurobiModel)

    # ========== 决策变量交通流约束 ==========
    h_subfunction_for_gurobi.decision_variable_trafficFlow_con(
        interStop_trafficFlow_d, decision_variable_inv_d, bus_obj_l, GurobiModel)

    # ========== 更新变量环境 ==========
    GurobiModel.update()

    # ========== 目标函数 ==========
    ObjectiveValue = h_subfunction_for_gurobi.get_objective_value(
        bus_obj_l, arrive_time_variable_d, station_dict, GurobiModel)
    GurobiModel.setObjective(ObjectiveValue, gurobipy.GRB.MINIMIZE)

    # ========== 求解参数设置 ==========
    GurobiModel.setParam("Method", 2)
    GurobiModel.setParam("Heuristics", 0.2)
    GurobiModel.setParam("ImproveStartTime", 50)
    GurobiModel.setParam('NonConvex', 2)
    GurobiModel.setParam("NumericFocus", 3)
    GurobiModel.setParam("TimeLimit", time_limit)
    GurobiModel.setParam("PoolSearchMode", 2)
    GurobiModel.setParam("PoolSolutions", 10)
    GurobiModel.setParam("PumpPasses", 200)
    GurobiModel.setParam("MIPFocus", 1)
    GurobiModel.setParam('FeasibilityTol', 1e-4)
    GurobiModel.setParam('OptimalityTol', 1e-4)
    GurobiModel.setParam('MIPGap', 0.01)
    GurobiModel.setParam('OutputFlag', 1)
    GurobiModel.setParam('LogFile', 'gurobi_log.txt')
    GurobiModel.setParam('Presolve', 2)

    # ========== 求解 ==========
    GurobiModel.optimize(my_callback)

    # ========== 检查可行性 ==========
    IIS_flag = 0
    if GurobiModel.status == gurobipy.GRB.INFEASIBLE:
        print(f"模型不可行 (solve_mode={solve_mode})，开始计算 IIS...")
        IIS_flag = 1
        GurobiModel.computeIIS()
        _print_iis_details(GurobiModel)

    is_feasible = (IIS_flag == 0 and GurobiModel.SolCount > 0)

    if is_feasible:
        # 提取可行解
        controlled_bus_trajectory_data = h_subfunction_for_gurobi.plot_multiline_bus_trajectory_after_trigger(
            arrive_time_variable_d, depart_time_variable_d,
            station_dict, bus_obj_l, bus_obj_id_l, station_interval_dis_dict)
        bus_speed_curve = h_subfunction_for_gurobi.get_bus_speed_curve(
            decision_variable_inv_d, line_obj_dic)
        objective_value = GurobiModel.objVal
        cur_bus_info, cur_stop_info = h_subfunction_for_gurobi.get_bus_current_info(
            bus_obj_l, stop_obj_l)
    else:
        bus_speed_curve = 0
        controlled_bus_trajectory_data = 0
        objective_value = 0
        cur_bus_info = {}
        cur_stop_info = {}

    return (GurobiModel, is_feasible, bus_speed_curve, controlled_bus_trajectory_data,
            objective_value, cur_bus_info, cur_stop_info, decision_variable_inv_d)


# ============================================================================
# 主函数：多轮 Solve Round 控制流
# ============================================================================

def Gurobimodel(bus_obj_dic, stop_obj_dic, trigger_time, station_dict, station_interval_dis_dict,
                scaled_bus_arrstation_od_otd_dict,
                scaled_line_station_od_otd_dict, static_busCap_alight_board_l, od_otd_arr_rate_dict,
                sorted_busline_edge_d,
                line_obj_dic, interStop_trafficFlow_d,
                # ===== [原有] PWL through-time 嵌入相关参数 =====
                signal_obj_dic=None,
                busline_stop_edge_d=None,
                enable_signal_pwl=True,
                signal_pwl_debug=False,
                pwl_generator_func=None,
                pwl_config=None,
                # ===== [NEW] 局部窗口改造新增参数 =====
                last_control_result=None):

    # ================================================================
    # 使用 PWL_CONFIG 作为默认配置，允许外部 pwl_config 覆盖
    # ================================================================
    if pwl_config is None:
        pwl_config = copy.deepcopy(PWL_CONFIG)
    else:
        # 将 PWL_CONFIG 中的默认值补充到 pwl_config 中（不覆盖已有值）
        merged_config = copy.deepcopy(PWL_CONFIG)
        merged_config.update(pwl_config)
        pwl_config = merged_config

    # ================================================================
    # 准备阶段（与原版一致）
    # ================================================================
    env = gurobipy.Env(params={"OutputFlag": 0})

    first_stop_num = 100
    simplified_bus_obj_dic, simplified_stop_obj_dic = Simplified_bus_and_stop_dict(
        bus_obj_dic, stop_obj_dic, first_stop_num, scaled_line_station_od_otd_dict)

    bus_obj_l, stop_obj_l, bus_obj_id_l, stop_obj_id_l = get_bus_stop_obj_l(
        simplified_bus_obj_dic, simplified_stop_obj_dic)

    # 控制周期相关
    control_cycle_id = _get_next_control_cycle_id()
    solve_history = []

    # ================================================================
    # 速度边界参数 — 两套速度边界彻底分离
    # ================================================================
    # (1) decision_lb_speed / decision_up_speed：
    #     仅用于 _build_and_solve_gurobi_model() 中的决策变量 lb/ub。
    #     decision_lb_speed 极小是因为晚点时后车可能距站极近，
    #     需要允许极低速度以避免模型不可行。
    decision_lb_speed = 1e-10
    decision_up_speed = 13.89

    # (2) pwl_lb_speed / pwl_up_speed：
    #     仅用于 build_signal_pwl_bundles() 中的 arrival window、
    #     v_ref clamp 和 physical window 构建。
    #     pwl_lb_speed 设为 1.0 m/s，保证 PWL 窗口宽度合理
    #     （若使用 1e-10 会导致 x_phys_ub 极大，窗口退化）。
    pwl_lb_speed = 1.0
    pwl_up_speed = decision_up_speed

    # --- 基本参数校验 ---
    assert decision_lb_speed > 0, \
        f"[Gurobimodel] decision_lb_speed must be > 0, got {decision_lb_speed}"
    assert decision_up_speed > decision_lb_speed, \
        f"[Gurobimodel] decision_up_speed({decision_up_speed}) must be > decision_lb_speed({decision_lb_speed})"
    assert pwl_lb_speed > 0, \
        f"[Gurobimodel] pwl_lb_speed must be > 0, got {pwl_lb_speed}"
    assert pwl_up_speed > pwl_lb_speed, \
        f"[Gurobimodel] pwl_up_speed({pwl_up_speed}) must be > pwl_lb_speed({pwl_lb_speed})"

    print(f"[Gurobimodel] 速度边界 — "
          f"decision=[{decision_lb_speed}, {decision_up_speed}], "
          f"pwl=[{pwl_lb_speed}, {pwl_up_speed}]")

    # ================================================================
    # 判断是否启用 PWL（与原版逻辑一致）
    # ================================================================
    can_build_pwl = (enable_signal_pwl
                     and signal_obj_dic is not None
                     and busline_stop_edge_d is not None)

    if enable_signal_pwl and not can_build_pwl:
        if signal_obj_dic is None:
            print("[Gurobimodel INFO] signal_obj_dic not provided, PWL disabled")
        if busline_stop_edge_d is None:
            print("[Gurobimodel INFO] busline_stop_edge_d not provided, PWL disabled")

    # ================================================================
    # Round 0: PWL local initial
    # ================================================================
    print(f"\n{'='*60}")
    print(f"[Solve Round 0] pwl_local_initial, t={trigger_time:.1f}")
    print(f"{'='*60}")

    signal_pwl_bundle_d_r0 = None
    model_aux_var_d_r0 = {}

    if can_build_pwl:
        try:
            signal_pwl_bundle_d_r0 = l_realtime_signal_pwl_builder.build_signal_pwl_bundles(
                current_time=trigger_time,
                bus_obj_l=bus_obj_l,
                line_obj_dic=line_obj_dic,
                signal_obj_dic=signal_obj_dic,
                sorted_busline_edge_d=sorted_busline_edge_d,
                busline_stop_edge_d=busline_stop_edge_d,
                station_dict=station_dict,
                lb_speed=pwl_lb_speed,       # [分离] PWL 专用速度下限
                up_speed=pwl_up_speed,       # [分离] PWL 专用速度上限
                pwl_generator_func=pwl_generator_func,
                pwl_config=pwl_config,
                last_control_result=last_control_result,
                rebuild_overrides=None,
                existing_bundles=None,
            )
        except Exception as e:
            print(f"[Gurobimodel WARNING] Round 0 PWL bundle construction failed: {e}")
            print(f"[Gurobimodel WARNING] Falling back to no PWL for Round 0")
            signal_pwl_bundle_d_r0 = None

    # Round 0 建模 + 求解
    t0_start = time_module.perf_counter()
    (model_r0, feasible_r0, speed_curve_r0, traj_data_r0, obj_r0,
     bus_info_r0, stop_info_r0, dec_var_inv_r0) = _build_and_solve_gurobi_model(
        env=env,
        bus_obj_l=bus_obj_l, stop_obj_l=stop_obj_l,
        bus_obj_id_l=bus_obj_id_l, stop_obj_id_l=stop_obj_id_l,
        trigger_time=trigger_time,
        station_dict=station_dict,
        lb_speed=decision_lb_speed,       # [分离] 决策变量专用速度下限
        up_speed=decision_up_speed,       # [分离] 决策变量专用速度上限
        station_interval_dis_dict=station_interval_dis_dict,
        scaled_bus_arrstation_od_otd_dict=scaled_bus_arrstation_od_otd_dict,
        scaled_line_station_od_otd_dict=scaled_line_station_od_otd_dict,
        static_busCap_alight_board_l=static_busCap_alight_board_l,
        od_otd_arr_rate_dict=od_otd_arr_rate_dict,
        sorted_busline_edge_d=sorted_busline_edge_d,
        line_obj_dic=line_obj_dic,
        interStop_trafficFlow_d=interStop_trafficFlow_d,
        signal_pwl_bundle_d=signal_pwl_bundle_d_r0,
        model_aux_var_d=model_aux_var_d_r0,
        solve_mode="pwl_local_initial",
        time_limit=120,
    )
    t0_elapsed = time_module.perf_counter() - t0_start

    # Round 0 结果分析
    pwl_info_r0 = None
    if feasible_r0 and signal_pwl_bundle_d_r0 is not None:
        pwl_info_r0 = _extract_pwl_solve_info(
            model_aux_var_d_r0, signal_pwl_bundle_d_r0, bus_obj_l, dec_var_inv_r0,
            boundary_hit_eps_abs=pwl_config.get("boundary_hit_eps_abs", 0.5),
            boundary_hit_eps_rel=pwl_config.get("boundary_hit_eps_rel", 0.02))

    _record_solve_round(
        solve_history, round_id=0, solve_mode="pwl_local_initial",
        GurobiModel=model_r0, objective_value=obj_r0 if feasible_r0 else None,
        pwl_solve_info=pwl_info_r0, signal_pwl_bundle_d=signal_pwl_bundle_d_r0,
        solve_time_sec=t0_elapsed, simulation_time=trigger_time,
        control_cycle_id=control_cycle_id)

    # ================================================================
    # Round 0 可行 → 检查边界
    # ================================================================
    if feasible_r0:
        hit_list_r0 = _check_boundary_hits(pwl_info_r0) if pwl_info_r0 else []

        if not hit_list_r0:
            # ✅ 无命中边界 → 最终解
            print(f"[Solve Round 0] ✅ 可行且无边界命中，直接返回 (obj={obj_r0:.2f})")
            return (speed_curve_r0, traj_data_r0, 0, obj_r0,
                    bus_info_r0, stop_info_r0, solve_history)

        # 有命中边界 → 进入 Round 1
        print(f"[Solve Round 0] ⚠ 可行但有 {len(hit_list_r0)} 辆公交命中边界，进入 Round 1")

        # ============================================================
        # Round 1: rebuild_from_solution
        # ============================================================
        print(f"\n{'='*60}")
        print(f"[Solve Round 1] pwl_rebuild_from_solution, t={trigger_time:.1f}")
        print(f"{'='*60}")

        # 为命中边界的公交计算重建参数
        overrides_r1 = _build_rebuild_overrides_from_solution(
            pwl_solve_info=pwl_info_r0,
            hit_list=hit_list_r0,
            signal_pwl_bundle_d=signal_pwl_bundle_d_r0,
            expand_factor=pwl_config.get("rebuild_expand_factor_hit_boundary", 1.5))

        # 使用重建参数构建新 bundle（仅重建命中边界的公交，其余复用 Round 0 的）
        signal_pwl_bundle_d_r1 = None
        model_aux_var_d_r1 = {}
        try:
            signal_pwl_bundle_d_r1 = l_realtime_signal_pwl_builder.build_signal_pwl_bundles(
                current_time=trigger_time,
                bus_obj_l=bus_obj_l,
                line_obj_dic=line_obj_dic,
                signal_obj_dic=signal_obj_dic,
                sorted_busline_edge_d=sorted_busline_edge_d,
                busline_stop_edge_d=busline_stop_edge_d,
                station_dict=station_dict,
                lb_speed=pwl_lb_speed,       # [分离] PWL 专用速度下限
                up_speed=pwl_up_speed,       # [分离] PWL 专用速度上限
                pwl_generator_func=pwl_generator_func,
                pwl_config=pwl_config,
                last_control_result=last_control_result,
                rebuild_overrides=overrides_r1,
                existing_bundles=signal_pwl_bundle_d_r0,
            )
        except Exception as e:
            print(f"[Gurobimodel WARNING] Round 1 PWL rebuild failed: {e}")
            signal_pwl_bundle_d_r1 = signal_pwl_bundle_d_r0  # 回退

        # Round 1 建模 + 求解（缩短时间限制以控制总用时）
        t1_start = time_module.perf_counter()
        (model_r1, feasible_r1, speed_curve_r1, traj_data_r1, obj_r1,
         bus_info_r1, stop_info_r1, dec_var_inv_r1) = _build_and_solve_gurobi_model(
            env=env,
            bus_obj_l=bus_obj_l, stop_obj_l=stop_obj_l,
            bus_obj_id_l=bus_obj_id_l, stop_obj_id_l=stop_obj_id_l,
            trigger_time=trigger_time,
            station_dict=station_dict,
            lb_speed=decision_lb_speed,       # [分离] 决策变量专用速度下限
            up_speed=decision_up_speed,       # [分离] 决策变量专用速度上限
            station_interval_dis_dict=station_interval_dis_dict,
            scaled_bus_arrstation_od_otd_dict=scaled_bus_arrstation_od_otd_dict,
            scaled_line_station_od_otd_dict=scaled_line_station_od_otd_dict,
            static_busCap_alight_board_l=static_busCap_alight_board_l,
            od_otd_arr_rate_dict=od_otd_arr_rate_dict,
            sorted_busline_edge_d=sorted_busline_edge_d,
            line_obj_dic=line_obj_dic,
            interStop_trafficFlow_d=interStop_trafficFlow_d,
            signal_pwl_bundle_d=signal_pwl_bundle_d_r1,
            model_aux_var_d=model_aux_var_d_r1,
            solve_mode="pwl_rebuild_from_solution",
            time_limit=120,  # Round 1 缩短时间限制
        )
        t1_elapsed = time_module.perf_counter() - t1_start

        # Round 1 结果分析
        pwl_info_r1 = None
        if feasible_r1 and signal_pwl_bundle_d_r1 is not None:
            pwl_info_r1 = _extract_pwl_solve_info(
                model_aux_var_d_r1, signal_pwl_bundle_d_r1, bus_obj_l, dec_var_inv_r1,
                boundary_hit_eps_abs=pwl_config.get("boundary_hit_eps_abs", 0.5),
                boundary_hit_eps_rel=pwl_config.get("boundary_hit_eps_rel", 0.02))

        _record_solve_round(
            solve_history, round_id=1, solve_mode="pwl_rebuild_from_solution",
            GurobiModel=model_r1, objective_value=obj_r1 if feasible_r1 else None,
            pwl_solve_info=pwl_info_r1, signal_pwl_bundle_d=signal_pwl_bundle_d_r1,
            solve_time_sec=t1_elapsed, simulation_time=trigger_time,
            control_cycle_id=control_cycle_id)

        if feasible_r1:
            # Round 1 可行 → 检查是否仍命中边界
            hit_list_r1 = _check_boundary_hits(pwl_info_r1) if pwl_info_r1 else []
            if hit_list_r1:
                print(f"[Solve Round 1] ⚠ WARNING: Round 1 仍有 {len(hit_list_r1)} 辆公交命中边界，"
                      f"接受当前解 (obj={obj_r1:.2f})")
            else:
                print(f"[Solve Round 1] ✅ 可行且无边界命中 (obj={obj_r1:.2f})")
            # 无论是否仍命中边界，Round 1 可行就接受
            return (speed_curve_r1, traj_data_r1, 0, obj_r1,
                    bus_info_r1, stop_info_r1, solve_history)

        # Round 1 无解 → 落入 Round 2 (pwl_physical_window)
        print(f"[Solve Round 1] ❌ 无解，进入 Round 2 (pwl_physical_window)")
    else:
        # Round 0 无解 → 直接进入 Round 2 (pwl_physical_window)
        print(f"[Solve Round 0] ❌ 无解，进入 Round 2 (pwl_physical_window)")

    # ================================================================
    # Round 2: pwl_physical_window
    # ================================================================
    # [Stochastic PWL + Physical Window 改造]
    # 本 Round 的核心思想：
    #   - PWL 仍然存在（不是 avg_delay_only）
    #   - 但 arrival window 不再使用 local window（v_ref + delta）
    #   - 而是直接使用 physical window：x_phys_lb / x_phys_ub
    #   - 这意味着 builder 需要用 physical window 重新生成一条完整的新 PWL
    #   - bundle 中 x_lb/x_ub 统一切换为 x_phys_lb/x_phys_ub
    #   - Gurobi 中 t_arr_sig 的上下界也对应 physical window
    #
    # 与旧 Round 2 (rebuild_from_fallback) 的区别：
    #   - 旧方案：用 fallback_ref_speed 重新计算 v_ref → 扩大 delta → 仍是局部窗口
    #   - 新方案：完全跳过局部窗口逻辑，直接使用全局物理窗口
    #   - 旧方案的 fallback_ref_speed / rebuild_expand_factor_infeasible 不再参与
    # ================================================================
    print(f"\n{'='*60}")
    print(f"[Solve Round 2] pwl_physical_window, t={trigger_time:.1f}")
    print(f"{'='*60}")

    signal_pwl_bundle_d_r2 = None
    model_aux_var_d_r2 = {}

    if can_build_pwl:
        try:
            # 关键：传入 window_mode_override="physical_window"
            # 这会让 builder 在构建每辆公交的 bundle 时：
            #   - 不使用 _compute_local_window() 的局部窗口
            #   - 直接使用 x_phys_lb, x_phys_ub 作为 t_arr_lb, t_arr_ub
            #   - bundle 中 x_lb = x_phys_lb, x_ub = x_phys_ub
            #   - window_mode 标记为 "physical_window"
            #   - rebuild_round 标记为 2
            signal_pwl_bundle_d_r2 = l_realtime_signal_pwl_builder.build_signal_pwl_bundles(
                current_time=trigger_time,
                bus_obj_l=bus_obj_l,
                line_obj_dic=line_obj_dic,
                signal_obj_dic=signal_obj_dic,
                sorted_busline_edge_d=sorted_busline_edge_d,
                busline_stop_edge_d=busline_stop_edge_d,
                station_dict=station_dict,
                lb_speed=pwl_lb_speed,
                up_speed=pwl_up_speed,
                pwl_generator_func=pwl_generator_func,
                pwl_config=pwl_config,
                last_control_result=last_control_result,
                rebuild_overrides=None,       # 不使用 per-bus overrides，全部重建
                existing_bundles=None,        # 不复用旧 bundle，完全重新生成
                window_mode_override="physical_window",  # [NEW] 强制使用 physical window
            )
        except Exception as e:
            print(f"[Gurobimodel WARNING] Round 2 PWL physical_window rebuild failed: {e}")
            signal_pwl_bundle_d_r2 = None

    t2_start = time_module.perf_counter()
    (model_r2, feasible_r2, speed_curve_r2, traj_data_r2, obj_r2,
     bus_info_r2, stop_info_r2, dec_var_inv_r2) = _build_and_solve_gurobi_model(
        env=env,
        bus_obj_l=bus_obj_l, stop_obj_l=stop_obj_l,
        bus_obj_id_l=bus_obj_id_l, stop_obj_id_l=stop_obj_id_l,
        trigger_time=trigger_time,
        station_dict=station_dict,
        lb_speed=decision_lb_speed,       # [分离] 决策变量专用速度下限
        up_speed=decision_up_speed,       # [分离] 决策变量专用速度上限
        station_interval_dis_dict=station_interval_dis_dict,
        scaled_bus_arrstation_od_otd_dict=scaled_bus_arrstation_od_otd_dict,
        scaled_line_station_od_otd_dict=scaled_line_station_od_otd_dict,
        static_busCap_alight_board_l=static_busCap_alight_board_l,
        od_otd_arr_rate_dict=od_otd_arr_rate_dict,
        sorted_busline_edge_d=sorted_busline_edge_d,
        line_obj_dic=line_obj_dic,
        interStop_trafficFlow_d=interStop_trafficFlow_d,
        signal_pwl_bundle_d=signal_pwl_bundle_d_r2,
        model_aux_var_d=model_aux_var_d_r2,
        solve_mode="pwl_physical_window",
        time_limit=120,  # Round 2 适中时间限制（physical window 通常比 local 宽，可能需要更多求解时间）
    )
    t2_elapsed = time_module.perf_counter() - t2_start

    pwl_info_r2 = None
    if feasible_r2 and signal_pwl_bundle_d_r2 is not None:
        pwl_info_r2 = _extract_pwl_solve_info(
            model_aux_var_d_r2, signal_pwl_bundle_d_r2, bus_obj_l, dec_var_inv_r2,
            boundary_hit_eps_abs=pwl_config.get("boundary_hit_eps_abs", 0.5),
            boundary_hit_eps_rel=pwl_config.get("boundary_hit_eps_rel", 0.02))

    _record_solve_round(
        solve_history, round_id=2, solve_mode="pwl_physical_window",
        GurobiModel=model_r2, objective_value=obj_r2 if feasible_r2 else None,
        pwl_solve_info=pwl_info_r2, signal_pwl_bundle_d=signal_pwl_bundle_d_r2,
        solve_time_sec=t2_elapsed, simulation_time=trigger_time,
        control_cycle_id=control_cycle_id)

    if feasible_r2:
        print(f"[Solve Round 2] ✅ 可行 (pwl_physical_window, obj={obj_r2:.2f})")
        return (speed_curve_r2, traj_data_r2, 0, obj_r2,
                bus_info_r2, stop_info_r2, solve_history)

    # Round 2 无解 → Round 3
    print(f"[Solve Round 2] ❌ 无解，进入 Round 3 (avg_delay_only)")

    # ================================================================
    # Round 3: avg_delay_only（禁用所有 PWL）
    # ================================================================
    print(f"\n{'='*60}")
    print(f"[Solve Round 3] avg_delay_only, t={trigger_time:.1f}")
    print(f"{'='*60}")

    model_aux_var_d_r3 = {}

    t3_start = time_module.perf_counter()
    (model_r3, feasible_r3, speed_curve_r3, traj_data_r3, obj_r3,
     bus_info_r3, stop_info_r3, dec_var_inv_r3) = _build_and_solve_gurobi_model(
        env=env,
        bus_obj_l=bus_obj_l, stop_obj_l=stop_obj_l,
        bus_obj_id_l=bus_obj_id_l, stop_obj_id_l=stop_obj_id_l,
        trigger_time=trigger_time,
        station_dict=station_dict,
        lb_speed=decision_lb_speed,       # [分离] 决策变量专用速度下限
        up_speed=decision_up_speed,       # [分离] 决策变量专用速度上限
        station_interval_dis_dict=station_interval_dis_dict,
        scaled_bus_arrstation_od_otd_dict=scaled_bus_arrstation_od_otd_dict,
        scaled_line_station_od_otd_dict=scaled_line_station_od_otd_dict,
        static_busCap_alight_board_l=static_busCap_alight_board_l,
        od_otd_arr_rate_dict=od_otd_arr_rate_dict,
        sorted_busline_edge_d=sorted_busline_edge_d,
        line_obj_dic=line_obj_dic,
        interStop_trafficFlow_d=interStop_trafficFlow_d,
        signal_pwl_bundle_d=None,  # 禁用 PWL
        model_aux_var_d=model_aux_var_d_r3,
        solve_mode="avg_delay_only",
        time_limit=120,  # Round 3 恢复正常时间限制
    )
    t3_elapsed = time_module.perf_counter() - t3_start

    _record_solve_round(
        solve_history, round_id=3, solve_mode="avg_delay_only",
        GurobiModel=model_r3, objective_value=obj_r3 if feasible_r3 else None,
        pwl_solve_info=None, signal_pwl_bundle_d=None,
        solve_time_sec=t3_elapsed, simulation_time=trigger_time,
        control_cycle_id=control_cycle_id)

    if feasible_r3:
        print(f"[Solve Round 3] ✅ 可行 (avg_delay_only, obj={obj_r3:.2f})")
        return (speed_curve_r3, traj_data_r3, 0, obj_r3,
                bus_info_r3, stop_info_r3, solve_history)

    # ================================================================
    # 全部失败
    # ================================================================
    print(f"[Solve ALL ROUNDS FAILED] 全部 {len(solve_history)} 轮求解均无可行解")
    return (0, 0, 1, 0, {}, {}, solve_history)


# ============================================================================
# 将触发时刻前后的公交车到站时间整合起来，绘制各公交车完整的到站时间曲线
# （原版注释代码保留）
# ============================================================================
# bus_arrival_time_dict = {
#     "705S_1": {"705S01": "200", "705S02": "500", "705S03_7S03": "800", "705S04_7S04": "1000"},
#     "705S_2": {"705S01": "500", "705S02": "800", "705S03_7S03": "1100"},
#     "705S_3": {"705S01": "800", "705S02": "900", "705S03_7S03": "1105"},
#     "705S_4": {"705S01": "1000"},
# }
# complete_arrival_time_dict = plot_complete_arrival_time_result(
#     arrival_time_variable_d, "arrival_time_", station_dict, bus_arrival_time_dict)
