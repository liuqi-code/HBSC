#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import Dict, List, Tuple
from collections import defaultdict

# 设置字体为 Times New Roman
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman']
plt.rcParams['mathtext.fontset'] = 'stix'  # 数学字体也使用类似Times的字体
plt.rcParams['axes.unicode_minus'] = False

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "1_experiment_shockwave_vs_rule_outputs")
os.makedirs(OUTPUT_DIR, exist_ok=True)


# ================================================================================
# 第一部分：从SUMO路网提取的真实信号配时数据（硬编码）
# ================================================================================
# 数据来源:  1_changsha_bus_network_with_signal_f_net.xml
# 提取方法:  解析 <tlLogic> 和 <connection tl=...> 标签
#
# 每个交叉口记录:
#   phases: [(duration_s, state_string), ...]
#   对于指定的 linkIndex，遍历所有phase的state_string，
#   该位置为 'G' 或 'g' 则计入绿灯时长，否则计入红灯时长。

SIGNAL_DATA = {
    # J10: 7个link, 周期=90s
    'J10': [
        (30, 'GgrrGGG'),
        (10,  'rgrryyy'),
        (6,  'rGrrrrr'),
        (3,  'ryrrrrr'),
        (30, 'GrGGrrr'),
        (10,  'rryyrrr'),
        (1,  'rrrrrrr'),
    ],
    # J100: 16个link, 周期=90s
    'J100': [
        (33, 'rrrrGGGgrrrrGGGg'),
        (3,  'rrrryyygrrrryyyg'),
        (6,  'rrrrrrrGrrrrrrrG'),
        (3,  'rrrrrrryrrrrrrry'),
        (32, 'GGGgrrrrGGGgrrrr'),
        (3,  'yyygrrrryyygrrrr'),
        (6,  'rrrGrrrrrrrGrrrr'),
        (3,  'rrryrrrrrrryrrrr'),
        (1,  'rrrrrrrrrrrrrrrr'),
    ],
    # J130: 16个link, 周期=90s
    'J130': [
        (33, 'rrrrGGGgrrrrGGGg'),
        (3,  'rrrryyygrrrryyyg'),
        (6,  'rrrrrrrGrrrrrrrG'),
        (3,  'rrrrrrryrrrrrrry'),
        (33, 'GGGgrrrrGGGgrrrr'),
        (3,  'yyygrrrryyygrrrr'),
        (6,  'rrrGrrrrrrrGrrrr'),
        (3,  'rrryrrrrrrryrrrr'),
    ],
    # J170: 9个link, 周期=90s
    'J170': [
        (38, 'GGGGGggGr'),
        (3,  'yyyyyggyr'),
        (6,  'grrGGGGgr'),
        (3,  'grryyyygr'),
        (37, 'GrrGGrrGG'),
        (3,  'yrryyrryy'),
    ],
}


def get_green_red_time(tl_id: str, link_index: int) -> Tuple[float, float, float]:
    """
    从硬编码的信号配时数据中计算指定 linkIndex 的绿灯/红灯时长。

    遍历所有 phase，检查 state_string 中对应 linkIndex 位置的字符：
    - 'G' 或 'g'  ->  计入绿灯时长
    - 其他 ('r', 'y', ...)  ->  计入红灯时长

    返回: (green_time, red_time, cycle_time)
    """
    phases = SIGNAL_DATA.get(tl_id)
    if phases is None:
        raise ValueError(f"Signal data for {tl_id} not found")
    green_t = 0.0
    red_t = 0.0
    for dur, state in phases:
        if link_index < len(state) and state[link_index] in ('G', 'g'):
            green_t += dur
        else:
            red_t += dur
    return green_t, red_t, green_t + red_t


# ================================================================================
# 第二部分：冲击波模型（自包含实现，与 step2 算法完全一致）
# ================================================================================

@dataclass
class _Seg:
    """信号时段"""
    t0: float
    t1: float
    eff_green: bool


class SingleCycleShockwave:
    """
    单信号周期冲击波模型

    坐标系:
      - 时间: 红灯开始 = t=0, [0, red) 为红灯, [red, cycle) 为绿灯
      - 空间: 停止线 = x=0, 上游为正方向

    核心算法:
      1. u(t) 信号门控函数: 有效绿灯=1, 否则=0
      2. d_q(t) 队尾轨迹: 分段线性
         - 红灯/启动损失期: 斜率 = +v_q  (队列增长)
         - 有效绿灯期:     斜率 = v_q - v_c (通常为负, 队列消散)
      3. 给定虚拟到达时刻 t_arr, d_join = d_q(t_arr)
      4. 放行工作量 H = d_join / v_c
      5. 从 t_arr 开始累计有效绿灯时间, 达到 H 时即为 t_pass
      6. delay = t_pass - t_arr
    """

    def __init__(self,
                 arrival_rate: float,           # 到达流率 (veh/h)
                 capacity: float,               # 饱和流率/通行能力 (veh/h)
                 cycle_time: float,             # 周期时长 (s)
                 green_time: float,             # 绿灯时长 (s)
                 initial_queue_length: float,   # 红灯开始时的残余排队 (m)
                 average_vehicle_length: float = 7.0,
                 jam_density: float = 140.0,    # 阻塞密度 (veh/km)
                 free_flow_speed: float = 50.0, # 自由流速度 (km/h)
                 startup_lost_time: float = 2.0):  # 启动损失时间 tau (s)

        self.C = cycle_time
        self.g = green_time
        self.r = cycle_time - green_time
        self.R0 = initial_queue_length
        self.tau = startup_lost_time

        # ---- 单位转换与冲击波速度 ----
        q_a = arrival_rate / 3600.0   # veh/s
        q_s = capacity / 3600.0       # veh/s
        k_j = jam_density / 1000.0    # veh/m
        v_f = free_flow_speed / 3.6   # m/s
        k_a = q_a / v_f if v_f > 0 else 0.0

        # v_q: 队尾增长速率 (到达冲击波, m/s)
        if k_j > k_a and q_a > 0:
            self.v_q = q_a / (k_j - k_a)
        else:
            self.v_q = q_a * average_vehicle_length if q_a > 0 else 0.0
        # v_c: 队列消散速率 (放行冲击波, m/s)
        self.v_c = q_s * average_vehicle_length

        # ---- 信号时段序列 ----
        self.segs: List[_Seg] = []
        eg_start = self.r + self.tau  # 有效绿灯起始

        if self.r > 0:
            self.segs.append(_Seg(0.0, self.r, False))
        if self.tau > 0 and eg_start < self.C:
            self.segs.append(_Seg(self.r, min(eg_start, self.C), False))
        if eg_start < self.C:
            self.segs.append(_Seg(eg_start, self.C, True))

        self.eg_start = eg_start

        # ---- 队尾轨迹 (分段线性) ----
        self.qtraj: List[Tuple[float, float]] = []
        dq = self.R0
        self.qtraj.append((0.0, dq))

        for seg in self.segs:
            rate = (self.v_q - self.v_c) if seg.eff_green else self.v_q
            dq_end = max(0.0, dq + rate * (seg.t1 - seg.t0))
            if dq > 0 and rate < 0:
                tc = dq / abs(rate)
                if tc < (seg.t1 - seg.t0):
                    self.qtraj.append((seg.t0 + tc, 0.0))
                    dq_end = 0.0
            self.qtraj.append((seg.t1, dq_end))
            dq = dq_end

        self.queue_end = dq

    def _dq(self, t: float) -> float:
        """分段线性插值获取t时刻队尾位置"""
        if t <= 0:
            return self.R0
        if t >= self.C:
            return self.queue_end
        for i in range(len(self.qtraj) - 1):
            t1, d1 = self.qtraj[i]
            t2, d2 = self.qtraj[i + 1]
            if t1 <= t <= t2:
                return d1 + (d2 - d1) * (t - t1) / (t2 - t1) if t2 > t1 else d1
        return self.qtraj[-1][1]

    def compute_delay(self, t_arr: float) -> float:
        """
        计算公交车在周期内到达时刻 t_arr 的延误。

        步骤:
          1. d_join = d_q(t_arr)  (到达时刻的排队长度)
          2. H = d_join / v_c     (放行工作量)
          3. 从 t_arr 逐段累计有效绿灯时间
          4. 当累计 >= H 时, 得到 t_pass
          5. delay = t_pass - t_arr
          6. 若当前周期不够, 延伸到下一周期有效绿灯
        """
        t_arr = float(np.clip(t_arr, 0.0, self.C - 0.01))

        d_join = self._dq(t_arr)
        H = d_join / self.v_c if self.v_c > 0 else 0.0

        if H < 1e-9:
            if t_arr >= self.eg_start:
                return 0.0
            else:
                return max(0.0, self.eg_start - t_arr)

        cum = 0.0
        for seg in self.segs:
            if seg.t1 <= t_arr:
                continue
            s0 = max(seg.t0, t_arr)
            if seg.eff_green:
                avail = seg.t1 - s0
                need = H - cum
                if avail >= need:
                    return max(0.0, (s0 + need) - t_arr)
                cum += avail

        # 跨周期: 下一周期有效绿灯开始处继续
        remain = H - cum
        t_pass = self.C + self.eg_start + remain
        return max(0.0, t_pass - t_arr)


# ================================================================================
# 第三部分：规则法平均延误 (Webster 延误公式第一项)
# ================================================================================

def rule_delay(r: float, C: float) -> float:
    """
    d_avg = r^2 / (2*C)

    假设:
      - 车辆在整个周期内均匀随机到达
      - P(红灯到达) = r/C, 红灯到达平均等待 = r/2
      - P(绿灯到达) = g/C, 绿灯到达平均等待 = 0
      - d_avg = (r/C)*(r/2) = r^2/(2C)

    关键: 这是一个仅由(r,C)决定的常数, 与公交车状态完全无关
    """
    return (r * r) / (2.0 * C) if C > 0 else 0.0


# ================================================================================
# 第四部分：蒙特卡洛冲击波延误估计
# ================================================================================

def mc_delay(C, g, t_arr, q0, base_flow=400.0, base_cap=1600.0,
             n=200, tau=2.0, cap_cv=0.10, flow_cv=0.20):
    """
    蒙特卡洛冲击波延误估计。
    对通行能力和到达流率进行正态随机抽样(截断到合理范围),
    对每次抽样建立冲击波模型计算延误, 返回统计量。
    """
    caps = np.clip(np.random.normal(base_cap, base_cap * cap_cv, n), 400, 3000)
    flows = np.clip(np.random.normal(base_flow, base_flow * flow_cv, n), 0, 2000)
    ds = np.empty(n)
    for i in range(n):
        sw = SingleCycleShockwave(
            arrival_rate=flows[i], capacity=caps[i],
            cycle_time=C, green_time=g,
            initial_queue_length=q0, startup_lost_time=tau)
        ds[i] = sw.compute_delay(t_arr)
    return {
        'mean': float(np.mean(ds)),  'std': float(np.std(ds)),
        'median': float(np.median(ds)),
        'min': float(np.min(ds)),    'max': float(np.max(ds)),
        'p25': float(np.percentile(ds, 25)),
        'p75': float(np.percentile(ds, 75)),
        'all': ds,
    }


# ================================================================================
# 第五部分：实验场景定义与执行
# ================================================================================

@dataclass
class Scenario:
    sid: str; name: str; tl_id: str; link_index: int
    from_edge: str; direction: str
    queue_m: float; queue_desc: str
    arr_desc: str; t_arr: float
    base_flow: float; base_cap: float

@dataclass
class Result:
    sc: Scenario
    C: float; g: float; r: float
    phase: str  # Red / Green
    sw_mean: float; sw_std: float; sw_med: float
    sw_min: float; sw_max: float
    rule_d: float; diff: float; rel: float


def build_scenarios() -> List[Scenario]:
    """构建 4 交叉口 x 6 场景 = 24 个实验场景"""
    ixs = [
        ('J10',  0,  'E_J19_J10',   's-straight', 500,  1650),
        ('J100', 10, 'E_J73_J100',  's-straight', 450,  1600),
        ('J130', 14, 'E_J102_J130', 's-straight', 550,  1700),
        ('J170', 1,  'E_J202_J170', 's-straight', 480,  1620),
    ]
    tmpls = [
        ('S1', 'Green-mid + No queue',           'green_mid',  0,   'No queue'),
        ('S2', 'Green-mid + Med queue(50m)',      'green_mid',  20,  'Queue 20m'),
        ('S3', 'Red-start + No queue',            'red_start',  0,   'No queue'),
        ('S4', 'Red-start + Large queue(120m)',   'red_start',  50, 'Queue 50m'),
        ('S5', 'Red-end + No queue',              'red_end',    0,   'No queue'),
        ('S6', 'Green-end + Large queue(120m)',   'green_end',  50, 'Queue 50m'),
    ]
    arr_labels = {'green_mid':'Green middle', 'red_start':'Red start',
                  'red_end':'Red end', 'green_end':'Green end'}
    out = []
    for tl, li, fe, dr, bf, bc in ixs:
        gt, rt, ct = get_green_red_time(tl, li)
        for sf, nm, rule, qm, qd in tmpls:
            if rule == 'green_mid':   ta = rt + gt * 0.5
            elif rule == 'red_start': ta = 1.0
            elif rule == 'red_end':   ta = max(rt - 2.0, 0.5)
            else:                     ta = ct - 2.0  # green_end
            out.append(Scenario(
                sid=f"{tl}-{sf}", name=nm, tl_id=tl, link_index=li,
                from_edge=fe, direction=dr, queue_m=qm, queue_desc=qd,
                arr_desc=arr_labels[rule], t_arr=ta,
                base_flow=bf, base_cap=bc))
    return out


def run_all(scenarios, n_mc=500):
    """执行全部实验"""
    print("=" * 88)
    print("  Experiment: Shockwave E[delay] vs Rule-based avg delay r^2/(2C)")
    print("=" * 88)
    print(f"  Monte Carlo samples/scenario: {n_mc}   |   Scenarios: {len(scenarios)}")
    print("-" * 88)

    results = []
    for i, sc in enumerate(scenarios):
        gt, rt, ct = get_green_red_time(sc.tl_id, sc.link_index)
        ph = "Red" if sc.t_arr < rt else "Green"
        m = mc_delay(ct, gt, sc.t_arr, sc.queue_m, sc.base_flow, sc.base_cap, n_mc)
        rd = rule_delay(rt, ct)
        d = m['mean'] - rd
        rp = (d / rd * 100) if rd > 0 else 0.0
        results.append(Result(sc=sc, C=ct, g=gt, r=rt, phase=ph,
            sw_mean=m['mean'], sw_std=m['std'], sw_med=m['median'],
            sw_min=m['min'], sw_max=m['max'],
            rule_d=rd, diff=d, rel=rp))
        if i % 6 == 0:
            print(f"\n  === {sc.tl_id} | C={ct:.0f}s g={gt:.0f}s r={rt:.0f}s | "
                  f"Rule={rd:.2f}s ===")
        s = '+' if d >= 0 else ''
        print(f"    {sc.sid:>8} | {sc.arr_desc:>14} t={sc.t_arr:5.1f}s ({ph:5}) | "
              f"Q={sc.queue_m:>5.0f}m | SW={m['mean']:6.2f} +/-{m['std']:5.2f} | "
              f"Rule={rd:6.2f} | {s}{d:.2f}s")
    print("\n" + "=" * 88)
    return results


# ================================================================================
# 第六部分：报告与可视化
# ================================================================================

def summary(results):
    print("\n" + "#" * 88)
    print("  EXPERIMENT SUMMARY & CONCLUSIONS")
    print("#" * 88)

    by = defaultdict(list)
    for r in results:
        by[r.sc.sid.split('-')[1]].append(r)

    print(f"\n  {'Type':<6} {'Description':<38} {'SW_Mean':>8} {'Rule':>8} {'Diff':>9}")
    print(f"  {'-'*74}")
    for s in ['S1','S2','S3','S4','S5','S6']:
        rs = by[s]
        a = np.mean([r.sw_mean for r in rs])
        b = rs[0].rule_d
        d = a - b
        print(f"  {s:<6} {rs[0].sc.name:<38} {a:>7.2f}s {b:>7.2f}s {d:>+8.2f}s")

    print(f"\n  KEY FINDINGS:")
    print("  " + "-" * 74)

    s1a = np.mean([r.sw_mean for r in by['S1']])
    s4a = np.mean([r.sw_mean for r in by['S4']])
    rd = by['S1'][0].rule_d

    print(f"  [S1] Green-mid + No queue:  SW={s1a:.2f}s  Rule={rd:.2f}s")
    print(f"       => Shockwave identifies FAVORABLE state: delay << rule")
    print(f"  [S4] Red-start + Large queue: SW={s4a:.2f}s  Rule={rd:.2f}s")
    print(f"       => Shockwave identifies UNFAVORABLE state: delay >> rule")
    ratio = s4a / s1a if s1a > 0.01 else float('inf')
    print(f"  [Ratio] Worst/Best = {s4a:.1f}/{s1a:.1f} = {ratio:.1f}x")
    print(f"          Rule-based gives SAME {rd:.2f}s for both!")
    print(f"\n  CONCLUSION:")
    print("  " + "-" * 74)
    print(f"  The shockwave model adapts to bus-intersection state:")
    print(f"    Favorable  -> {s1a:.2f}s  |  Unfavorable -> {s4a:.2f}s")
    print(f"  Rule-based is state-blind: always {rd:.2f}s")
    print(f"  => Shockwave method is valuable for bus priority optimization.\n")


def save_csv(results, path):
    with open(path, 'w', newline='', encoding='utf-8-sig') as f:
        w = csv.writer(f)
        w.writerow(['scenario_id','name','tl_id','from_edge','direction',
                     'cycle_s','green_s','red_s','arrival_desc','t_arrival_s',
                     'phase','queue_m','queue_desc','base_flow','base_cap',
                     'sw_mean','sw_std','sw_median','sw_min','sw_max',
                     'rule_delay','diff','rel_pct'])
        for r in results:
            s = r.sc
            w.writerow([s.sid,s.name,s.tl_id,s.from_edge,s.direction,
                f'{r.C:.0f}',f'{r.g:.0f}',f'{r.r:.0f}',s.arr_desc,
                f'{s.t_arr:.1f}',r.phase,f'{s.queue_m:.0f}',s.queue_desc,
                f'{s.base_flow:.0f}',f'{s.base_cap:.0f}',
                f'{r.sw_mean:.2f}',f'{r.sw_std:.2f}',f'{r.sw_med:.2f}',
                f'{r.sw_min:.2f}',f'{r.sw_max:.2f}',
                f'{r.rule_d:.2f}',f'{r.diff:.2f}',f'{r.rel:.1f}'])
    print(f"[CSV] {path}")


def plot_profiles(results, path):
    """
    延误剖面图: 扫描整个周期, 展示冲击波延误随到达时刻变化。
    冲击波 = 状态敏感曲线, 规则法 = 一条水平线。
    """
    grps = defaultdict(list)
    for r in results: grps[r.sc.tl_id].append(r)
    tls = ['J10', 'J170']
    fig, axes = plt.subplots(1, 2, figsize=(16, 6))
    af = np.atleast_1d(axes).flatten()

    FrontSize = 20
    Frontype = 'Times New Roman'

    for idx, tl in enumerate(tls):
        ax = af[idx]; r0 = grps[tl][0]
        gt, rt, ct = r0.g, r0.r, r0.C
        ts = np.linspace(0.5, ct-0.5, 300)

        cfgs = [('Queue=0m',0,'#27AE60',2.5),('Queue=20m',20,'#F39C12',2),('Queue=50m',50,'#E74C3C',2)]
        for lb, q, co, lw in cfgs:
            ds = [mc_delay(ct,gt,t,q,r0.sc.base_flow,r0.sc.base_cap,n=50000)['mean'] for t in ts]
            ax.plot(ts, ds, color=co, lw=lw, label=lb, alpha=0.9)

        rd = rule_delay(rt, ct)
        ax.axhline(y=rd, color='black', ls='--', lw=2, alpha=0.7, label=f'Rule={rd:.1f}s')
        ax.axvspan(0,rt,alpha=0.07,color='red')
        ax.axvspan(rt,ct,alpha=0.07,color='green')

        # 标注相位文字
        ymax = ax.get_ylim()[1] if ax.get_ylim()[1] > 0 else 10
        ax.text(rt/2, ymax*0.92, f'Red ({rt:.0f}s)', ha='center', fontsize=FrontSize-2, color='#C0392B', alpha=0.6, fontweight='bold')
        ax.text(rt+(ct-rt)/2, ymax*0.92, f'Green ({gt:.0f}s)', ha='center', fontsize=FrontSize-2, color='#27AE60', alpha=0.6, fontweight='bold')

        ax.set_xlabel('Arrival time in cycle (s)', fontsize=FrontSize)
        ax.set_ylabel('Delay (s)', fontsize=FrontSize)
        # ax.set_title(f'{tl} | C={ct:.0f}s g={gt:.0f}s r={rt:.0f}s', fontsize=12)
        ax.legend(fontsize=FrontSize-2, loc='upper right', bbox_to_anchor=(1.0, 0.75))
        # ax.grid(True,alpha=0.3)
        ax.set_xlim(0,ct);
        ax.set_ylim(bottom=0)
        ax.tick_params(axis='both', labelsize=FrontSize)

    plt.suptitle('Delay Profile: Shockwave (state-sensitive curves) vs Rule-based (flat line)',
                 fontsize=FrontSize, fontweight='bold', y=1.02)
    plt.tight_layout();
    plt.savefig(path,dpi=1000,bbox_inches='tight');
    plt.close()
    print(f"[Fig] {path}")


# ================================================================================
# 第七部分：主函数
# ================================================================================

def main():
    N_MC = 50
    print("\n" + "=" * 88)
    print("  Shockwave Expected Delay vs Rule-based Average Delay")
    print("  Signal data: 1_changsha_bus_network_with_signal_f_net.xml (embedded)")
    print("=" * 88)

    # 打印信号配时
    print(f"\n  {'TL':>6} {'Link':>5} {'From Edge':<22} {'C':>4} {'g':>4} {'r':>4}  {'Rule r^2/(2C)':>14}")
    print("  "+"-"*66)
    for tl,li,fe in [('J10',0,'E_J19_J10'),('J100',10,'E_J73_J100'),
                      ('J130',14,'E_J102_J130'),('J170',1,'E_J202_J170')]:
        gt,rt,ct = get_green_red_time(tl,li)
        print(f"  {tl:>6} {li:>5} {fe:<22} {ct:>4.0f} {gt:>4.0f} {rt:>4.0f}  {rule_delay(rt,ct):>13.2f}s")
    print()

    scenarios = build_scenarios()
    np.random.seed(42)
    results = run_all(scenarios, N_MC)
    summary(results)

    csv_p = os.path.join(OUTPUT_DIR, "experiment_results.csv")
    save_csv(results, csv_p)

    print("[Vis] Generating delay profiles (~1 min)...")
    plot_profiles(results, os.path.join(OUTPUT_DIR, "delay_profile.svg"))

    print("\n"+"="*88)
    print("  Output files:")
    for f in ["experiment_results.csv","delay_profile.svg"]:
        print(f"    {os.path.join(OUTPUT_DIR, f)}")
    print("="*88+"\n  Done!\n")

if __name__ == "__main__":
    main()
