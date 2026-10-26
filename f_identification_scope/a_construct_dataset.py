# 功能：读取i_up_control -update文件夹内，模型对公交车的控制结果，构建数据集
# 时间：2025.11.24
# 作者：LRG

import re
import ast
import xml.etree.ElementTree as ET
from collections import defaultdict
from i_up_control_update import g_get_schedule_timetable


def get_busControlScheme(add):
    """基于控制模型的输出信息的xml文件，读取各次模型触发时刻，决策的公交车的到发时间"""
    gurobiInfo_file = ET.parse(add)
    gurobiInfo_root = gurobiInfo_file.getroot()

    temp_bus_actual_timetable = {}
    for child in gurobiInfo_root:
        if child.get("gurobiNoUseFlag") == "0":
            # 表示当前模型可求解
            temp_bus_actual_timetable[child.get("id")] = ast.literal_eval(child.get("busControlScheme"))

    return temp_bus_actual_timetable


def get_all_time_bus_info(add):
    """读取所有模型触发时刻的公交车状态信息"""
    file = ET.parse(add)
    root = file.getroot()

    all_time_bus_info_d = {}
    for child in root:
        all_time_bus_info_d[child.get("id")] = {}
        for bus_child in child:
            all_time_bus_info_d[child.get("id")][bus_child.get("id")] = {}
            all_time_bus_info_d[child.get("id")][bus_child.get("id")]["cur_lane_and_remain_length"] = ast.literal_eval(bus_child.get("bus_cur_external_lane_remainLen_l"))
            all_time_bus_info_d[child.get("id")][bus_child.get("id")]["cur_speed"] = float(bus_child.get("bus_speed_n"))
            all_time_bus_info_d[child.get("id")][bus_child.get("id")]["next_stop_id"] = bus_child.get("next_stop_id_s")
            all_time_bus_info_d[child.get("id")][bus_child.get("id")]["next_stop_length"] = float(bus_child.get("next_stop_length_n"))
            all_time_bus_info_d[child.get("id")][bus_child.get("id")]["onboard_passenger_num"] = sum(list(ast.literal_eval(bus_child.get("bus_passenger_d")).values()))
            all_time_bus_info_d[child.get("id")][bus_child.get("id")]["just_server_stop_data"] = ast.literal_eval(bus_child.get("just_server_stop_data_l"))
    return all_time_bus_info_d


def get_all_time_stop_info(add):
    """读取所有模型触发时刻的公交站状态信息"""
    file = ET.parse(add)
    root = file.getroot()

    all_time_stop_info_d = {}
    for timestep_child in root:
        all_time_stop_info_d[timestep_child.get("id")] = {}
        for stop_child in timestep_child:
            # 各站保存的信息依次为刚服务的公交车ID、公交线路ID、到达时间、发车时间、发车后站内乘客分布
            all_time_stop_info_d[timestep_child.get("id")][stop_child.get("id")] = ast.literal_eval(stop_child.get("just_leave_data_l"))

    return all_time_stop_info_d


def get_all_time_busline_to_stop_flow_speed(add):
    """读取所有模型触发时刻的公交车在各站间的平均交通流速度"""
    file = ET.parse(add)
    root = file.getroot()

    all_time_busline_to_stop_flow_speed_d = {}
    for child in root:
        try:
            all_time_busline_to_stop_flow_speed_d[child.get("id")] = ast.literal_eval(child.get("edge_traffic_speed_d"))
        except:
            debug = 0
    return all_time_busline_to_stop_flow_speed_d


def calculate_passenger_count(Passenger_file_add, timestep_l):
    """
    根据乘客在各公交站的时间区间，统计每个关键时刻在每个公交站的乘客数量。

    参数:
    passenger_atStop_interval_d (dict):
        乘客-公交站-时间区间字典。
        e.g. {"passenger_id": {"stop_id": [start_time, end_time], ...}, ...}
    timestep_l (list):
        关键时刻列表。
        e.g. [time1, time2, ...]

    返回:
    dict:
        关键时刻-公交站-乘客数量字典。
        e.g. {"time": {"stop_id": count, ...}, ...}
    """

    passenger_atStop_interval_d = get_all_passenger_atStop_timeInterval(Passenger_file_add)

    # 初始化结果字典。使用 defaultdict(lambda: defaultdict(int))
    # 可以自动处理时间点和公交站点的键不存在的情况，直接进行计数。
    all_time_atStop_passenger_num_d = defaultdict(lambda: defaultdict(int))

    # 1. 遍历所有的乘客和他们的信息
    for passenger_id, stop_info in passenger_atStop_interval_d.items():
        # 2. 遍历该乘客在所有公交站的信息
        for stop_id, time_interval in stop_info.items():
            start_time, end_time = time_interval

            # 3. 遍历所有的关键时刻
            for time_step in timestep_l:
                # 检查该关键时刻 time_step 是否落在乘客的 [start_time, end_time] 区间内
                # 假设时间区间是闭区间 [start_time, end_time]，即包括起点和终点
                # 如果是半开区间 [start_time, end_time)，则条件应为: start_time <= time_step < end_time
                if start_time <= float(time_step) <= end_time:
                    # 如果在区间内，则在相应的时间点和公交站进行计数
                    # 注意：为了保持输出格式和键的类型一致，我们通常将时间转换为字符串作为字典的键
                    time_step_str = str(time_step)

                    # 计数器加一
                    all_time_atStop_passenger_num_d[time_step_str][stop_id] += 1

    # 将 defaultdict 转换回普通的 dict 返回，方便外部使用和查看
    return dict(all_time_atStop_passenger_num_d)


def get_all_passenger_atStop_timeInterval(add):
    """基于仿真结束生成的乘客信息，记录各乘客在站内的时间区间，便于统计各模型触发时刻，各站有多少乘客"""
    file = ET.parse(add)
    root = file.getroot()

    """获取各乘客在公交站的等待时间区间"""
    passenger_atStop_interval_d = {}
    for passenger_child in root:
        passenger_id = passenger_child.get("passenger_id_s")
        if passenger_id not in passenger_atStop_interval_d:

            # 判断该乘客是否换乘
            if passenger_child.get("transfer_stop_id_s") != "":   # 会换乘
                for travel_data_child in passenger_child:
                    travel_parts = travel_data_child.get("travel_data_l").split()
                    stop_parts = [passenger_child.get("start_stop_id_s"), passenger_child.get("transfer_stop_id_s")]

                    for temp_index, temp_parts in enumerate(travel_parts):

                        # 判断该乘客是否已经进入公交站
                        if temp_index == 0:
                            first_travel_data_l = temp_parts.split(";")
                            if len(first_travel_data_l) == 6 and "_" in first_travel_data_l[-1]:
                                passenger_atStop_interval_d[passenger_id] = {}
                            else:
                                break

                        # 若乘客已经进入公交站，则保存其信息
                        nums = re.findall(r'\d+\.?\d*', temp_parts)
                        if len(nums) >= 2:
                            passenger_atStop_interval_d[passenger_id][stop_parts[temp_index]] = [float(nums[0]), float(nums[1])]

    return passenger_atStop_interval_d


def generate_delay_bus_dict(all_time_bus_info_d, bus_schedule, ThresholdDelayArrive):
    """
    生成各时刻晚点公交车字典

    Parameters:
    -----------
    all_time_bus_info_d : dict
        各时刻公交车信息
    bus_schedule : dict
        公交车计划时刻表
    ThresholdDelayArrive : float
        晚点阈值（秒），默认60秒

    Returns:
    --------
    AllTimeDelayBus_d : dict
        {"时刻": ["晚点车辆1", "晚点车辆2", ...], ...}
    """
    AllTimeDelayBus_d = {}

    for time_str, buses_info in all_time_bus_info_d.items():
        # 转换时间为整数（去掉小数）
        time_key = str(int(float(time_str)))
        delay_buses = []

        for bus_id, bus_data in buses_info.items():
            # 检查是否有刚服务的站点信息
            if 'just_server_stop_data' not in bus_data:
                continue

            just_server_stop_data = bus_data['just_server_stop_data']
            if not just_server_stop_data or len(just_server_stop_data) < 2:
                continue

            # 提取刚服务站点和实际到达时间
            stop_id = just_server_stop_data[0]
            actual_arrival_time = just_server_stop_data[1]

            # 提取线路标号（从公交车标号中提取，如 "311S_2" -> "311S"）
            line_id = bus_id.rsplit('_', 1)[0]

            # 查询计划时刻表
            try:
                scheduled_time = bus_schedule[line_id][bus_id][stop_id][0]  # [0]是到达时间

                # 计算延迟
                delay = actual_arrival_time - scheduled_time

                # 判断是否晚点
                if delay > ThresholdDelayArrive:
                    delay_buses.append(bus_id)
                if delay < -120:
                    print(f"Bus {bus_id} arrives at {stop_id} early!\n")
            except KeyError:
                # 如果找不到该车或该站的计划时刻表，跳过
                continue

        # 保存该时刻的晚点车辆列表
        if delay_buses:
            AllTimeDelayBus_d[time_key] = delay_buses

    return AllTimeDelayBus_d


# """1.获取无控制、无晚点设计、高限速程度下的公交车计划时刻表"""
# bus_schedule = g_get_schedule_timetable.get_bus_schedule()
#
# """2. 读取控制模型的输出信息"""
# folder_add = "../i_up_control_update/"
# for temp_num in range(5, 6):
#
#     # 获取在各模型触发时刻，模型对公交车到发时间的决策结果
#     GurobiInfo_file_add = folder_add + "output_file_" + str(temp_num) + "/GurobimodelInfo.xml"
#     all_time_bus_decision_timetable = get_busControlScheme(GurobiInfo_file_add)
#
#     # 获取在各模型触发时刻，公交车、公交站、路网、乘客信息
#     CurBusInfo_file_add = folder_add + "output_file_" + str(temp_num) + "/cur_bus_info.xml"
#     CurStopInfo_file_add = folder_add + "output_file_" + str(temp_num) + "/cur_stop_info.xml"
#     CurBuslineEdgeTrafficSpeed_file_add = folder_add + "output_file_" + str(temp_num) + "/cur_busline_edge_traffic_speed_info.xml"
#     Passenger_file_add = folder_add + "output_file_" + str(temp_num) + "/save_data_passenger.add.xml"
#
#     all_time_bus_info_d = get_all_time_bus_info(CurBusInfo_file_add)
#     all_time_stop_info_d = get_all_time_stop_info(CurStopInfo_file_add)
#     all_time_busline_to_stop_flow_speed_d = get_all_time_busline_to_stop_flow_speed(CurBuslineEdgeTrafficSpeed_file_add)
#     all_time_atStop_passenger_num_d = calculate_passenger_count(Passenger_file_add, list(all_time_bus_decision_timetable.keys()))
#
#     # 获取在各模型触发时刻，公交车晚点数量情况
#     ThresholdDelayArrive = 60
#     AllTimeDelayBus_d = generate_delay_bus_dict(all_time_bus_info_d, bus_schedule, ThresholdDelayArrive)
#
#
#     debug = 0
#
#
#
#
#
# debug = 0
