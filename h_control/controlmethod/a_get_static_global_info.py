import re
import random
import copy


def get_busline_from_station_ID(station_ID, temp_station_dict):
    """基于共线公交站的名称获取涉及的公交线路 公交线路构成的列表 ["705S","7S"...]"""
    shared_busline = []
    split_station_list = station_ID.split("_")
    for temp_split_station in split_station_list:
        shared_busline.append(re.match(r"([0-9]*[A-Z]+)([0-9]*)", temp_split_station).groups()[0])
    return list(set(shared_busline) & set(temp_station_dict.keys()))


def get_busline_station_od_otd_dict(temp_station_dict):
    """基于各线路的公交站信息确定各站的od、otd {公交线路1:{公交站1:{od1,od2,otd1,...}}}"""
    line_station_od_otd = {}
    # 确定OD
    for busline in temp_station_dict:
        line_station_od_otd[busline] = {}
        for index, station in enumerate(temp_station_dict[busline][:-1]):
            if station not in line_station_od_otd[busline]:
                line_station_od_otd[busline][station] = []
            # 添加该站涉及的本线路的OD
            for station_D_index in range(index+1, len(temp_station_dict[busline])):
                line_station_od_otd[busline][station].append(station + "-" + temp_station_dict[busline][station_D_index])
            # 该公交站为共线公交站，需要增加该站其它线路的OD
            if "_" in station and len(list(set([re.match(r'^(\d+[A-Z])', part).group(1) for part in station.split('_')]) & set(list(temp_station_dict.keys())))) > 1:
                contributed_buslines = get_busline_from_station_ID(station, temp_station_dict)
                contributed_buslines.remove(busline)
                # 添加OD
                for contributed_busline in contributed_buslines:
                    if contributed_busline in list(temp_station_dict.keys()) and station in temp_station_dict[contributed_busline]:
                        station_index_contributed_busline = temp_station_dict[contributed_busline].index(station)
                        for station_D_index_contributed_busline in range(station_index_contributed_busline + 1, len(temp_station_dict[contributed_busline])):
                            if station + "-" + temp_station_dict[contributed_busline][station_D_index_contributed_busline] not in line_station_od_otd[busline][station]:
                                line_station_od_otd[busline][station].append(station + "-" +
                                                                             temp_station_dict[contributed_busline][station_D_index_contributed_busline])

    # # 确定OTD
    # for busline in temp_station_dict:
    #     for index_O, station in enumerate(temp_station_dict[busline]):
    #         if station == "122S15_311S21_406X13":
    #             debug = 0
    #         if "_" in station and len(list(set([re.match(r'^(\d+[A-Z])', part).group(1) for part in station.split('_')]) & set(list(temp_station_dict.keys())))) > 1:
    #             # 共线车站需考虑所有相关线路后续涉及到的中转站构造OTD
    #             split_station_list = station.split("_")
    #             shared_busline_list = []
    #             for temp_split_station in split_station_list:
    #                 if re.match(r"([0-9]*[A-Z]+)([0-9]*)", temp_split_station).groups()[0] in list(temp_station_dict.keys()):
    #                     shared_busline_list.append(re.match(r"([0-9]*[A-Z]+)([0-9]*)", temp_split_station).groups()[0])
    #             line_station_od_otd = get_OTD_station(index_O, busline, temp_station_dict, line_station_od_otd, station,
    #                                                   shared_busline_list)
    #             # 共线车站需要额外考虑作为中转站T构造OTD
    #             line_station_od_otd = get_TOD_station(busline, temp_station_dict, line_station_od_otd,
    #                                                   station, shared_busline_list)
    #         else:    # 非共线车站只需考虑本线路后续涉及到的中转站构造OTD
    #             line_station_od_otd = get_OTD_station(index_O, busline, temp_station_dict, line_station_od_otd, station,
    #                                                   [busline])

    return line_station_od_otd


def get_scaled_line_station_od_otd_dict(line_station_od_otd_dict):
    """对line_station_od_otd_dict中的部分OD进行删减，从而降低Gurobi模型求解复杂度"""
    station_od_d = {}
    scaled_line_station_od_otd_dict = {}
    for busline in line_station_od_otd_dict:
        scaled_line_station_od_otd_dict[busline] = {}
        for station in line_station_od_otd_dict[busline]:
            scaled_line_station_od_otd_dict[busline][station] = []

            # 该车站od信息第一次计算
            if station not in station_od_d:
                station_od_d[station] = []
                for index, od_otd in enumerate(line_station_od_otd_dict[busline][station]):
                    # 每个车站删减一半的OD，降低模型求解复杂度
                    if index % 8 == 0:
                        scaled_line_station_od_otd_dict[busline][station].append(od_otd)
                        station_od_d[station].append(od_otd)
            # 该车站为共线车站，已经在其他线路处理过了
            else:
                scaled_line_station_od_otd_dict[busline][station] = copy.deepcopy(station_od_d[station])

    return scaled_line_station_od_otd_dict


def get_bus_arrstation_od_otd(line_station_od_otd_dict, station_dict):
    """获取不同公交到达车站时，车内可能的od {公交线路1:{公交站1:{od1,od2,otd1,...}}}"""
    bus_arrstation_od_otd = {}
    for busline in station_dict:
        bus_arrstation_od_otd[busline] = {}
        for station in station_dict[busline][1:]:  # 遍历该线路第二个及后续车站记录公交到站前车内数据
            bus_arrstation_od_otd[busline][station] = []
            station_index = station_dict[busline].index(station)
            passed_station_list = station_dict[busline][:station_index]  # 获取该线路在该站之前的所有公交站
            for passed_station in passed_station_list:  # 遍历已经完成服务的所有公交站
                passed_station_od_otd_list = line_station_od_otd_dict[busline][passed_station]
                # 去除D为passed_station_list元素之一的od和otd
                filted_passed_station_od_otd_list = [item for item in passed_station_od_otd_list
                                                     if item.split("-")[-1] not in passed_station_list]
                # 去除D为其它线路公交站的od、OT均为passed_station_list元素之一的otd
                temp_filted_passed_station_od_otd_list = copy.deepcopy(filted_passed_station_od_otd_list)
                for temp_item in temp_filted_passed_station_od_otd_list:
                    if temp_item.count("-") == 1 and temp_item.split("-")[1] not in station_dict[busline]:
                        filted_passed_station_od_otd_list.remove(temp_item)
                    if temp_item.count("-") == 2 and temp_item.split("-")[0] in passed_station_list and \
                            temp_item.split("-")[1] in passed_station_list:
                        filted_passed_station_od_otd_list.remove(temp_item)
                bus_arrstation_od_otd[busline][station].append(filted_passed_station_od_otd_list)
            bus_arrstation_od_otd[busline][station] = sum(bus_arrstation_od_otd[busline][station], [])
            bus_arrstation_od_otd[busline][station] = list(dict.fromkeys(bus_arrstation_od_otd[busline][station]))  # 进行元素去重
    return bus_arrstation_od_otd


def get_arr_rate(line_station_od_otd_dict):
    """获取OD、OTD乘客到达率 (LRG 20250226新增修改)"""
    arr_rate_dict = {}
    for busline in line_station_od_otd_dict:
        for station in line_station_od_otd_dict[busline]:
            if station not in arr_rate_dict:
                arr_rate_dict[station] = {}
            for od_otd in line_station_od_otd_dict[busline][station]:
                if od_otd.count("-") == 1 and od_otd.startswith(station):  # 表示在处理以该站为O的OD数据
                    arr_rate_dict[station][od_otd] = str(random.uniform(0.00035, 0.00055))   # 定义随机数规则 str(round(random.uniform(0.035, 0.055), 6))
                # else:
                #     arr_rate_dict[station][od_otd] = str(random.uniform(0.035, 0.055))  # 定义随机数规则 str(0.035)
                    # arr_rate_dict[station][od_otd] = str(0)  # 定义随机数规则
    return arr_rate_dict


def get_static_info(line_obj_dic):
    """获取公交线路、线路公交站等静态信息"""

    bus_line_list = list(line_obj_dic.keys())

    station_dict = {}
    station_interval_dis_dict = {}
    for busline in bus_line_list:
        station_dict[busline] = line_obj_dic[busline].stop_id_l
        station_interval_dis_dict[busline] = line_obj_dic[busline].distance_between_stop_d

    """初始化公交站、公交车OD、OTD及乘客到达率"""
    line_station_od_otd_dict = get_busline_station_od_otd_dict(station_dict)
    # 对line_station_od_otd_dict中的部分OD进行删减，从而降低Gurobi模型求解复杂度
    scaled_line_station_od_otd_dict = get_scaled_line_station_od_otd_dict(line_station_od_otd_dict)

    scaled_bus_arrstation_od_otd_dict = get_bus_arrstation_od_otd(scaled_line_station_od_otd_dict, station_dict)
    od_otd_arr_rate_dict = get_arr_rate(scaled_line_station_od_otd_dict)

    """常量定义"""
    BusCap = 50  # 公交乘客容量  50更符合实际一些  # 20(20S)、50(90S)
    AveAlightingTime = 1.2  # 每位乘客平均上车时间（s/per）
    AveBoardingTime = 1.5  # 每位乘客平均下车时间（s/per）

    return bus_line_list, station_dict, station_interval_dis_dict, scaled_line_station_od_otd_dict, line_station_od_otd_dict, \
           scaled_bus_arrstation_od_otd_dict, od_otd_arr_rate_dict, BusCap, AveAlightingTime, AveBoardingTime


def get_sorted_busline_edge(edge_file):
    """获取各公交线路的路段ID构成的字典"""
    involved_tl_ID_l = []
    busline_tl_time_d = {}  # 保存各线路要经过的信号灯及对应相位时长及信号周期时长
    sorted_busline_edge_d = {}
    stop_edge_d = {}  # 保存各线路的公交站对应的路段ID
    edge_root = edge_file.getroot()
    for busline_child in edge_root:
        if busline_child.tag == "busline" and busline_child.get("id") not in sorted_busline_edge_d:
            sorted_busline_edge_d[busline_child.get("id")] = {}
            stop_edge_d[busline_child.get("id")] = {}
            busline_tl_time_d[busline_child.get("id")] = {}

            for element_child in busline_child:
                if element_child.tag == "element" and element_child.get("id") not in sorted_busline_edge_d[busline_child.get("id")]:
                    if element_child.get("type") == "w":
                        sorted_busline_edge_d[busline_child.get("id")][element_child.get("id")] = [element_child.get("type"), element_child.get("length"),
                                                                                                   element_child.get("dedicated_flag"), element_child.get("impacted_edge"),
                                                                                                   element_child.get("impacted_length")]
                    elif element_child.get("type") == "s":
                        sorted_busline_edge_d[busline_child.get("id")][element_child.get("id")] = [element_child.get("type"), element_child.get("length"),
                                                                                                   element_child.get("dedicated_flag"), element_child.get("stop_id")]
                        stop_edge_d[busline_child.get("id")][element_child.get("stop_id")] = element_child.get("id")
                    else:
                        sorted_busline_edge_d[busline_child.get("id")][element_child.get("id")] = [element_child.get("type"), element_child.get("length"),
                                                                                                   element_child.get("dedicated_flag"), element_child.get("tl"),
                                                                                                   element_child.get("phase_time"), element_child.get("cycle_time"),
                                                                                                   element_child.get("from_lane"), element_child.get("dir")]
                        if element_child.get("tl"):  # 若终点站过后的路段是以交叉口结尾的则不算，因为行程已经结束
                            involved_tl_ID_l.append(element_child.get("tl"))
                            busline_tl_time_d[busline_child.get("id")][element_child.get("tl")] = [element_child.get("phase_time"), element_child.get("cycle_time")]
    return sorted_busline_edge_d, list(set(involved_tl_ID_l)), stop_edge_d, busline_tl_time_d


def get_timestep_tl_lane_traffic_flow(file):
    """读取lane_traffic_flow文件夹中生成的bus_edge_lane_traffic_flow.xml"""
    timestep_tl_lane_traffic_flow = {}
    root = file.getroot()
    for timestep_child in root:
        timestep_tl_lane_traffic_flow[timestep_child.get("id")] = {}
        for lane_child in timestep_child:
            timestep_tl_lane_traffic_flow[timestep_child.get("id")][lane_child.get("id")] = lane_child.get("trafficFlow")
    return timestep_tl_lane_traffic_flow

