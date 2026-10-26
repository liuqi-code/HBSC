import numpy as np
import traci
import f_gurobi_model


# def get_bus_cur_pos(bus_obj_dic, ):
#     """获取模型触发时刻，仿真系统中各公交车的位置信息"""
#     cur_bus_edge_pos_d = {}  # 触发时刻各公交车所处路段及剩余距离
#     # 触发时刻更新公交车的后面要经过的信号灯的相关信息
#     vehicle_id_list_2 = traci.vehicle.getIDList()
#     for vehicle_id_2 in vehicle_id_list_2:
#         if traci.vehicle.getTypeID(vehicle_id_2) == "Bus":
#             bus_obj_dic[vehicle_id_2].update_subsequent_signal_info(simulation_current_time,
#                                                                     signal_obj_dic, busline_tl_time_d)
#             # 触发时刻获取各公交车所处路段及剩余距离
#             cur_lane_id = traci.vehicle.getLaneID(vehicle_id_2)
#             # 表示公交车当前不是处于连接部分
#             if not cur_lane_id.startswith(":"):
#                 cur_edge_remain_length = traci.lane.getLength(cur_lane_id) - traci.vehicle.getLanePosition(vehicle_id_2)
#                 cur_bus_edge_pos_d[vehicle_id_2] = [cur_lane_id[:-2], cur_edge_remain_length]
#             else:
#                 # 如果公交车此刻正好处于连接部分，则需要往前找到非连接的路段，并设置剩余路段长度为0
#                 bus_served_lane_l = bus_obj_dic[vehicle_id_2].bus_served_lane_l
#                 temp_index = len(bus_served_lane_l) - 1
#                 while temp_index:
#                     if not bus_served_lane_l[temp_index].startswith(":"):
#                         temp_lane_id = bus_served_lane_l[temp_index]
#                         cur_bus_edge_pos_d[vehicle_id_2] = [temp_lane_id[:-2], 0]
#                         break
#                     else:
#                         temp_index -= 1
#     return cur_bus_edge_pos_d


def get_bus_sequence_on_stops_every_second(bus_obj_dic, stop_obj_dic, station_interval_dis_dict, scaled_line_station_od_otd_dict):
    """计算当前各车到达各车站的距离，从而确定各站公交车的到达顺序"""

    first_stop_num = 100
    simplified_bus_obj_dic, simplified_stop_obj_dic = f_gurobi_model.Simplified_bus_and_stop_dict(bus_obj_dic, stop_obj_dic,
                                                                                                  first_stop_num,
                                                                                                  scaled_line_station_od_otd_dict)

    bus_obj_l, stop_obj_l, bus_obj_id_l, stop_obj_id_l = f_gurobi_model.get_bus_stop_obj_l(simplified_bus_obj_dic, simplified_stop_obj_dic)

    # 获取各车到达下一站的公交站ID及距离
    bus_next_station_distance_l = {}
    for bus_obj in bus_obj_l:
        bus_next_station_distance_l[bus_obj.bus_id_s] = [bus_obj.next_stop_id_s, int(bus_obj.next_stop_length_n)]

    # 计算各站未服务的公交车到达该站的距离
    unserved_bus_to_station_distance_d = {}
    for stop_obj in stop_obj_l:
        unserved_bus_to_station_distance_d[stop_obj.stop_id_s] = {}
        unserved_bus_list = stop_obj.unserved_bus_l
        for unserved_bus in unserved_bus_list:
            unserved_bus_to_station_distance_d[stop_obj.stop_id_s][unserved_bus] = bus_next_station_distance_l[unserved_bus][1]  # unserved_bus到达下一站的距离
            if stop_obj.stop_id_s != bus_next_station_distance_l[unserved_bus][0]:  # 表示unserved_bus的下一站不是stop_obj站
                unserved_busline = unserved_bus.split("_")[0]
                unserved_busline_stations_l = list(station_interval_dis_dict[unserved_busline].keys())
                index_O = unserved_busline_stations_l.index(bus_next_station_distance_l[unserved_bus][0])
                index_D = unserved_busline_stations_l.index(stop_obj.stop_id_s)
                unserved_complete_interval_l = unserved_busline_stations_l[index_O+1: index_D+1]  # unserved_bus到达stop_obj还需要经过的完整的站间列表
                for unserved_complete_interval in unserved_complete_interval_l:
                    unserved_bus_to_station_distance_d[stop_obj.stop_id_s][unserved_bus] += station_interval_dis_dict[unserved_busline][unserved_complete_interval]

    # 确定各站的公交车行车顺序
    bus_sequence_on_stops_d = {}
    for stop_id in unserved_bus_to_station_distance_d:
        unserved_bus_list = {}
        for unserved_bus in list(unserved_bus_to_station_distance_d[stop_id].keys()):
            if stop_id in list(station_interval_dis_dict[unserved_bus.split("_")[0]].keys())[:-1]:
                unserved_bus_list[unserved_bus] = unserved_bus_to_station_distance_d[stop_id][unserved_bus]

        bus_sequence_on_stops_d[stop_id] = sorted(unserved_bus_list, key=lambda k: unserved_bus_list[k])

    return bus_sequence_on_stops_d


def check_bus_order(station_dict):
    """
    检查字典中每个站点的公交车列表是否按线路位次从小到大排列

    参数:
        station_dict: 字典，键为公交站标号，值为公交车标号列表
                     公交车标号格式: "线路标号_位次"，如 "122S_1"

    返回:
        bool: 如果所有站点的所有线路都按位次排序则返回True，否则返回False
        dict: 详细信息，包含每个站点的检查结果
    """
    all_valid = True
    details = {}

    for station, bus_list in station_dict.items():
        # 按线路分组
        route_buses = {}
        for bus in bus_list:
            route, position = bus.split('_')
            position = int(position)
            if route not in route_buses:
                route_buses[route] = []
            route_buses[route].append(position)

        # 检查每条线路的位次是否递增
        station_valid = True
        invalid_routes = []

        for route, positions in route_buses.items():
            # 检查是否严格递增
            is_sorted = all(positions[i] < positions[i + 1] for i in range(len(positions) - 1))
            if not is_sorted:
                station_valid = False
                invalid_routes.append(f"{route}: {positions}")

        details[station] = {
            'valid': station_valid,
            'invalid_routes': invalid_routes
        }

        if not station_valid:
            all_valid = False

    return all_valid


