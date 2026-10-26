import xml.etree.ElementTree as ET
import math
import os


# 读取路网文件获取指定路段的车道数量
network_file = ET.parse("../b_network/1_changsha_bus_network_with_signal_e.net.xml")
network_root = network_file.getroot()


def get_lane_num_for_specified_edge(root, specified_edge_id):
    """读取路网文件获取指定路段的车道数量"""
    for child in root:
        if child.tag == "edge" and child.get("id") == specified_edge_id and child.get("function") != "internal":
            return int(child[-1].get("index"))+1

    print(f"ERROR for {specified_edge_id}")
    return 0


# 读取a_sorted_busline_edge文件获取各公交线路经过的车道
busline_lane_file = ET.parse("b_sorted_busline_edge.xml")
busline_lane_root = busline_lane_file.getroot()

"""获取所有公交线路经过的以交叉口结束的路段的相关信息"""
busline_edge_info_d = {}
for busline_child in busline_lane_root:
    busline_edge_info_d[busline_child.get("id")] = {}
    for edge_child in busline_child:
        if edge_child.get("type") == "i" and edge_child.get("cycle_time"):
            busline_edge_info_d[busline_child.get("id")][edge_child.get("from_lane")[:-2]] = [
                "i",
                float(edge_child.get("length")),
                float(edge_child.get("cycle_time")) - float(edge_child.get("phase_time")),
                edge_child.get("dir")]
        # 非交叉口路段也保存交通流速度
        if edge_child.get("type") != "i":
            # # 方式1：
            # for temp_lane_num in range(0, get_lane_num_for_specified_edge(network_root, edge_child.get("id"))):
            #     temp_lane_id = edge_child.get("id") + "_" + str(temp_lane_num)
            #     busline_edge_info_d[busline_child.get("id")][temp_lane_id] = [edge_child.get("type"),
            #                                                                    edge_child.get("length")]

            # 方式2：不同车道的交通流速度相差不大，所以就用最右侧车道的交通流速度
            busline_edge_info_d[busline_child.get("id")][edge_child.get("id")] = [edge_child.get("type"), edge_child.get("length")]


bus_edge_lane_traffic_flow_d = {}
root_folder = "../g_pre_sim/output_file/DividedPreSimFcd/"
for subdir, dirs, files in os .walk(root_folder):
    for file in files:
        if file .endswith(".xml"):
            file_path = os .path .join(subdir, file)

            # 读取FCD数据文件
            FCD_file = ET.parse(file_path)
            FCD_root = FCD_file.getroot()

            """获取所需要车道不同时刻的交通流速度"""
            time_period = 600
            period_lane_traffic_flow = {}
            for timestep_child in FCD_root:
                time = float(timestep_child.get("time"))  # 获取当前时间
                # 以100s为周期记录车道的交通流速度
                if time > 1 and (time % time_period == 0 or time > 17900):
                    # # 首先，对保存的该周期内的速度数据进行补全处理，即若某些时间片段下没有某些路段的数据，则以13.98m/s补全至时间数量
                    # for lane_id in period_lane_traffic_flow:
                    #     if len(period_lane_traffic_flow[lane_id]) < time_period:
                    #         period_lane_traffic_flow[lane_id].extend([13.98]*(time_period-len(period_lane_traffic_flow[lane_id])))
                    # 其次，计算交通流速度均值
                    if time > 17900:
                        time = 18001
                    bus_edge_lane_traffic_flow_d[math.floor(time / time_period)] = {}
                    # 查询各公交线路需要经过的各车道在该周期内的车辆速度
                    for busline_key in busline_edge_info_d:
                        for edge_key in busline_edge_info_d[busline_key]:
                            # if busline_edge_info_d[busline_key][edge_key][0] != "i":
                            #     edge_key += "_0"
                            if edge_key in list(period_lane_traffic_flow.keys()):
                                bus_edge_lane_traffic_flow_d[math.floor(time/time_period)][edge_key] = sum(period_lane_traffic_flow[edge_key])/len(period_lane_traffic_flow[edge_key])
                            else:  # 如果当前周期内lane_key路段没有车辆运行，则赋值为上个周期该路段的速度
                                # if math.floor(time/time_period) == 1:
                                bus_edge_lane_traffic_flow_d[math.floor(time / time_period)][edge_key] = 13.98
                                # else:
                                #     bus_edge_lane_traffic_flow_d[math.floor(time/time_period)][lane_key] = bus_edge_lane_traffic_flow_d[math.floor(time/time_period)-1][lane_key]
                    # 该周期数据处理结束之后，清空，以进行下一轮周期
                    period_lane_traffic_flow = {}
                    continue
                else:
                    # 保存当前周期内的有车辆经过的车道及其车速值
                    for vehicle_child in timestep_child:
                        if vehicle_child.tag == "vehicle":
                            if vehicle_child.get("lane") not in period_lane_traffic_flow:
                                period_lane_traffic_flow[vehicle_child.get("lane")[:-2]] = [float(vehicle_child.get("speed"))]
                            else:
                                period_lane_traffic_flow[vehicle_child.get("lane")[:-2]].append(float(vehicle_child.get("speed")))

            # 释放内存
            FCD_root.clear()
            del FCD_root


"""保存为xml格式并写入xml文件"""
bus_edge_lane_traffic_flow_root = ET.Element("additional")
bus_edge_lane_traffic_flow_root.text = "\n\t"
for timestep_key in bus_edge_lane_traffic_flow_d:
    timestep = ET.SubElement(bus_edge_lane_traffic_flow_root, "timestep")
    timestep.set("id", str(timestep_key))
    timestep.text = "\n\t\t"

    for lane_id in bus_edge_lane_traffic_flow_d[timestep_key]:
        lane = ET.SubElement(timestep, "lane")
        lane.set("id", lane_id)
        lane.set("trafficFlow", str(bus_edge_lane_traffic_flow_d[timestep_key][lane_id]))
        lane.tail = "\n\t\t"

    timestep.tail = "\n\t"


bus_edge_lane_traffic_flow_file = f"../g_pre_sim/output_file/bus_edge_lane_traffic_flow.xml"
if os.path.exists(bus_edge_lane_traffic_flow_file):
    os.remove(bus_edge_lane_traffic_flow_file)
bus_edge_lane_traffic_flow_tree = ET.ElementTree(bus_edge_lane_traffic_flow_root)
bus_edge_lane_traffic_flow_tree.write(bus_edge_lane_traffic_flow_file, encoding="utf-8", xml_declaration=True)

debug = 0

