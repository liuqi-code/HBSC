import xml.etree.ElementTree as ET
import re
import os
import copy


"""基于edge ID判断其属于哪一种元素，输出元素类型"""
def get_edge_type(edge_id, root):
    edge_type = "None"  # 初始化edge元素类型
    for child in root:
        if child.tag == "edge" and child.get("id") == edge_id:
            to_node = child.get("to")
            break

    for child in root:
        if child.tag == "junction" and child.get("id") == to_node:
            if child.get("type") == "traffic_light":  # 表明该edge元素以交叉口结尾
                edge_type = "i"
                break
        if child.tag == "edge" and child.get("id") == edge_id:
            if child[0].get("allow") == "pedestrian":
                edge_type = "s"
                break
    if edge_type == "None":  # 表明该edge元素以公交站或者非信号灯交叉口结尾
        edge_type = "w"
    return edge_type


"""判断edge1是否通过左转进入edge2"""
def judge_left_turn(edge1, list, index_edge1, root):
    left_turn_flag = 0
    for child in root:  # 找到edge1最左边车道索引
        if child.tag == "edge" and child.get("id") == edge1:
            left_lane_id = child[-1].get("index")
            if index_edge1 != len(list)-1:  # 不是公交车经过的最后一个路段
                for child_con in root:  # 判断edge1进入edge2是否经最左边车道
                    if child_con.tag == "connection" and child_con.get("from") == edge1 and child_con.get("to") == list[index_edge1+1]:
                        if child.get("fromLane") == left_lane_id:  # 表示edge1经过最左边车道进入edge2
                            left_turn_flag = 1

            if child[0].get("allow") == "bus":  # 判断该路段是否布设了公交专用道
                return str(1 - left_turn_flag)
            else:
                return str(0)


"""转换为xml数据格式"""
def change_edge_data_style_xml(root, data_d):
    for busline in data_d:
        line = ET.SubElement(root, "busline")
        line.set("id", busline)
        line.text = "\n\t\t"
        line.tail = "\n\t"
        for element in data_d[busline]:
            ele = ET.SubElement(line, "element")
            ele.set("id", element)
            ele.set("type", data_d[busline][element][0])
            ele.set("length", data_d[busline][element][1])
            ele.set("dedicated_flag", data_d[busline][element][2])
            if data_d[busline][element][0] == "i" and len(data_d[busline][element]) > 3:  # 若线路最后一个edge为以交叉口结尾，则其没有tl、phase_time等属性
                ele.set("tl", data_d[busline][element][3])
                ele.set("phase_time", str(data_d[busline][element][4]))
                ele.set("cycle_time", str(data_d[busline][element][5]))
                ele.set("from_lane", data_d[busline][element][6])
                ele.set("dir", data_d[busline][element][7])
            if data_d[busline][element][0] == "w" and len(data_d[busline][element]) > 3:  # 若线路第一个edge为以公交站结尾，则其没有impacted_edge等属性
                ele.set("impacted_edge", data_d[busline][element][3])
                ele.set("impacted_length", str(data_d[busline][element][4]))
            if data_d[busline][element][0] == "s":
                if len(data_d[busline][element]) < 4:
                    nouse = 0
                ele.set("stop_id", data_d[busline][element][3])
            ele.tail = "\n\t\t"
    return root


"""转换信控交叉口的信号配时数据为xml格式"""
def change_tl_data_style_xml(root, tl_data_d):
    for busline in tl_data_d:
        line = ET.SubElement(root, "busline")
        line.set("id", busline)
        line.text = "\n\t\t"
        line.tail = "\n\t"
        for tl in tl_data_d[busline]:
            trafficl = ET.SubElement(line, "tlLogic")
            trafficl.set("id", tl)
            trafficl.text = "\n\t\t\t"
            trafficl.tail = "\n\t\t"
            index = 0
            while index < len(tl_data_d[busline][tl])-1:
                ele = ET.SubElement(trafficl, "phase")
                ele.set("duration", tl_data_d[busline][tl][index])
                ele.set("state", tl_data_d[busline][tl][index+1])
                ele.tail = "\n\t\t\t"
                index += 2
    return root


"""获取from_edge与to_edge之间一个连接的linkIndex"""
def get_connection_linkindex(root, from_edge, to_edge):
    for child in root:
        if child.tag == "connection" and child.get("from") == from_edge and child.get("to") == to_edge:
            return child.get("linkIndex"), child.get("tl"), child.get("from")+"_"+child.get("fromLane"), child.get("dir")


"""获取tl信号灯的linkIndex连接的绿灯时长及周期时长"""
def get_phase_and_cycle_time(root, linkIndex, tl):
    c_time = 0
    g_time = 0
    for child in root:
        if child.tag == "tlLogic" and child.get("id") == tl:
            for subchild in child:
                c_time += int(subchild.get("duration"))
                if subchild.get("state")[int(linkIndex)] in ["g", "G", "y"]:
                    g_time += int(subchild.get("duration"))
            return g_time, c_time


"""对于交叉口前多个w类型路段的话，若w路段没有布设专用道，则在这些w路段运行时，就会考虑交叉口延时，故其受影响的路段一直到i路段，距离也累加至i路段"""
def get_merge_info(merged_sorted_busline_edge_d, sorted_busline_edge_d, busline, element):
    element_list = list(sorted_busline_edge_d[busline].keys())
    element_index = element_list.index(element)
    temp_index = element_index

    merged_tostop_element_value = float(sorted_busline_edge_d[busline][element][1])
    over_onstop_edge_flag = 0

    while temp_index:  # 遍历该edge之前所有元素
        if sorted_busline_edge_d[busline][element_list[temp_index - 1]][0] == "i":  # 若前一个edge为以交叉口结尾，则退出
            break
        if sorted_busline_edge_d[busline][element_list[temp_index - 1]][0] == "w":  # 若前一个为以公交站结尾的路段，则添加两个属性，“该路段可一直合并至的路段元素，合并之后的等效距离”
            if over_onstop_edge_flag == 0:
                if sorted_busline_edge_d[busline][element_list[temp_index - 1]][2] == "0":
                    merged_tostop_element_value += float(sorted_busline_edge_d[busline][element_list[temp_index - 1]][1])
                    merged_sorted_busline_edge_d[busline][element_list[temp_index - 1]].append(element)
                    merged_sorted_busline_edge_d[busline][element_list[temp_index - 1]].append(str(merged_tostop_element_value))
                else:
                    merged_sorted_busline_edge_d[busline][element_list[temp_index - 1]].append(element_list[temp_index - 1])
                    merged_sorted_busline_edge_d[busline][element_list[temp_index - 1]].append(merged_sorted_busline_edge_d[busline][element_list[temp_index - 1]][1])
            else:
                merged_sorted_busline_edge_d[busline][element_list[temp_index - 1]].append(element_list[temp_index - 1])
                merged_sorted_busline_edge_d[busline][element_list[temp_index - 1]].append(merged_sorted_busline_edge_d[busline][element_list[temp_index - 1]][1])

        if sorted_busline_edge_d[busline][element_list[temp_index - 1]][0] == "s":  # 公交站所在路段
            over_onstop_edge_flag = 1

        temp_index -= 1
    return merged_sorted_busline_edge_d


"""处理路网中每个w类型路段，查询其可影响至后面哪个路段，即合并至哪个路段"""
def get_merge_info_2(merged_sorted_busline_edge_d, sorted_busline_edge_d, busline, element, index):

    later_elements_l = list(sorted_busline_edge_d[busline].keys())[index+1:]  # 该element后面的所有edge元素
    impacted_edge_id = element
    impacted_edge_length = float(sorted_busline_edge_d[busline][element][1])

    for later_element in later_elements_l:
        # 如果当前edge没有布设公交专用道，而且后面是i类型路段元素，则影响至交叉口
        if sorted_busline_edge_d[busline][later_element][0] == "i" and sorted_busline_edge_d[busline][element][2] == "0":
            impacted_edge_id = later_element
            impacted_edge_length += float(sorted_busline_edge_d[busline][later_element][1])
            break
        # 后面是w类型路段，而且与当前edge路段的专用道属性一致，则影响至该w类型路段结束
        elif sorted_busline_edge_d[busline][later_element][0] == "w" and sorted_busline_edge_d[busline][element][2] == sorted_busline_edge_d[busline][later_element][2]:
            impacted_edge_id = later_element
            impacted_edge_length += float(sorted_busline_edge_d[busline][later_element][1])
        else:
            break

    merged_sorted_busline_edge_d[busline][element].append(impacted_edge_id)
    merged_sorted_busline_edge_d[busline][element].append(str(impacted_edge_length))

    return merged_sorted_busline_edge_d


def get_busline_sorted_edge_l(bus_timetable_add, modified_line_edge_file):
    """获取各线路的排序的路段列表，并写入文件"""
    bus_timetable_file = ET.parse(bus_timetable_add)
    bus_timetable_root = bus_timetable_file.getroot()

    modified_line_edge = {}
    for child in bus_timetable_root:
        if child.tag == "vehicle" and child.get("id").split("_")[0] not in modified_line_edge:
            modified_line_edge[child.get("id").split("_")[0]] = child[0].get("edges")

    # 转换为树结构
    root = ET.Element("data")
    root.text = "\n\t"
    for busline in modified_line_edge:
        route = ET.SubElement(root, "route")
        route.set("L_id", busline)
        route.set("edges", modified_line_edge[busline])
        route.tail = "\n\t"

    # 写入xml文件
    if os.path.exists(modified_line_edge_file):
        os.remove(modified_line_edge_file)
    root_tree = ET.ElementTree(root)
    root_tree.write(modified_line_edge_file, encoding="utf-8", xml_declaration=True)


def get_busline_elements():

    """读取公交时刻表文件，生成线路的排序路段文件"""
    bus_timetable_add = "../d_bus_rou/2_bus_timetable.rou.xml"
    busline_sorted_edge_save_add = "b_busline_sorted_edge.xml"
    get_busline_sorted_edge_l(bus_timetable_add, busline_sorted_edge_save_add)

    """读取公交线路的排序路段文件和路网文件"""
    sorted_busline_element_file = ET.parse(busline_sorted_edge_save_add)
    root_sorted_busline_element = sorted_busline_element_file.getroot()
    network_file = ET.parse("../b_network/1_changsha_bus_network_with_signal_f.net.xml")
    root_network = network_file.getroot()

    """获取各线路已经按照行车方向排列的元素集合"""
    sorted_busline_edge_d = {}  # {线路ID：{ele1:[类型,长度，专用道flag], ele2,...},...}
    for child in root_sorted_busline_element:
        sorted_busline_edge_d[child.get("L_id")] = {}
        child_edge_split_l = child.get("edges").split()
        for index, temp_edge in enumerate(child_edge_split_l):  # 遍历该公交线路的每个edge元素，在路网文件中获取对应的edge长度和公交专用道信息
            for child_net in root_network:
                if child_net.tag == "edge" and child_net.get("id") == temp_edge:
                    sorted_busline_edge_d[child.get("L_id")][temp_edge] = [get_edge_type(temp_edge, root_network),
                                                                           child_net[0].get("length"),
                                                                           judge_left_turn(temp_edge,
                                                                                           child_edge_split_l, index,
                                                                                           root_network)]
                    break

    """处理以交叉口结尾的路段元素"""
    """对各以信控交叉口结尾的路段，补充所需要相位的时长以及信号周期时长"""
    for busline in sorted_busline_edge_d:
        element_id_l = list(sorted_busline_edge_d[busline].keys())
        for index, element in enumerate(element_id_l):
            if index < len(element_id_l) - 1 and sorted_busline_edge_d[busline][element][0] == "i":
                linkIndex, tl, from_lane, dir = get_connection_linkindex(root_network, element, element_id_l[index+1])  # 获取element与element_id_l[index+1]之间某一个连接的linkindex
                phase_time, cycle_time = get_phase_and_cycle_time(root_network, linkIndex, tl)  # 获取tl信号灯的linkIndex连接的绿灯时长及周期时长
                sorted_busline_edge_d[busline][element].append(tl)
                # sorted_busline_edge_d[busline][element].append(linkIndex)
                sorted_busline_edge_d[busline][element].append(str(phase_time))
                sorted_busline_edge_d[busline][element].append(str(cycle_time))
                sorted_busline_edge_d[busline][element].append(str(from_lane))
                sorted_busline_edge_d[busline][element].append(str(dir))

    """处理以公交站结尾的路段元素"""
    merged_sorted_busline_edge_d = copy.deepcopy(sorted_busline_edge_d)
    for busline in merged_sorted_busline_edge_d:
        element_id_l = list(sorted_busline_edge_d[busline].keys())
        for index, element in enumerate(element_id_l):
            if sorted_busline_edge_d[busline][element][0] == "w":
                merged_sorted_busline_edge_d = get_merge_info_2(merged_sorted_busline_edge_d, sorted_busline_edge_d, busline, element, index)

    # """如果以交叉口结尾的路段的前一个路段为w类型路段，并且w类型路段对于该线路公交车来说是没有布设公交专用道的，则w路段添加额外属性"""
    # merged_sorted_busline_edge_d = copy.deepcopy(sorted_busline_edge_d)
    # for busline in sorted_busline_edge_d:
    #     element_id_l = list(sorted_busline_edge_d[busline].keys())
    #     for index, element in enumerate(element_id_l):
    #         # if busline == "122X" and element == "E_J210_J190":
    #         if index > 0 and sorted_busline_edge_d[busline][element][0] == "i":
    #             merged_sorted_busline_edge_d = get_merge_info(merged_sorted_busline_edge_d, sorted_busline_edge_d, busline, element)
    # """对于后面没有i路段的w路段来说，其影响距离就是其本身"""
    # for busline in merged_sorted_busline_edge_d:
    #     for element in merged_sorted_busline_edge_d[busline]:
    #         if merged_sorted_busline_edge_d[busline][element][0] == "w" and len(merged_sorted_busline_edge_d[busline][element]) == 3:
    #             merged_sorted_busline_edge_d[busline][element].append(element)
    #             merged_sorted_busline_edge_d[busline][element].append(merged_sorted_busline_edge_d[busline][element][1])

    """处理处于公交站的路段元素"""
    bus_station_file = ET.parse("../b_network/3_bus_station.add.xml")
    bus_station_root = bus_station_file.getroot()
    for busline in merged_sorted_busline_edge_d:
        for element in merged_sorted_busline_edge_d[busline]:
            if merged_sorted_busline_edge_d[busline][element][0] == "s":
                # 查找element对应的公交站ID
                find_flag = 0
                for child_stop in bus_station_root:
                    if child_stop.get("lane") == element + "_1":
                        merged_sorted_busline_edge_d[busline][element].append(child_stop.get("id"))
                        find_flag = 1
                        break
                if find_flag == 0:
                    print(f"{busline}的{element}没有找到公交站ID！")

    """获取各线路按照行车方向经过的各类路段元素集合、信号灯集合"""
    sorted_busline_totl_edge_d = {}  # 排序好的以交叉口结尾的路段元素集合  {线路ID：{totl1:["i",长度，专用道flag], totl2,...},...}
    sorted_busline_onstop_edge_d = {}  # 排序好的公交站所在路段元素集合  {线路ID：{onstop1:["s",长度，专用道flag], onstop2,...},...}
    sorted_busline_tostop_edge_d = {}  # 排序好的以公交站结尾的路段元素集合  {线路ID：{tostop1:["w",长度，专用道flag], tostop2,...},...}
    sorted_busline_trafficlight_d = {}  # 排序好的交叉口ID集合{线路ID：{tl 1:[phase 1,...], tl 2:[phase 1,...],...}}
    for busline in merged_sorted_busline_edge_d:
        sorted_busline_trafficlight_d[busline] = {}
        sorted_busline_onstop_edge_d[busline] = {}
        sorted_busline_totl_edge_d[busline] = {}
        sorted_busline_tostop_edge_d[busline] = {}
        for element in merged_sorted_busline_edge_d[busline]:

            # 获取以交叉口结尾的路段元素及信控交叉口集合
            if merged_sorted_busline_edge_d[busline][element][0] == "i":
                to_node = re.findall(r'J\d+', element)[-1]
                for child in root_network:
                    if child.tag == "junction" and child.get("id") == to_node:
                        if child.get("type") == "traffic_light":
                            sorted_busline_trafficlight_d[busline][to_node] = []
                            sorted_busline_totl_edge_d[busline][element] = merged_sorted_busline_edge_d[busline][element]
                        break

            else:
                # 获取公交站所在的路段元素
                for child in root_network:
                    if child.tag == "edge" and child.get("id") == element:
                        if child[0].get("allow") == "pedestrian":
                            sorted_busline_onstop_edge_d[busline][element] = merged_sorted_busline_edge_d[busline][element]
                        else:
                            # 获取以公交站结尾的路段元素
                            sorted_busline_tostop_edge_d[busline][element] = merged_sorted_busline_edge_d[busline][element]
                        break

    # 获取各信控交叉口的信号配时
    for busline in sorted_busline_trafficlight_d:
        for tl in sorted_busline_trafficlight_d[busline]:  # 遍历各公交线路的每个信控交叉口
            for child in root_network:
                if child.tag == "tlLogic" and child.get("id") == tl:
                    for sub_child in child:
                        sorted_busline_trafficlight_d[busline][tl].append(sub_child.get("duration"))
                        sorted_busline_trafficlight_d[busline][tl].append(sub_child.get("state"))
                    break

    """保存至xml文件"""
    root_final_edge = ET.Element("data")
    root_final_edge.text = "\n\t"
    root_final_edge = change_edge_data_style_xml(root_final_edge, merged_sorted_busline_edge_d)
    final_edge_file = "b_sorted_busline_edge.xml"
    if os.path.exists(final_edge_file):
        os.remove(final_edge_file)
    final_edge_xml_tree = ET.ElementTree(root_final_edge)
    final_edge_xml_tree.write(final_edge_file, encoding="utf-8", xml_declaration=True)

    root_totl_edge = ET.Element("data")
    root_totl_edge.text = "\n\t"
    root_totl_edge = change_edge_data_style_xml(root_totl_edge, sorted_busline_totl_edge_d)
    totl_edge_file = "b_sorted_busline_totl_edge.xml"
    if os.path.exists(totl_edge_file):
        os.remove(totl_edge_file)
    totl_edge_xml_tree = ET.ElementTree(root_totl_edge)
    totl_edge_xml_tree.write(totl_edge_file, encoding="utf-8", xml_declaration=True)

    root_onstop_edge = ET.Element("data")
    root_onstop_edge.text = "\n\t"
    root_onstop_edge = change_edge_data_style_xml(root_onstop_edge, sorted_busline_onstop_edge_d)
    onstop_edge_file = "b_sorted_busline_onstop_edge.xml"
    if os.path.exists(onstop_edge_file):
        os.remove(onstop_edge_file)
    onstop_edge_xml_tree = ET.ElementTree(root_onstop_edge)
    onstop_edge_xml_tree.write(onstop_edge_file, encoding="utf-8", xml_declaration=True)

    root_tostop_edge = ET.Element("data")
    root_tostop_edge.text = "\n\t"
    root_tostop_edge = change_edge_data_style_xml(root_tostop_edge, sorted_busline_tostop_edge_d)
    tostop_edge_file = "b_sorted_busline_tostop_edge.xml"
    if os.path.exists(tostop_edge_file):
        os.remove(tostop_edge_file)
    tostop_edge_xml_tree = ET.ElementTree(root_tostop_edge)
    tostop_edge_xml_tree.write(tostop_edge_file, encoding="utf-8", xml_declaration=True)

    root_trafficlight = ET.Element("data")
    root_trafficlight.text = "\n\t"
    root_trafficlight = change_tl_data_style_xml(root_trafficlight, sorted_busline_trafficlight_d)
    trafficlight_file = "b_sorted_busline_trafficlight.xml"
    if os.path.exists(trafficlight_file):
        os.remove(trafficlight_file)
    trafficlight_xml_tree = ET.ElementTree(root_trafficlight)
    trafficlight_xml_tree.write(trafficlight_file, encoding="utf-8", xml_declaration=True)
