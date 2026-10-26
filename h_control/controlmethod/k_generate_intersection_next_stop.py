"""
功能：解析公交线路XML文件，生成字典变量
    键 = 公交线路ID
    值 = 该公交线路依次经过的每个交叉口路段(type="i")的上游最近公交站标号(stop_id)组成的列表

逻辑说明：
    1) 找到 type="i" 交叉口路段后，向上游查找最近的 type="s" 公交站路段，记录其 stop_id
    2) 若向上游查找时先遇到另一个 type="i"（两交叉口之间无公交站），则跳过该交叉口
    3) 每条线路的最后一个公交站为终点站，其之后的所有路段（含交叉口）一律忽略
"""

import xml.etree.ElementTree as ET
import json
import sys
import os


def parse_busline_xml(xml_path: str) -> dict:
    tree = ET.parse(xml_path)
    root = tree.getroot()

    result = {}

    for busline in root.findall('busline'):
        busline_id = busline.get('id')
        elements = list(busline.findall('element'))

        # ---- 找首个公交站(首站)的stop_id ----
        first_stop_id = None
        for elem in elements:
            if elem.get('type') == 's':
                first_stop_id = elem.get('stop_id')
                break

        # ---- 找最后一个公交站(终点站)的索引和stop_id ----
        last_stop_idx = -1
        last_stop_id = None
        for i in range(len(elements) - 1, -1, -1):
            if elements[i].get('type') == 's':
                last_stop_idx = i
                last_stop_id = elements[i].get('stop_id')
                break

        if last_stop_idx == -1 or first_stop_id is None:
            result[busline_id] = []
            continue

        # 只处理终点站及之前的element
        upstream_stop_list = []

        for idx in range(last_stop_idx + 1):
            elem = elements[idx]
            if elem.get('type') == 'i':
                found_stop_id = None
                for j in range(idx + 1, last_stop_idx + 1):
                    j_type = elements[j].get('type')
                    if j_type == 's':
                        found_stop_id = elements[j].get('stop_id')
                        break
                    elif j_type == 'i':
                        break

                if found_stop_id is not None:
                    # ---- 第4点：跳过首站和终点站 ----
                    if found_stop_id == first_stop_id:
                        continue
                    if found_stop_id == last_stop_id:
                        continue
                    upstream_stop_list.append(found_stop_id)

        result[busline_id] = upstream_stop_list

    return result


import re
import json
import os


def extract_line_id(stop_component: str) -> str:
    """
    从公交站分量中提取公交线路ID。
    例如: "102X17" -> "102X", "7S26" -> "7S", "705S36" -> "705S"
    规则: 数字+字母 为线路ID，其后的数字为站点序号
    """
    match = re.match(r'^(\d+[A-Za-z]+)(\d+)$', stop_component)
    if match:
        return match.group(1)
    return None


def get_union_stops(busline_dict: dict, d: dict) -> list:
    """
    从字典d中提取涉及的公交线路，返回这些线路在busline_dict中值列表的并集。

    Parameters
    ----------
    busline_dict : dict
        键为公交线路ID，值为该线路交叉口上游公交站标号列表
    d : dict
        形如 {1: [['122S14_311S20', '102X17_122S19_311S25_406X17'], 5160, 7516]}
        其中值列表的第一个元素是公交站标号字符串列表

    Returns
    -------
    list
        涉及的所有公交线路在busline_dict中的公交站标号的并集（去重，保持顺序）
    """
    # ---- 第1步：从d中提取所有公交站标号字符串 ----
    all_stop_id_strings = []
    for key, value in d.items():
        # value的第一个元素是公交站标号列表
        stop_id_list = value[0]
        all_stop_id_strings.extend(stop_id_list)

    # ---- 第2步：拆分每个stop_id，提取涉及的公交线路 ----
    involved_lines = set()
    for stop_id_str in all_stop_id_strings:
        components = stop_id_str.split('_')
        for comp in components:
            line_id = extract_line_id(comp)
            if line_id:
                involved_lines.add(line_id)

    print(f"涉及的公交站标号: {all_stop_id_strings}")
    print(f"提取到的公交线路: {sorted(involved_lines)}")

    # ---- 第3步：取这些线路在busline_dict中值列表的并集 ----
    union_stops = []
    seen = set()
    for line_id in sorted(involved_lines):
        if line_id not in busline_dict:
            print(f"  ⚠ 线路 {line_id} 不在busline_dict中，跳过")
            continue
        for stop in busline_dict[line_id]:
            if stop not in seen:
                seen.add(stop)
                union_stops.append(stop)

    print(f"并集公交站数量: {len(union_stops)}")

    return union_stops


if __name__ == '__main__':
    # 1. 生成交叉口后的最近公交站的xml文件
    xml_path = r"D:\LRG_Cur_Code\4_ML_SUMO\4_ML_SUMO\m_low_control\b_sorted_busline_edge.xml"
    busline_dict = parse_busline_xml(xml_path)

    # 2. 生成晚点场景涉及线路的公交站标号列表
    d = {1: [['122S14_311S20', '102X17_122S19_311S25_406X17'], 5160, 7516]}
    result = get_union_stops(busline_dict, d)

    debug = 0


