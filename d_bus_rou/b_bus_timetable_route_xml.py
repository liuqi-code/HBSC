import openpyxl
import pandas as pd
from xml.dom import minidom
from subprocess import Popen
from xml.etree.ElementTree import Element, SubElement, tostring


# 读取Excel文件
timetable_excel = openpyxl.load_workbook("../a_sim_input_data/bus_timetable.xlsx")
timetable_data = []
for sheet_name in timetable_excel.sheetnames:
    sheet_data = timetable_excel[sheet_name]
    for row in sheet_data.iter_rows(values_only=True):
        # 提取发车序号和时间
        departure_number, departure_time = row
        # 将时间转换为字符串并拆分
        departure_time_str = departure_time.strftime("%H:%M:%S")
        hours, minutes, seconds = map(int, departure_time_str.split(":"))
        seconds_since_six = (hours - 6) * 3600 + minutes * 60 + seconds
        # 将每辆车的信息储存在列表内
        timetable_data.append([sheet_name, departure_number, seconds_since_six])
timetable_data = sorted(timetable_data, key=lambda x: x[2])

# 读取 Excel 文件
excel_data = pd.read_excel("../a_sim_input_data/new_station.xlsx", sheet_name="new_station")

# 创建根节点
routes = Element("routes")

# 遍历每个时刻表数据
for data in timetable_data:
    # 创建 flow 元素
    trip = SubElement(routes, "trip", {"id": f"{data[0]}_{data[1]}", "type": "Bus", "depart": str(data[2])})

    # 筛选相应线路的站点数据并复制数据以避免警告
    line_data = excel_data[excel_data["BUS_STOP_ID"].str.contains(data[0])].copy()

    # 定义正则表达式来匹配BUS_STOP_ID中包含line_id的部分，并提取后两位数字
    pattern = f"{data[0]}(\d+)"
    line_data["sort_value"] = line_data["BUS_STOP_ID"].str.extract(pattern).astype(int)

    # 按照BUS_STOP_ID中_分割后包含line_id的字符串的后两位数字排序
    sorted_line_data = line_data.sort_values(by="sort_value")

    # 获取第一个站和最后一个站的 LANE_ID
    first_lane_id = sorted_line_data["LANE_ID"].iloc[0][:-2]
    last_lane_id = sorted_line_data["NEW_EDGE_NAME"].iloc[-1].split(";")[-2]

    # 添加 from 和 to 属性到 flow 元素
    trip.set("from", first_lane_id)
    trip.set("to", last_lane_id)

    # 遍历排序后的站点数据，添加到 flow 元素中
    for index, row in sorted_line_data.iterrows():
        # 创建 stop 元素
        stop = SubElement(trip, "stop", {"busStop": row["BUS_STOP_ID"], "duration": "0", "line":data[0]})

# 格式化 XML 输出
xml_str = minidom.parseString(tostring(routes)).toprettyxml(indent="    ")

# 将 XML 写入文件
with open("2_bus_timetable_trips.xml", "w") as f:
    f.write(xml_str)

print("公交时刻表发车 XML 文件已生成。")

duarouter_command = Popen("II_bus_time_route.bat")
stdout_1, stderr_1 = duarouter_command.communicate()
