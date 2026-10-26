import xml.etree.ElementTree as ET


def get_bus_schedule():
    """基于无控制下的公交车运行轨迹获取公交车计划时刻表"""
    busSchedule_add = "../h_no_control/output_file/BusSchedule.xml"
    busSchedule_file = ET.parse(busSchedule_add)
    busSchedule_root = busSchedule_file.getroot()

    bus_schedule = {}
    for busline_child in busSchedule_root:
        if busline_child.get("id") not in bus_schedule:
            bus_schedule[busline_child.get("id")] = {}
        for bus_child in busline_child:
            bus_schedule[busline_child.get("id")][bus_child.get("id")] = {}
            for stop_child in bus_child:
                bus_schedule[busline_child.get("id")][bus_child.get("id")][stop_child.get("id")] = \
                    [int(float(stop_child.get("arriveTime"))),
                     int(float(stop_child.get("departTime")))]
            # 如果某个车还没来得及到第一个公交站，那么不计入计划时刻表
            if not bus_schedule[busline_child.get("id")][bus_child.get("id")]:
                del bus_schedule[busline_child.get("id")][bus_child.get("id")]
    return bus_schedule

