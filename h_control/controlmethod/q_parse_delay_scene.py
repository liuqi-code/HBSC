import xml.etree.ElementTree as ET
import ast


def parse_variable_speed_sign(xml_path):
    tree = ET.parse(xml_path)
    root = tree.getroot()
    sorted_lane_time_speedLimit_d = {}
    for vss in root.findall("variableSpeedSign"):
        lane_id = vss.get("lanes")
        time_speed = {}
        for step in vss.findall("step"):
            time_speed[int(step.get("time"))] = float(step.get("speed"))
        sorted_lane_time_speedLimit_d[lane_id] = time_speed
    return sorted_lane_time_speedLimit_d


def parse_delay_scene_stop_list(xml_path):
    tree = ET.parse(xml_path)
    root = tree.getroot()
    delay_scene_start_end_stopList_d = {}
    for idx, ds in enumerate(root.findall("delayScene"), start=1):
        stop_list = ast.literal_eval(ds.get("start2endStop"))
        start_time = int(ds.get("startTime"))
        end_time = int(ds.get("endTime"))
        delay_scene_start_end_stopList_d[idx] = [stop_list, start_time, end_time]
    return delay_scene_start_end_stopList_d


def load_all(vss_xml, stop_xml):
    sorted_lane_time_speedLimit_d = parse_variable_speed_sign(vss_xml)
    delay_scene_start_end_stopList_d = parse_delay_scene_stop_list(stop_xml)
    return sorted_lane_time_speedLimit_d, delay_scene_start_end_stopList_d


if __name__ == "__main__":
    vss_path = "/mnt/user-data/uploads/d_delay_scene_add.xml"
    stop_path = "/mnt/user-data/uploads/delay_scene_start_end_stopList.xml"

    sorted_lane_time_speedLimit_d, delay_scene_start_end_stopList_d = load_all(vss_path, stop_path)

    print("sorted_lane_time_speedLimit_d:")
    for lane, ts in sorted_lane_time_speedLimit_d.items():
        print(f"  {lane}: {ts}")

    print("\ndelay_scene_start_end_stopList_d:")
    for k, v in delay_scene_start_end_stopList_d.items():
        print(f"  {k}: {v}")

