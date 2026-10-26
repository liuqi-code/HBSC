from sim_obj import passenger
from sim_obj import signal
from sim_obj import lane
from sim_obj import stop
from sim_obj import line
from sim_obj import bus
import xml.etree.ElementTree as ET


def create_obj_fun():
    lane_obj_xml_path = "initialize_obj/save_obj_lane.add.xml"
    stop_obj_xml_path = "initialize_obj/save_obj_stop.add.xml"
    signal_obj_xml_path = "initialize_obj/save_obj_signal.add.xml"
    line_obj_xml_path = "initialize_obj/save_obj_line.add.xml"
    bus_obj_xml_path = "initialize_obj/save_obj_bus.add.xml"
    passenger_obj_xml_path = "initialize_obj/save_obj_passenger.add.xml"

    lane_file = ET.parse(lane_obj_xml_path)
    lane_root = lane_file.getroot()
    lane_obj_dic = {}
    for child in lane_root:
        if child.tag == "lane_obj":
            lane_obj_dic[child.get("lane_id_s")] = lane.Lane(child.get("lane_id_s"), float(child.get("length_n")),
                                                             child.get("from_junction_s"), child.get("to_junction_s"))

    stop_file = ET.parse(stop_obj_xml_path)
    stop_root = stop_file.getroot()
    stop_obj_dic = {}
    for child in stop_root:
        if child.tag == "stop_obj":
            stop_obj_dic[child.get("stop_id_s")] = stop.Stop(child.get("stop_id_s"), child.get("at_edge_s"),
                                                             child.get("at_lane_s"), child.get("service_line_l").split(" "))

    signal_file = ET.parse(signal_obj_xml_path)
    signal_root = signal_file.getroot()
    signal_obj_dic = {}
    for child in signal_root:
        if child.tag == "signal_obj":
            signal_id_s = child.get("signal_id_s")
            for sub_child in child:
                if sub_child.tag == "in_lane":
                    in_lane_d = sub_child.attrib
                    for in_land_id in in_lane_d.keys():
                        in_lane_d[in_land_id] = in_lane_d[in_land_id].split(" ")
                if sub_child.tag == "out_lane":
                    out_lane_d = sub_child.attrib
                    for out_land_id in out_lane_d.keys():
                        out_lane_d[out_land_id] = out_lane_d[out_land_id].split(" ")
                if sub_child.tag == "detector":
                    detector_d = sub_child.attrib
                if sub_child.tag == "connection":
                    connection_d = sub_child.attrib
                    new_connection_d = {}
                    for connection_id in connection_d.keys():
                        new_connection_d[connection_id[2:]] = connection_d[connection_id].split(" ")
                if sub_child.tag == "phase":
                    phase_d = sub_child.attrib
                    new_phase_d = {}
                    for phase_id in phase_d.keys():
                        new_phase_d[phase_id[2:]] = [n.split(";") for n in phase_d[phase_id].split(" ")]
            signal_obj_dic[signal_id_s] = signal.Signal(signal_id_s, in_lane_d, out_lane_d, detector_d, new_connection_d, new_phase_d)

    line_file = ET.parse(line_obj_xml_path)
    line_root = line_file.getroot()
    line_obj_dic = {}
    for child in line_root:
        if child.tag == "line_obj":
            line_id_s = child.get("line_id_s")
            edge_id_l = child.get("edge_id_l").split(" ")
            stop_id_l = child.get("stop_id_l").split(" ")
            signal_id_l = child.get("signal_id_l").split(" ")
            for sub_child in child:
                if sub_child.tag == "edge_length":
                    edge_length_d = sub_child.attrib
                    for edge_id in edge_length_d.keys():
                        edge_length_d[edge_id] = float(edge_length_d[edge_id])
                if sub_child.tag == "edge_average_speed":
                    edge_average_speed_d = sub_child.attrib
                    for edge_id in edge_average_speed_d.keys():
                        edge_average_speed_d[edge_id] = [float(n) for n in edge_average_speed_d[edge_id].split(" ")]
                if sub_child.tag == "distance_between_stop":
                    distance_between_stop_d = sub_child.attrib
                    new_distance_between_stop_d = {}
                    for stop_id in distance_between_stop_d.keys():
                        new_distance_between_stop_d[stop_id[2:]] = float(distance_between_stop_d[stop_id])
                if sub_child.tag == "distance_between_signal":
                    distance_between_signal_d = sub_child.attrib
                    for signal_id in distance_between_signal_d.keys():
                        distance_between_signal_d[signal_id] = float(distance_between_signal_d[signal_id])
                if sub_child.tag == "edge_between_stop":
                    edge_between_stop_d = sub_child.attrib
                    new_edge_between_stop_d = {}
                    for stop_id in edge_between_stop_d.keys():
                        new_edge_between_stop_d[stop_id[2:]] = edge_between_stop_d[stop_id].split(" ")
                if sub_child.tag == "stop_edge":
                    stop_edge_d = sub_child.attrib
                    new_stop_edge_d = {}
                    for stop_id in stop_edge_d.keys():
                        new_stop_edge_d[stop_id[2:]] = stop_edge_d[stop_id]
                if sub_child.tag == "signal_between_stop":
                    signal_between_stop_d = sub_child.attrib
                    new_signal_between_stop_d = {}
                    for stop_id in signal_between_stop_d.keys():
                        new_signal_between_stop_d[stop_id[2:]] = signal_between_stop_d[stop_id].split(" ")

                if sub_child.tag == "edge_between_signal":
                    edge_between_signal_d = sub_child.attrib
                    for signal_id in edge_between_signal_d.keys():
                        edge_between_signal_d[signal_id] = edge_between_signal_d[signal_id].split(" ")

            line_obj_dic[line_id_s] = line.Line(line_id_s, edge_id_l, stop_id_l, signal_id_l, edge_length_d,
                                                edge_average_speed_d, new_distance_between_stop_d, distance_between_signal_d,
                                                new_edge_between_stop_d, edge_between_signal_d, new_stop_edge_d, new_signal_between_stop_d)

    bus_file = ET.parse(bus_obj_xml_path)
    bus_root = bus_file.getroot()
    bus_obj_dic = {}
    for child in bus_root:
        if child.tag == "bus_obj":
            bus_obj_dic[child.get("bus_id_s")] = bus.Bus(child.get("bus_id_s"), float(child.get("start_time_n")),
                                                         child.get("belong_line_id_s"))

    passenger_file = ET.parse(passenger_obj_xml_path)
    passenger_root = passenger_file.getroot()
    passenger_obj_dic = {}
    for child in passenger_root:
        if child.tag == "passenger_obj":
            passenger_obj_dic[child.get("passenger_id_s")] = passenger.Passenger(child.get("passenger_id_s"),
                                                                                 float(child.get("start_time_n")),
                                                                                 child.get("start_stop_id_s"),
                                                                                 child.get("transfer_stop_id_s"),
                                                                                 child.get("end_stop_id_s"),
                                                                                 child.get("start_edge_id_s"),
                                                                                 child.get("transfer_edge_id_s"),
                                                                                 child.get("end_edge_id_s"))

    return lane_obj_dic, stop_obj_dic, signal_obj_dic, line_obj_dic, bus_obj_dic, passenger_obj_dic
