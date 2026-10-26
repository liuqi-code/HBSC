import torch
import pickle
import numpy as np
import torch.nn as nn
from pathlib import Path
import copy
import a_construct_dataset
import torch.nn.functional as F
import xml.etree.ElementTree as ET
from collections import OrderedDict
from collections import defaultdict
from torch.utils.data import Dataset, DataLoader as TorchDataLoader
from torch_geometric.nn import GATv2Conv, Linear
from torch_geometric.data import HeteroData, Batch
from torch_geometric.loader import DataLoader as PyGDataLoader
from i_up_control_update import g_get_schedule_timetable


BUSLINE_DISTANCE_BETWEEN_STOPS = {}


class DynamicScheduleManager:


    THRESHOLD_CHANGE = 30

    def __init__(self, original_bus_schedule, decision_timetable):

        self.original_bus_schedule = original_bus_schedule
        self.decision_timetable = decision_timetable


        self.sorted_times = self._get_sorted_times()


        self.affected_decisions = self._precompute_affected_decisions()


        self._cache = {}


        self._print_precompute_statistics()

    def _get_sorted_times(self):

        times = []
        for time_str in self.decision_timetable.keys():
            try:
                times.append(float(time_str))
            except ValueError:
                continue
        return sorted(times)

    def _time_to_str(self, time_value):

        return str(float(time_value))

    def _get_decision_at_time(self, time_value):


        possible_keys = [
            str(float(time_value)),
            str(int(float(time_value))),
            str(time_value),
        ]
        for key in possible_keys:
            if key in self.decision_timetable:
                return self.decision_timetable[key]
        return None

    def _precompute_affected_decisions(self):

        affected_decisions = {}


        cumulative_affected = {}

        print(f"\n{'=' * 60}")
        print(f"预计算'受影响'决策（阈值={self.THRESHOLD_CHANGE}秒）")
        print(f"{'=' * 60}")

        for time_idx, time_value in enumerate(self.sorted_times):

            decisions = self._get_decision_at_time(time_value)
            if decisions is None:
                affected_decisions[time_value] = {}
                continue


            current_dynamic_schedule = self._build_dynamic_schedule_from_affected(cumulative_affected)


            affected_at_this_time = {}
            total_decisions = 0
            affected_count = 0

            for bus_id, stop_decisions in decisions.items():
                line_id = bus_id.rsplit('_', 1)[0]

                for stop_id, decision_times in stop_decisions.items():
                    total_decisions += 1

                    if not isinstance(decision_times, (list, tuple)) or len(decision_times) < 2:
                        continue

                    decision_arrival = decision_times[0]


                    scheduled_arrival = self._get_scheduled_arrival(
                        current_dynamic_schedule, line_id, bus_id, stop_id
                    )

                    if scheduled_arrival is None:

                        scheduled_arrival = self._get_scheduled_arrival(
                            self.original_bus_schedule, line_id, bus_id, stop_id
                        )

                    if scheduled_arrival is None:

                        continue


                    deviation = abs(decision_arrival - scheduled_arrival)

                    if deviation > self.THRESHOLD_CHANGE:

                        affected_count += 1

                        if bus_id not in affected_at_this_time:
                            affected_at_this_time[bus_id] = {}
                        affected_at_this_time[bus_id][stop_id] = [
                            decision_times[0],
                            decision_times[1]
                        ]


                        if bus_id not in cumulative_affected:
                            cumulative_affected[bus_id] = {}
                        cumulative_affected[bus_id][stop_id] = [
                            decision_times[0],
                            decision_times[1]
                        ]


            affected_decisions[time_value] = affected_at_this_time


            if total_decisions > 0:
                print(f"  时刻 {time_value}: 总决策数={total_decisions}, "
                      f"受影响数={affected_count} ({affected_count / total_decisions * 100:.1f}%)")

        print(f"{'=' * 60}\n")

        return affected_decisions

    def _build_dynamic_schedule_from_affected(self, cumulative_affected):


        dynamic_schedule = copy.deepcopy(self.original_bus_schedule)


        for bus_id, stop_decisions in cumulative_affected.items():
            line_id = bus_id.rsplit('_', 1)[0]


            if line_id not in dynamic_schedule:
                dynamic_schedule[line_id] = {}
            if bus_id not in dynamic_schedule[line_id]:
                dynamic_schedule[line_id][bus_id] = {}


            for stop_id, times in stop_decisions.items():
                dynamic_schedule[line_id][bus_id][stop_id] = [times[0], times[1]]

        return dynamic_schedule

    def _get_scheduled_arrival(self, schedule, line_id, bus_id, stop_id):

        try:
            return schedule[line_id][bus_id][stop_id][0]
        except (KeyError, TypeError, IndexError):
            return None

    def _print_precompute_statistics(self):

        total_affected_buses = set()
        total_affected_stops = 0

        for time_value, affected in self.affected_decisions.items():
            for bus_id, stops in affected.items():
                total_affected_buses.add(bus_id)
                total_affected_stops += len(stops)

        print(f"预计算完成统计:")
        print(f"  总时刻数: {len(self.sorted_times)}")
        print(f"  有'受影响'决策的时刻数: {sum(1 for a in self.affected_decisions.values() if a)}")
        print(f"  涉及的公交车数: {len(total_affected_buses)}")
        print(f"  总'受影响'的公交车-站点组合数: {total_affected_stops}")
        print()

    def get_dynamic_schedule(self, current_time):

        current_time_float = float(current_time)


        if current_time_float in self._cache:
            return self._cache[current_time_float]


        dynamic_schedule = copy.deepcopy(self.original_bus_schedule)


        past_times = [t for t in self.sorted_times if t < current_time_float]


        for past_time in past_times:

            affected_at_past_time = self.affected_decisions.get(past_time, {})

            if not affected_at_past_time:
                continue


            for bus_id, stop_decisions in affected_at_past_time.items():

                line_id = bus_id.rsplit('_', 1)[0]


                if line_id not in dynamic_schedule:
                    dynamic_schedule[line_id] = {}
                if bus_id not in dynamic_schedule[line_id]:
                    dynamic_schedule[line_id][bus_id] = {}


                for stop_id, times in stop_decisions.items():
                    if isinstance(times, (list, tuple)) and len(times) >= 2:

                        dynamic_schedule[line_id][bus_id][stop_id] = [
                            times[0],
                            times[1]
                        ]


        self._cache[current_time_float] = dynamic_schedule

        return dynamic_schedule

    def get_earliest_time(self):

        if self.sorted_times:
            return self.sorted_times[0]
        return None

    def is_earliest_time(self, current_time):

        earliest = self.get_earliest_time()
        if earliest is None:
            return True
        return float(current_time) <= earliest

    def clear_cache(self):

        self._cache.clear()

    def get_affected_info_at_time(self, time_value):

        time_float = float(time_value)
        return self.affected_decisions.get(time_float, {})

    def is_bus_stop_affected(self, time_value, bus_id, stop_id):

        time_float = float(time_value)
        affected = self.affected_decisions.get(time_float, {})

        if bus_id in affected and stop_id in affected[bus_id]:
            return True
        return False

    def get_schedule_info_for_debug(self, current_time, bus_id, stop_id):

        line_id = bus_id.rsplit('_', 1)[0]


        original_time = None
        try:
            original_time = self.original_bus_schedule[line_id][bus_id][stop_id]
        except KeyError:
            pass


        dynamic_schedule = self.get_dynamic_schedule(current_time)
        dynamic_time = None
        try:
            dynamic_time = dynamic_schedule[line_id][bus_id][stop_id]
        except KeyError:
            pass


        update_source = None
        current_time_float = float(current_time)
        past_times = [t for t in self.sorted_times if t < current_time_float]

        for past_time in reversed(past_times):
            affected = self.affected_decisions.get(past_time, {})
            if bus_id in affected and stop_id in affected[bus_id]:
                update_source = past_time
                break

        return {
            'current_time': current_time,
            'bus_id': bus_id,
            'stop_id': stop_id,
            'original_schedule': original_time,
            'dynamic_schedule': dynamic_time,
            'update_source_time': update_source,
            'is_updated': original_time != dynamic_time if original_time and dynamic_time else False,
            'is_affected_update': update_source is not None
        }


def normalize_time_key(time_value):

    if isinstance(time_value, str):
        if '.' in time_value:
            return time_value
        else:
            return time_value + '.0'
    else:
        return str(float(time_value))


def get_time_value_from_dict(time_dict, time_moment):

    possible_keys = [
        str(float(time_moment)),
        str(int(float(time_moment))),
        str(time_moment),
    ]
    for key in possible_keys:
        if key in time_dict:
            return time_dict[key]
    return None


class BusDelayDataProcessor:


    def __init__(self):
        self.bus_id_to_idx = {}
        self.stop_id_to_idx = {}
        self.line_id_to_idx = {}


        self.max_speed = 15.0
        self.max_passengers = 80.0
        self.max_delay = 300.0
        self.max_waiting_passengers = 50.0


        self.max_bus_to_station_distance = 1000.0
        self.max_historical_speed = 50.0
        self.max_speed_limit = 15.0
        self.max_speed_diff = 30.0
        self.max_station_distance = 2000.0


        self.max_headway = 600.0
        self.ideal_headway = 300.0
        self.max_headway_deviation = 300.0

    def build_hetero_graph(self, time_moment, all_time_bus_info_d, all_time_stop_info_d,
                           all_time_busline_to_stop_flow_speed_d, all_time_atStop_passenger_num_d,
                           bus_schedule, busline_distance_between_stops, schedule_manager=None):

        buses_info = get_time_value_from_dict(all_time_bus_info_d, time_moment) or {}
        stops_info = get_time_value_from_dict(all_time_stop_info_d, time_moment) or {}
        flow_speed_info = get_time_value_from_dict(all_time_busline_to_stop_flow_speed_d, time_moment) or {}
        passenger_info = get_time_value_from_dict(all_time_atStop_passenger_num_d, time_moment) or {}


        if schedule_manager is not None:
            current_bus_schedule = schedule_manager.get_dynamic_schedule(time_moment)
        else:
            current_bus_schedule = bus_schedule

        data = HeteroData()
        self._build_node_mappings(buses_info, stops_info)


        bus_features = self._build_bus_features(buses_info, current_bus_schedule)
        data['bus'].x = torch.FloatTensor(bus_features)

        stop_features = self._build_stop_features(stops_info, passenger_info, current_bus_schedule)
        data['station'].x = torch.FloatTensor(stop_features)


        bus_to_station_edges, bus_to_station_attrs = self._build_bus_to_station_edges(buses_info, flow_speed_info)
        data['bus', 'approaching', 'station'].edge_index = torch.LongTensor(bus_to_station_edges)
        data['bus', 'approaching', 'station'].edge_attr = torch.FloatTensor(bus_to_station_attrs)


        bus_to_bus_edges, bus_to_bus_attrs = self._build_bus_to_bus_edges(
            buses_info,
            K=1
        )
        data['bus', 'headway', 'bus'].edge_index = torch.LongTensor(bus_to_bus_edges)
        data['bus', 'headway', 'bus'].edge_attr = torch.FloatTensor(bus_to_bus_attrs)


        station_to_station_edges, station_to_station_attrs = self._build_station_to_station_edges(
            busline_distance_between_stops)
        data['station', 'connects', 'station'].edge_index = torch.LongTensor(station_to_station_edges)
        data['station', 'connects', 'station'].edge_attr = torch.FloatTensor(station_to_station_attrs)

        return data

    def _build_node_mappings(self, buses_info, stops_info):

        bus_ids = sorted(buses_info.keys())
        self.bus_id_to_idx = {bus_id: idx for idx, bus_id in enumerate(bus_ids)}

        all_stop_ids = set(stops_info.keys())
        for bus_data in buses_info.values():
            if 'next_stop_id' in bus_data:
                all_stop_ids.add(bus_data['next_stop_id'])
            if 'just_server_stop_data' in bus_data and bus_data['just_server_stop_data']:
                all_stop_ids.add(bus_data['just_server_stop_data'][0])

        all_stop_ids = sorted(all_stop_ids)
        self.stop_id_to_idx = {stop_id: idx for idx, stop_id in enumerate(all_stop_ids)}

        line_ids = set()
        for bus_id in bus_ids:
            line_id = bus_id.rsplit('_', 1)[0]
            line_ids.add(line_id)
        self.line_id_to_idx = {line_id: idx for idx, line_id in enumerate(sorted(line_ids))}

    def compute_normalization_params(self, all_experiments, busline_distance_between_stops):

        print("正在计算归一化参数...")

        speeds, passengers, delays, waiting_passengers_list = [], [], [], []
        bus_to_station_distances, historical_speeds, speed_diffs = [], [], []


        actual_headways = []
        planned_headways = []

        for exp in all_experiments:

            schedule_manager = DynamicScheduleManager(
                exp['original_bus_schedule'],
                exp['all_time_bus_decision_timetable']
            )

            original_schedule = exp['original_bus_schedule']


            for line_id, buses in original_schedule.items():

                first_stop_times = []
                for bus_id, stops in buses.items():
                    if stops:

                        first_stop_id = min(stops.keys(), key=lambda s: stops[s][0])
                        first_stop_times.append((bus_id, stops[first_stop_id][1]))


                first_stop_times.sort(key=lambda x: x[1])


                for i in range(1, len(first_stop_times)):
                    interval = first_stop_times[i][1] - first_stop_times[i - 1][1]
                    if 60 < interval < 1800:
                        planned_headways.append(interval)

            for time_str, buses_info in exp['all_time_bus_info_d'].items():

                current_bus_schedule = schedule_manager.get_dynamic_schedule(time_str)

                line_speeds = defaultdict(list)


                line_arrival_times = defaultdict(list)

                for bus_id, bus_data in buses_info.items():
                    speeds.append(bus_data.get('cur_speed', 0.0))
                    passengers.append(bus_data.get('onboard_passenger_num', 0))

                    next_stop_distance = bus_data.get('next_stop_length', 0.0)
                    if next_stop_distance > 0:
                        bus_to_station_distances.append(next_stop_distance)

                    if 'just_server_stop_data' in bus_data and bus_data['just_server_stop_data']:
                        stop_id = bus_data['just_server_stop_data'][0]
                        actual_arrival = bus_data['just_server_stop_data'][1]
                        line_id = bus_id.rsplit('_', 1)[0]


                        line_arrival_times[line_id].append((bus_id, actual_arrival))

                        try:
                            scheduled_arrival = current_bus_schedule[line_id][bus_id][stop_id][0]
                            delays.append(abs(actual_arrival - scheduled_arrival))
                        except KeyError:
                            pass

                    line_id = bus_id.rsplit('_', 1)[0]
                    line_speeds[line_id].append(bus_data.get('cur_speed', 0.0))


                for line_id, speed_list in line_speeds.items():
                    if len(speed_list) >= 2:
                        for i in range(len(speed_list)):
                            for j in range(i + 1, len(speed_list)):
                                speed_diffs.append(abs(speed_list[i] - speed_list[j]))


                for line_id, arrival_list in line_arrival_times.items():
                    if len(arrival_list) >= 2:

                        arrival_list.sort(key=lambda x: x[1])

                        for i in range(1, len(arrival_list)):
                            headway = arrival_list[i][1] - arrival_list[i - 1][1]
                            if headway > 0:
                                actual_headways.append(headway)

            for time_str, passenger_info in exp['all_time_atStop_passenger_num_d'].items():
                waiting_passengers_list.extend(passenger_info.values())

            for time_str, flow_speed_info in exp['all_time_busline_to_stop_flow_speed_d'].items():
                for (line_id, stop_id), speed in flow_speed_info.items():
                    if speed > 0:
                        historical_speeds.append(speed)


        self.max_speed = max(speeds) * 1.1 if speeds else 50.0
        self.max_passengers = max(passengers) * 1.1 if passengers else 80.0
        self.max_delay = max(delays) * 1.1 if delays else 300.0
        self.max_waiting_passengers = max(waiting_passengers_list) * 1.1 if waiting_passengers_list else 50.0


        self.max_bus_to_station_distance = max(bus_to_station_distances) * 1.1 if bus_to_station_distances else 1000.0
        self.max_historical_speed = max(historical_speeds) * 1.1 if historical_speeds else 50.0
        self.max_speed_limit = self.max_speed
        self.max_speed_diff = max(speed_diffs) * 1.1 if speed_diffs else 30.0

        all_station_distances = []
        for line_id, stop_distances in busline_distance_between_stops.items():
            all_station_distances.extend(stop_distances.values())
        self.max_station_distance = max(all_station_distances) * 1.1 if all_station_distances else 2000.0


        if planned_headways:
            self.ideal_headway = np.median(planned_headways)
        else:
            self.ideal_headway = 300.0

        if actual_headways:
            self.max_headway = max(actual_headways) * 1.1

            headway_deviations = [abs(h - self.ideal_headway) for h in actual_headways]
            self.max_headway_deviation = max(headway_deviations) * 1.1 if headway_deviations else 300.0
        else:
            self.max_headway = 600.0
            self.max_headway_deviation = 300.0

        print(f"归一化参数计算完成：")
        print(f"\n【节点特征】最大速度: {self.max_speed:.2f}, 最大乘客: {self.max_passengers:.0f}, "
              f"最大延迟: {self.max_delay:.2f}, 最大等待: {self.max_waiting_passengers:.0f}")
        print(f"【边特征】车到站距离: {self.max_bus_to_station_distance:.2f}, 历史速度: {self.max_historical_speed:.2f}, "
              f"速度差: {self.max_speed_diff:.2f}, 站点距离: {self.max_station_distance:.2f}")
        print(f"【Headway】理想间隔: {self.ideal_headway:.2f}秒, 最大headway: {self.max_headway:.2f}秒, "
              f"最大偏差: {self.max_headway_deviation:.2f}秒")

    def _build_bus_features(self, buses_info, bus_schedule):

        features = []
        for bus_id in sorted(self.bus_id_to_idx.keys()):
            bus_data = buses_info[bus_id]
            current_speed = bus_data.get('cur_speed', 0.0)
            onboard_passengers = bus_data.get('onboard_passenger_num', 0)
            current_delay = 0.0
            if 'just_server_stop_data' in bus_data and bus_data['just_server_stop_data']:
                stop_id = bus_data['just_server_stop_data'][0]
                actual_arrival = bus_data['just_server_stop_data'][1]
                line_id = bus_id.rsplit('_', 1)[0]
                try:

                    scheduled_arrival = bus_schedule[line_id][bus_id][stop_id][0]
                    current_delay = abs(actual_arrival - scheduled_arrival)
                except KeyError:
                    current_delay = 0.0
            features.append([
                current_speed / self.max_speed,
                onboard_passengers / self.max_passengers,
                current_delay / self.max_delay,
            ])
        return features

    def _build_stop_features(self, stops_info, passenger_info, bus_schedule):

        features = []
        for stop_id in sorted(self.stop_id_to_idx.keys()):
            waiting_passengers = passenger_info.get(stop_id, 0)
            last_bus_delay = 0.0
            if stop_id in stops_info:
                stop_data = stops_info[stop_id]
                if len(stop_data) >= 3:
                    last_bus_id, line_id, actual_arrival = stop_data[0], stop_data[1], stop_data[2]
                    try:

                        scheduled_arrival = bus_schedule[line_id][last_bus_id][stop_id][0]
                        last_bus_delay = abs(actual_arrival - scheduled_arrival)
                    except KeyError:
                        pass
            features.append([
                waiting_passengers / self.max_waiting_passengers,
                last_bus_delay / self.max_delay,
            ])
        return features

    def _build_bus_to_station_edges(self, buses_info, flow_speed_info):

        edge_index, edge_attr = [[], []], []
        for bus_id, bus_data in buses_info.items():
            bus_idx = self.bus_id_to_idx[bus_id]
            next_stop_id = bus_data.get('next_stop_id')
            if next_stop_id and next_stop_id in self.stop_id_to_idx:
                stop_idx = self.stop_id_to_idx[next_stop_id]
                distance = bus_data.get('next_stop_length', 0.0)
                current_speed = bus_data.get('cur_speed', 0.0)
                speed_limit = self.max_speed_limit
                edge_index[0].append(bus_idx)
                edge_index[1].append(stop_idx)
                edge_attr.append([
                    distance / self.max_bus_to_station_distance,
                    current_speed / self.max_speed,
                    speed_limit / self.max_speed_limit,
                ])
        if not edge_index[0]:
            edge_index = [[0], [0]]
            edge_attr = [[0.0, 0.0, 0.0]]
        return edge_index, edge_attr

    def _build_bus_to_bus_edges(self, buses_info, K=1):

        edge_index = [[], []]
        edge_attr = []


        line_buses = defaultdict(list)
        for bus_id in buses_info.keys():
            line_id = bus_id.rsplit('_', 1)[0]
            line_buses[line_id].append(bus_id)

        for line_id, bus_list in line_buses.items():
            if len(bus_list) < 2:
                continue


            bus_info_list = []
            for bus_id in bus_list:
                if bus_id not in self.bus_id_to_idx:
                    continue

                bus_data = buses_info[bus_id]
                bus_idx = self.bus_id_to_idx[bus_id]
                speed = bus_data.get('cur_speed', 0.0)
                next_stop_length = bus_data.get('next_stop_length', float('inf'))


                arrival_time = None
                has_valid_arrival = False
                if 'just_server_stop_data' in bus_data and bus_data['just_server_stop_data']:
                    just_server = bus_data['just_server_stop_data']
                    if len(just_server) >= 2:
                        arrival_time = just_server[1]
                        has_valid_arrival = True

                bus_info_list.append({
                    'bus_id': bus_id,
                    'bus_idx': bus_idx,
                    'speed': speed,
                    'arrival_time': arrival_time,
                    'has_valid_arrival': has_valid_arrival,
                    'next_stop_length': next_stop_length
                })

            if len(bus_info_list) < 2:
                continue


            def sort_key(info):
                if info['has_valid_arrival']:
                    return (0, info['arrival_time'])
                else:
                    return (1, info['next_stop_length'])

            sorted_buses = sorted(bus_info_list, key=sort_key)

            for i, current_info in enumerate(sorted_buses):
                current_idx = current_info['bus_idx']
                current_speed = current_info['speed']
                current_arrival = current_info['arrival_time']
                current_valid = current_info['has_valid_arrival']

                for k in range(1, K + 1):

                    if i - k >= 0:
                        prev_info = sorted_buses[i - k]
                        prev_idx = prev_info['bus_idx']
                        prev_speed = prev_info['speed']
                        prev_arrival = prev_info['arrival_time']
                        prev_valid = prev_info['has_valid_arrival']


                        speed_diff = (current_speed - prev_speed) / self.max_speed_diff


                        if current_valid and prev_valid:
                            actual_headway = current_arrival - prev_arrival
                            headway_dev = (actual_headway - self.ideal_headway) / self.max_headway_deviation
                        else:
                            headway_dev = 0.0

                        edge_index[0].append(current_idx)
                        edge_index[1].append(prev_idx)
                        edge_attr.append([speed_diff, headway_dev])


                    if i + k < len(sorted_buses):
                        next_info = sorted_buses[i + k]
                        next_idx = next_info['bus_idx']
                        next_speed = next_info['speed']
                        next_arrival = next_info['arrival_time']
                        next_valid = next_info['has_valid_arrival']

                        speed_diff = (current_speed - next_speed) / self.max_speed_diff

                        if current_valid and next_valid:
                            actual_headway = next_arrival - current_arrival
                            headway_dev = (actual_headway - self.ideal_headway) / self.max_headway_deviation
                        else:
                            headway_dev = 0.0

                        edge_index[0].append(current_idx)
                        edge_index[1].append(next_idx)
                        edge_attr.append([speed_diff, headway_dev])

        if not edge_index[0]:
            edge_index = [[0], [0]]
            edge_attr = [[0.0, 0.0]]

        return edge_index, edge_attr

    def _build_station_to_station_edges(self, busline_distance_between_stops):

        edge_index, edge_attr = [[], []], []
        for line_id, stop_distances in busline_distance_between_stops.items():
            for (stop1_id, stop2_id), distance in stop_distances.items():
                if stop1_id in self.stop_id_to_idx and stop2_id in self.stop_id_to_idx:
                    stop1_idx = self.stop_id_to_idx[stop1_id]
                    stop2_idx = self.stop_id_to_idx[stop2_id]
                    edge_index[0].extend([stop1_idx, stop2_idx])
                    edge_index[1].extend([stop2_idx, stop1_idx])
                    speed_limit = self.max_speed_limit
                    attrs = [distance / self.max_station_distance, speed_limit / self.max_speed_limit]
                    edge_attr.extend([attrs, attrs])
        if not edge_index[0]:
            edge_index = [[0], [0]]
            edge_attr = [[0.0, 0.0]]
        return edge_index, edge_attr


class HeteroGATLayer(nn.Module):


    def __init__(self, in_channels, out_channels, num_heads=4,
                 bus_bus_edge_dim=2,
                 station_station_edge_dim=2,
                 bus_station_edge_dim=3,
                 dropout=0.1):
        super().__init__()

        self.dropout = dropout


        self.bus_conv = GATv2Conv(
            in_channels,
            out_channels,
            heads=num_heads,
            concat=False,
            edge_dim=bus_bus_edge_dim,
            add_self_loops=False,
            dropout=dropout
        )


        self.station_conv = GATv2Conv(
            in_channels,
            out_channels,
            heads=num_heads,
            concat=False,
            edge_dim=station_station_edge_dim,
            add_self_loops=True,
            dropout=dropout
        )


        self.bus_to_station_linear = nn.Linear(in_channels, out_channels)


        self.edge_gate = nn.Sequential(
            nn.Linear(bus_station_edge_dim, 32),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(32, out_channels),
            nn.Sigmoid()
        )

        self.station_fusion = nn.Linear(out_channels * 2, out_channels)


        self.bus_norm = nn.LayerNorm(out_channels)
        self.station_norm = nn.LayerNorm(out_channels)


        self.bus_residual = nn.Linear(in_channels, out_channels) if in_channels != out_channels else nn.Identity()
        self.station_residual = nn.Linear(in_channels, out_channels) if in_channels != out_channels else nn.Identity()

    def forward(self, bus_x, station_x,
                bus_bus_edge_index, station_station_edge_index, bus_station_edge_index,
                bus_bus_edge_attr=None, station_station_edge_attr=None, bus_station_edge_attr=None,
                return_attention_weights=False):

        bus_bus_attn_weights = None


        bus_residual = self.bus_residual(bus_x)

        if bus_bus_edge_index.size(1) > 0:
            if return_attention_weights:
                bus_out, bus_bus_attn_weights = self.bus_conv(
                    bus_x, bus_bus_edge_index,
                    edge_attr=bus_bus_edge_attr,
                    return_attention_weights=True
                )
            else:
                bus_out = self.bus_conv(
                    bus_x, bus_bus_edge_index,
                    edge_attr=bus_bus_edge_attr
                )
            bus_out = self.bus_norm(bus_out + bus_residual)
        else:
            bus_out = self.bus_norm(bus_residual)


        station_residual = self.station_residual(station_x)

        if station_station_edge_index.size(1) > 0:
            station_self = self.station_conv(
                station_x, station_station_edge_index,
                edge_attr=station_station_edge_attr
            )
        else:
            station_self = station_residual


        if bus_station_edge_index.size(1) > 0:
            source_features = bus_x[bus_station_edge_index[0]]
            transformed = self.bus_to_station_linear(source_features)


            if bus_station_edge_attr is not None:
                gate = self.edge_gate(bus_station_edge_attr)
                transformed = transformed * (0.5 + 0.5 * gate)

            num_stations = station_x.size(0)
            station_from_bus = torch.zeros(num_stations, transformed.size(1), device=station_x.device)
            target_indices = bus_station_edge_index[1].unsqueeze(1).expand(-1, transformed.size(1))
            station_from_bus.scatter_add_(0, target_indices, transformed)

            edge_count = torch.zeros(num_stations, device=station_x.device)
            edge_count.scatter_add_(0, bus_station_edge_index[1],
                                    torch.ones(bus_station_edge_index.size(1), device=station_x.device))
            edge_count = edge_count.clamp(min=1).unsqueeze(1)
            station_from_bus = station_from_bus / edge_count

            station_combined = torch.cat([station_self, station_from_bus], dim=-1)
            station_out = self.station_fusion(station_combined)
        else:
            station_out = station_self

        station_out = self.station_norm(station_out + station_residual)

        if return_attention_weights:
            return bus_out, station_out, bus_bus_attn_weights
        return bus_out, station_out


class BusDelayPredictionGNN(nn.Module):


    def __init__(self, bus_feature_dim=3, stop_feature_dim=2, hidden_dim=64, num_gnn_layers=3,
                 bus_bus_edge_dim=2, station_station_edge_dim=2, bus_station_edge_dim=3,
                 dropout=0.15):
        super().__init__()
        self.hidden_dim = hidden_dim
        self.num_gnn_layers = num_gnn_layers
        self.dropout = dropout


        self.bus_encoder = nn.Sequential(
            nn.Linear(bus_feature_dim, hidden_dim),
            nn.LayerNorm(hidden_dim),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(hidden_dim, hidden_dim)
        )
        self.station_encoder = nn.Sequential(
            nn.Linear(stop_feature_dim, hidden_dim),
            nn.LayerNorm(hidden_dim),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(hidden_dim, hidden_dim)
        )


        self.gnn_layers = nn.ModuleList([
            HeteroGATLayer(
                hidden_dim, hidden_dim, num_heads=4,
                bus_bus_edge_dim=bus_bus_edge_dim,
                station_station_edge_dim=station_station_edge_dim,
                bus_station_edge_dim=bus_station_edge_dim,
                dropout=dropout
            )
            for _ in range(num_gnn_layers)
        ])


        self.bus_skip = nn.Sequential(
            nn.Linear(hidden_dim * (num_gnn_layers + 1), hidden_dim),
            nn.LayerNorm(hidden_dim),
            nn.ReLU(),
            nn.Dropout(dropout)
        )
        self.station_skip = nn.Sequential(
            nn.Linear(hidden_dim * (num_gnn_layers + 1), hidden_dim),
            nn.LayerNorm(hidden_dim),
            nn.ReLU(),
            nn.Dropout(dropout)
        )


        self.stop_lstm = nn.LSTM(hidden_dim, hidden_dim, batch_first=True,
                                 dropout=dropout if num_gnn_layers > 1 else 0)


        self.cross_attention = nn.MultiheadAttention(hidden_dim, num_heads=4,
                                                     batch_first=True, dropout=dropout)


        self.classifier = nn.Sequential(
            nn.Linear(hidden_dim, hidden_dim),
            nn.LayerNorm(hidden_dim),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(hidden_dim, hidden_dim // 2),
            nn.ReLU(),
            nn.Dropout(dropout * 0.5),
            nn.Linear(hidden_dim // 2, 2)
        )

        self._init_weights()

    def _init_weights(self):
        for module in self.modules():
            if isinstance(module, nn.Linear):
                nn.init.xavier_uniform_(module.weight)
                if module.bias is not None:
                    nn.init.zeros_(module.bias)

    def forward(self, batch_data, target_bus_indices, future_stop_indices_list):

        bus_x = batch_data['bus'].x
        station_x = batch_data['station'].x

        bus_bus_edge_index = batch_data['bus', 'headway', 'bus'].edge_index
        station_station_edge_index = batch_data['station', 'connects', 'station'].edge_index
        bus_station_edge_index = batch_data['bus', 'approaching', 'station'].edge_index

        bus_bus_edge_attr = batch_data['bus', 'headway', 'bus'].edge_attr
        station_station_edge_attr = batch_data['station', 'connects', 'station'].edge_attr
        bus_station_edge_attr = batch_data['bus', 'approaching', 'station'].edge_attr

        bus_batch = batch_data['bus'].batch
        station_batch = batch_data['station'].batch

        bus_h = self.bus_encoder(bus_x)
        station_h = self.station_encoder(station_x)
        bus_h_list, station_h_list = [bus_h], [station_h]

        for gnn_layer in self.gnn_layers:
            bus_h, station_h = gnn_layer(
                bus_h, station_h,
                bus_bus_edge_index, station_station_edge_index, bus_station_edge_index,
                bus_bus_edge_attr=bus_bus_edge_attr,
                station_station_edge_attr=station_station_edge_attr,
                bus_station_edge_attr=bus_station_edge_attr
            )
            bus_h_list.append(bus_h)
            station_h_list.append(station_h)

        bus_h = self.bus_skip(torch.cat(bus_h_list, dim=-1))
        station_h = self.station_skip(torch.cat(station_h_list, dim=-1))

        batch_size = len(target_bus_indices)
        all_predictions = []

        for i in range(batch_size):
            mask_bus = bus_batch == i
            mask_station = station_batch == i
            bus_h_i = bus_h[mask_bus]
            station_h_i = station_h[mask_station]
            target_bus_idx = target_bus_indices[i]
            future_stop_indices = future_stop_indices_list[i]

            if target_bus_idx >= bus_h_i.size(0):
                target_bus_idx = 0
            valid_stop_indices = [idx for idx in future_stop_indices if idx < station_h_i.size(0)]
            if not valid_stop_indices:
                valid_stop_indices = [0]

            target_bus_emb = bus_h_i[target_bus_idx].unsqueeze(0)
            future_stops_emb = station_h_i[valid_stop_indices]
            stop_sequence = future_stops_emb.unsqueeze(0)

            lstm_out, _ = self.stop_lstm(stop_sequence)
            lstm_out = lstm_out.squeeze(0)

            attended, _ = self.cross_attention(
                query=lstm_out.unsqueeze(0),
                key=target_bus_emb.unsqueeze(0),
                value=target_bus_emb.unsqueeze(0)
            )
            attended = attended.squeeze(0)

            final_features = attended + lstm_out
            predictions = self.classifier(final_features)
            all_predictions.append(predictions)

        return all_predictions


class MultiExperimentDataManager:


    def __init__(self, base_dir='./experiments'):
        self.base_dir = Path(base_dir)
        self.experiments = []

    def load_experiment_from_xml(self, all_xml_add, experiment_id):

        decision_timetable = a_construct_dataset.get_busControlScheme(all_xml_add["GurobiInfo"])


        original_bus_schedule = g_get_schedule_timetable.get_bus_schedule()

        return {
            'experiment_id': experiment_id,
            'all_time_bus_info_d': a_construct_dataset.get_all_time_bus_info(all_xml_add["cur_bus"]),
            'all_time_stop_info_d': a_construct_dataset.get_all_time_stop_info(all_xml_add["cur_stop"]),
            'all_time_busline_to_stop_flow_speed_d': a_construct_dataset.get_all_time_busline_to_stop_flow_speed(
                all_xml_add["cur_busline_edge_speed"]),
            'all_time_bus_decision_timetable': decision_timetable,
            'all_time_atStop_passenger_num_d': a_construct_dataset.calculate_passenger_count(
                all_xml_add["passengerInfo"], list(decision_timetable.keys())),

            'original_bus_schedule': original_bus_schedule,

            'bus_schedule': original_bus_schedule,
        }

    def create_schedule_manager(self, experiment_data):

        return DynamicScheduleManager(
            experiment_data['original_bus_schedule'],
            experiment_data['all_time_bus_decision_timetable']
        )

    def add_experiment(self, experiment_data):


        schedule_manager = self.create_schedule_manager(experiment_data)


        AllTimeDelayBus_d = generate_delay_bus_dict_dynamic(
            experiment_data['all_time_bus_info_d'],
            schedule_manager,
            ThresholdDelayArrive=60
        )
        experiment_data['AllTimeDelayBus_d'] = AllTimeDelayBus_d
        self.experiments.append(experiment_data)
        print(f"添加实验 {experiment_data['experiment_id']}: {len(AllTimeDelayBus_d)} 个晚点场景")

    def load_all_experiments_from_directory(self, xml_dir):
        xml_dir = Path(xml_dir)
        experiment_folders = sorted(xml_dir.glob('output_file_*'), key=lambda p: int(p.name.split('_')[-1]))
        experiment_folders = [f for f in experiment_folders if f.is_dir()]
        print(f"发现 {len(experiment_folders)} 个实验文件夹")
        if len(experiment_folders) == 0:
            print(f"警告: 在 {xml_dir} 中未找到任何 output_file_* 文件夹")
            return
        for folder in experiment_folders:
            folder_number = folder.name.split('_')[-1]
            if int(folder_number) < 5:
                continue
            experiment_id = f"exp_{int(folder_number):03d}"
            print(f"\n{'=' * 60}\n加载实验 {experiment_id}: {folder.name}\n{'=' * 60}")
            try:
                all_xml_add = {
                    "cur_bus": str(folder) + "/cur_bus_info.xml",
                    "cur_stop": str(folder) + "/cur_stop_info.xml",
                    "cur_busline_edge_speed": str(folder) + "/cur_busline_edge_traffic_speed_info.xml",
                    "GurobiInfo": str(folder) + "/GurobimodelInfo.xml",
                    "passengerInfo": str(folder) + "/save_data_passenger.add.xml"
                }
                experiment_data = self.load_experiment_from_xml(all_xml_add, experiment_id)
                self.add_experiment(experiment_data)
            except Exception as e:
                print(f"错误: 加载实验 {experiment_id} 失败: {e}")
                import traceback
                traceback.print_exc()
        print(f"\n{'=' * 60}\n完成！成功加载 {len(self.experiments)} 个实验\n{'=' * 60}")

    def save_to_disk(self, save_path='experiments_data.pkl'):
        with open(save_path, 'wb') as f:
            pickle.dump(self.experiments, f)
        print(f"已保存 {len(self.experiments)} 个实验到 {save_path}")

    def load_from_disk(self, load_path='experiments_data.pkl'):
        with open(load_path, 'rb') as f:
            self.experiments = pickle.load(f)
        print(f"已加载 {len(self.experiments)} 个实验")

    def get_statistics(self):
        stats = {'num_experiments': len(self.experiments), 'total_delay_scenes': 0, 'total_buses': set(),
                 'total_stops': set(), 'total_time_moments': 0}
        for exp in self.experiments:
            stats['total_delay_scenes'] += len(exp['AllTimeDelayBus_d'])
            for time_buses in exp['all_time_bus_info_d'].values():
                stats['total_buses'].update(time_buses.keys())
            for time_stops in exp['all_time_stop_info_d'].values():
                stats['total_stops'].update(time_stops.keys())
            stats['total_time_moments'] += len(exp['all_time_bus_info_d'])
        stats['total_buses'] = len(stats['total_buses'])
        stats['total_stops'] = len(stats['total_stops'])
        return stats


def generate_delay_bus_dict_dynamic(all_time_bus_info_d, schedule_manager, ThresholdDelayArrive=60):

    AllTimeDelayBus_d = {}

    for time_str, buses_info in all_time_bus_info_d.items():

        time_key = str(int(float(time_str)))
        delay_buses = []


        current_bus_schedule = schedule_manager.get_dynamic_schedule(time_str)

        for bus_id, bus_data in buses_info.items():

            if 'just_server_stop_data' not in bus_data:
                continue

            just_server_stop_data = bus_data['just_server_stop_data']
            if not just_server_stop_data or len(just_server_stop_data) < 2:
                continue


            stop_id = just_server_stop_data[0]
            actual_arrival_time = just_server_stop_data[1]


            line_id = bus_id.rsplit('_', 1)[0]


            try:
                scheduled_time = current_bus_schedule[line_id][bus_id][stop_id][0]


                delay = actual_arrival_time - scheduled_time


                if delay > ThresholdDelayArrive:
                    delay_buses.append(bus_id)
                if delay < -120:
                    print(f"Bus {bus_id} arrives at {stop_id} early!\n")
            except KeyError:

                continue


        if delay_buses:
            AllTimeDelayBus_d[time_key] = delay_buses

    return AllTimeDelayBus_d


class BusDelayDatasetBuilder:


    def __init__(self, data_manager, processor):
        self.data_manager = data_manager
        self.processor = processor
        self.samples = []

    def generate_labels_for_bus(self, experiment_data, time_moment, bus_id, future_stops, schedule_manager):

        labels = []
        decision_timetable = experiment_data['all_time_bus_decision_timetable']


        current_bus_schedule = schedule_manager.get_dynamic_schedule(time_moment)

        bus_decisions = get_time_value_from_dict(decision_timetable, time_moment)
        if bus_decisions is None or bus_id not in bus_decisions:
            return [0] * len(future_stops)
        bus_decisions = bus_decisions[bus_id]
        line_id = bus_id.rsplit('_', 1)[0]


        THRESHOLD_CHANGE = DynamicScheduleManager.THRESHOLD_CHANGE

        for stop_id in future_stops:
            if stop_id not in bus_decisions:
                labels.append(0)
                continue
            decision_arrival = bus_decisions[stop_id][0]
            try:

                scheduled_arrival = current_bus_schedule[line_id][bus_id][stop_id][0]
                deviation = abs(decision_arrival - scheduled_arrival)
                labels.append(1 if deviation > THRESHOLD_CHANGE else 0)
            except KeyError:
                labels.append(0)
        return labels

    def build_dataset(self, max_future_stops=10, busline_distance_between_stops=None):

        print("开始构建训练数据集...")
        self.samples = []
        if busline_distance_between_stops is None:
            busline_distance_between_stops = BUSLINE_DISTANCE_BETWEEN_STOPS
        total_buses_processed, skipped_buses = 0, 0

        for exp_idx, experiment_data in enumerate(self.data_manager.experiments):
            experiment_id = experiment_data['experiment_id']
            all_time_bus_info_d = experiment_data['all_time_bus_info_d']
            print(f"\n处理实验 {experiment_id} ({exp_idx + 1}/{len(self.data_manager.experiments)})")


            schedule_manager = DynamicScheduleManager(
                experiment_data['original_bus_schedule'],
                experiment_data['all_time_bus_decision_timetable']
            )

            for time_str in all_time_bus_info_d.keys():
                time_moment = int(float(time_str))
                buses_info = get_time_value_from_dict(all_time_bus_info_d, time_moment)
                if not buses_info:
                    continue
                try:

                    graph_data = self.processor.build_hetero_graph(
                        time_moment,
                        experiment_data['all_time_bus_info_d'],
                        experiment_data['all_time_stop_info_d'],
                        experiment_data['all_time_busline_to_stop_flow_speed_d'],
                        experiment_data['all_time_atStop_passenger_num_d'],
                        experiment_data['original_bus_schedule'],
                        busline_distance_between_stops,
                        schedule_manager=schedule_manager
                    )
                except Exception as e:
                    print(f"  警告：时刻 {time_moment} 构图失败: {e}")
                    continue

                for bus_id in buses_info.keys():
                    total_buses_processed += 1
                    if bus_id not in self.processor.bus_id_to_idx:
                        skipped_buses += 1
                        continue
                    target_bus_idx = self.processor.bus_id_to_idx[bus_id]


                    future_stops = self._get_future_stops(
                        experiment_data, bus_id, time_moment, max_future_stops,
                        schedule_manager=schedule_manager
                    )
                    if len(future_stops) == 0:
                        skipped_buses += 1
                        continue
                    valid_future_stops = [stop_id for stop_id in future_stops if
                                          stop_id in self.processor.stop_id_to_idx]
                    if len(valid_future_stops) == 0:
                        skipped_buses += 1
                        continue
                    future_stop_indices = [self.processor.stop_id_to_idx[stop_id] for stop_id in valid_future_stops]


                    labels = self.generate_labels_for_bus(
                        experiment_data, time_moment, bus_id, valid_future_stops,
                        schedule_manager=schedule_manager
                    )
                    is_affected = any(label == 1 for label in labels)
                    sample = {
                        'experiment_id': experiment_id, 'time_moment': time_moment, 'bus_id': bus_id,
                        'graph_data': graph_data, 'target_bus_idx': target_bus_idx,
                        'future_stop_indices': future_stop_indices, 'future_stop_ids': valid_future_stops,
                        'labels': labels, 'is_affected': is_affected,
                    }
                    self.samples.append(sample)
            print(f"  当前累计样本数: {len(self.samples)}")
        self._print_statistics(total_buses_processed, skipped_buses)
        return self.samples

    def _print_statistics(self, total_buses_processed, skipped_buses):
        print(f"\n{'=' * 60}\n数据集构建完成\n{'=' * 60}")
        print(f"总处理公交车数: {total_buses_processed}, 成功生成样本数: {len(self.samples)}, 跳过的公交车数: {skipped_buses}")
        if len(self.samples) > 0:
            positive_samples = sum(1 for s in self.samples if s['is_affected'])
            print(
                f"正样本数: {positive_samples}, 负样本数: {len(self.samples) - positive_samples}, 正样本比例: {positive_samples / len(self.samples):.2%}")
            total_affected_stops = sum(sum(s['labels']) for s in self.samples)
            total_stops = sum(len(s['labels']) for s in self.samples)
            print(
                f"受影响站点总数: {total_affected_stops}, 所有站点总数: {total_stops}, 受影响站点比例: {total_affected_stops / total_stops:.2%}")

    def _get_future_stops(self, experiment_data, bus_id, time_moment, max_stops, schedule_manager=None):

        all_time_bus_info_d = experiment_data['all_time_bus_info_d']
        line_id = bus_id.rsplit('_', 1)[0]


        if schedule_manager is not None:
            current_bus_schedule = schedule_manager.get_dynamic_schedule(time_moment)
        else:
            current_bus_schedule = experiment_data['original_bus_schedule']

        try:
            bus_stops = current_bus_schedule[line_id][bus_id]
            all_stops_sorted = sorted(bus_stops.items(), key=lambda x: x[1][0])
            buses_info = get_time_value_from_dict(all_time_bus_info_d, time_moment)
            if buses_info and bus_id in buses_info:
                bus_data = buses_info[bus_id]
                if 'just_server_stop_data' in bus_data and bus_data['just_server_stop_data']:
                    last_served_stop = bus_data['just_server_stop_data'][0]
                    last_served_index = next(
                        (idx for idx, (stop_id, _) in enumerate(all_stops_sorted) if stop_id == last_served_stop), -1)
                    remaining_stops = [stop_id for stop_id, _ in
                                       all_stops_sorted[last_served_index + 1:]] if last_served_index >= 0 else [stop_id
                                                                                                                 for
                                                                                                                 stop_id, _
                                                                                                                 in
                                                                                                                 all_stops_sorted]
                else:
                    remaining_stops = [stop_id for stop_id, _ in all_stops_sorted]
            else:
                remaining_stops = [stop_id for stop_id, _ in all_stops_sorted]
            return remaining_stops[:min(len(remaining_stops), max_stops)]
        except KeyError:
            return []

    def save_dataset(self, save_path='dataset.pkl'):
        with open(save_path, 'wb') as f:
            pickle.dump(self.samples, f)
        print(f"数据集已保存到 {save_path}")

    def load_dataset(self, load_path='dataset.pkl'):
        with open(load_path, 'rb') as f:
            self.samples = pickle.load(f)
        print(f"已加载 {len(self.samples)} 个样本")


class BusDelayDataset(Dataset):

    def __init__(self, samples):
        self.samples = samples
    def __len__(self):
        return len(self.samples)
    def __getitem__(self, idx):
        return self.samples[idx]


def custom_collate_fn(batch):

    graphs = [sample['graph_data'] for sample in batch]
    batched_graph = Batch.from_data_list(graphs)
    target_bus_indices = [sample['target_bus_idx'] for sample in batch]
    future_stop_indices_list = [sample['future_stop_indices'] for sample in batch]
    labels_list = [sample['labels'] for sample in batch]
    return {'batched_graph': batched_graph, 'target_bus_indices': target_bus_indices,
            'future_stop_indices_list': future_stop_indices_list, 'labels_list': labels_list, 'samples': batch}


def create_dataloader(samples, batch_size=32, shuffle=True):

    dataset = BusDelayDataset(samples)

    return TorchDataLoader(dataset, batch_size=batch_size, shuffle=shuffle, collate_fn=custom_collate_fn)


class FocalLoss(nn.Module):


    def __init__(self, alpha=None, gamma=1.5, reduction='mean', label_smoothing=0.05):
        super(FocalLoss, self).__init__()
        self.gamma = gamma
        self.reduction = reduction
        self.label_smoothing = label_smoothing

        if alpha is None:
            self.alpha = None
        elif isinstance(alpha, (list, tuple)):
            alpha_clamped = [alpha[0], min(alpha[1], 4.0)]
            self.alpha = torch.tensor(alpha_clamped, dtype=torch.float32)
            print(f"[FocalLoss] 权重: {alpha} -> {alpha_clamped}, gamma={gamma}, smoothing={label_smoothing}")
        else:
            self.alpha = alpha

    def forward(self, inputs, targets):
        eps = 1e-7
        num_classes = inputs.size(1)


        if self.label_smoothing > 0:
            smooth_targets = torch.zeros_like(inputs)
            smooth_targets.fill_(self.label_smoothing / (num_classes - 1))
            smooth_targets.scatter_(1, targets.unsqueeze(1), 1.0 - self.label_smoothing)
        else:
            smooth_targets = F.one_hot(targets, num_classes).float()

        p = F.softmax(inputs, dim=1)
        p = p.clamp(min=eps, max=1 - eps)

        ce_loss = -(smooth_targets * torch.log(p)).sum(dim=1)

        p_t = p.gather(1, targets.unsqueeze(1)).squeeze(1)
        focal_weight = (1 - p_t) ** self.gamma
        focal_loss = focal_weight * ce_loss

        if self.alpha is not None:
            if self.alpha.device != inputs.device:
                self.alpha = self.alpha.to(inputs.device)
            alpha_t = self.alpha.gather(0, targets)
            focal_loss = alpha_t * focal_loss

        if self.reduction == 'mean':
            return focal_loss.mean()
        elif self.reduction == 'sum':
            return focal_loss.sum()
        return focal_loss


class MultiExperimentTrainer:


    def __init__(self, model, learning_rate=0.001, device='cpu', class_weights=None,
                 weight_decay=1e-4):
        self.model = model.to(device)
        self.device = device


        self.optimizer = torch.optim.AdamW(
            model.parameters(),
            lr=learning_rate,
            weight_decay=weight_decay
        )


        if class_weights is not None:
            self.criterion = FocalLoss(alpha=class_weights, gamma=1.5, label_smoothing=0.05)
        else:
            self.criterion = FocalLoss(alpha=None, gamma=1.5, label_smoothing=0.05)
        print(f"使用 AdamW (lr={learning_rate}, wd={weight_decay})")


        self.scheduler = torch.optim.lr_scheduler.CosineAnnealingWarmRestarts(
            self.optimizer, T_0=10, T_mult=2, eta_min=1e-6
        )


        self.train_losses, self.val_losses = [], []
        self.train_accs, self.val_accs = [], []
        self.val_precisions, self.val_recalls, self.val_f1s = [], [], []


        self.best_val_loss = float('inf')
        self.best_val_f1 = 0.0
        self.best_model_state = None
        self.best_epoch = 0
        self.best_threshold = 0.5

        self.best_val_loss_only = float('inf')
        self.best_model_state_by_loss = None
        self.best_epoch_by_loss = 0
        self.best_f1_at_best_loss = 0.0
        self.best_threshold_by_loss = 0.5

    def _compute_metrics(self, all_preds, all_labels):

        if len(all_preds) == 0:
            return {'accuracy': 0, 'precision': 0, 'recall': 0, 'f1': 0}

        all_preds = np.array(all_preds)
        all_labels = np.array(all_labels)

        accuracy = (all_preds == all_labels).sum() / len(all_labels)


        tp = float(((all_preds == 1) & (all_labels == 1)).sum())
        fp = float(((all_preds == 1) & (all_labels == 0)).sum())
        fn = float(((all_preds == 0) & (all_labels == 1)).sum())
        tn = float(((all_preds == 0) & (all_labels == 0)).sum())

        precision = tp / (tp + fp) if (tp + fp) > 0 else 0
        recall = tp / (tp + fn) if (tp + fn) > 0 else 0
        f1 = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0

        return {
            'accuracy': accuracy,
            'precision': precision,
            'recall': recall,
            'f1': f1,
            'tp': int(tp), 'fp': int(fp), 'fn': int(fn), 'tn': int(tn)
        }

    def train_epoch(self, train_loader):

        self.model.train()
        total_loss = 0.0
        num_batches = 0
        all_preds, all_labels = [], []

        for batch in train_loader:
            self.optimizer.zero_grad()
            batched_graph = batch['batched_graph'].to(self.device)
            target_bus_indices = batch['target_bus_indices']
            future_stop_indices_list = batch['future_stop_indices_list']
            labels_list = batch['labels_list']

            predictions_list = self.model(batched_graph, target_bus_indices, future_stop_indices_list)

            batch_loss = 0.0
            batch_sample_count = 0

            for predictions, labels in zip(predictions_list, labels_list):
                if len(labels) != predictions.size(0):
                    min_len = min(len(labels), predictions.size(0))
                    labels, predictions = labels[:min_len], predictions[:min_len]

                labels_tensor = torch.tensor(labels, dtype=torch.long, device=self.device)
                sample_loss = self.criterion(predictions, labels_tensor)
                batch_loss += sample_loss * len(labels)
                batch_sample_count += len(labels)

                pred_labels = predictions.argmax(dim=1).cpu().numpy()
                all_preds.extend(pred_labels)
                all_labels.extend(labels)

            if batch_sample_count > 0:
                batch_loss = batch_loss / batch_sample_count

            batch_loss.backward()
            torch.nn.utils.clip_grad_norm_(self.model.parameters(), max_norm=1.0)
            self.optimizer.step()

            total_loss += batch_loss.item()
            num_batches += 1

        avg_loss = total_loss / num_batches if num_batches > 0 else 0
        metrics = self._compute_metrics(all_preds, all_labels)

        return avg_loss, metrics['accuracy']

    def validate(self, val_loader):

        self.model.eval()
        total_loss = 0.0
        total_samples = 0
        all_preds, all_labels, all_probs = [], [], []

        with torch.no_grad():
            for batch in val_loader:
                batched_graph = batch['batched_graph'].to(self.device)
                target_bus_indices = batch['target_bus_indices']
                future_stop_indices_list = batch['future_stop_indices_list']
                labels_list = batch['labels_list']

                predictions_list = self.model(batched_graph, target_bus_indices, future_stop_indices_list)

                for predictions, labels in zip(predictions_list, labels_list):
                    if len(labels) != predictions.size(0):
                        min_len = min(len(labels), predictions.size(0))
                        labels, predictions = labels[:min_len], predictions[:min_len]

                    labels_tensor = torch.tensor(labels, dtype=torch.long, device=self.device)
                    sample_loss = self.criterion(predictions, labels_tensor).item()
                    total_loss += sample_loss * len(labels)
                    total_samples += len(labels)


                    probs = torch.softmax(predictions, dim=1)[:, 1].cpu().numpy()
                    all_probs.extend(probs)
                    all_labels.extend(labels)


                    pred_labels = (probs >= self.best_threshold).astype(int)
                    all_preds.extend(pred_labels)

        avg_loss = total_loss / total_samples if total_samples > 0 else 0


        best_thresh, best_f1_at_thresh = self._find_best_threshold(all_probs, all_labels)


        all_preds_optimal = (np.array(all_probs) >= best_thresh).astype(int).tolist()
        metrics = self._compute_metrics(all_preds_optimal, all_labels)
        metrics['best_threshold'] = best_thresh

        return avg_loss, metrics

    def _find_best_threshold(self, all_probs, all_labels):

        best_f1 = 0
        best_thresh = 0.5

        for thresh in np.arange(0.25, 0.75, 0.02):
            preds = (np.array(all_probs) >= thresh).astype(int)
            labels = np.array(all_labels)

            tp = ((preds == 1) & (labels == 1)).sum()
            fp = ((preds == 1) & (labels == 0)).sum()
            fn = ((preds == 0) & (labels == 1)).sum()

            precision = tp / (tp + fp) if (tp + fp) > 0 else 0
            recall = tp / (tp + fn) if (tp + fn) > 0 else 0
            f1 = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0

            if f1 > best_f1:
                best_f1 = f1
                best_thresh = thresh

        return best_thresh, best_f1

    def train(self, train_loader, val_loader, num_epochs=100, early_stop_patience=20):

        print(f"\n{'=' * 60}")
        print(f"开始训练（修复版）")
        print(f"{'=' * 60}")

        self._print_class_distribution(train_loader, val_loader)

        no_improve_count = 0

        for epoch in range(num_epochs):

            train_loss, train_acc = self.train_epoch(train_loader)


            val_loss, val_metrics = self.validate(val_loader)
            val_acc = val_metrics['accuracy']
            val_precision = val_metrics['precision']
            val_recall = val_metrics['recall']
            val_f1 = val_metrics['f1']
            best_thresh = val_metrics.get('best_threshold', 0.5)


            self.train_losses.append(train_loss)
            self.val_losses.append(val_loss)
            self.train_accs.append(train_acc)
            self.val_accs.append(val_acc)
            self.val_precisions.append(val_precision)
            self.val_recalls.append(val_recall)
            self.val_f1s.append(val_f1)


            self.scheduler.step(val_loss)


            is_best_by_loss = False
            if val_loss < self.best_val_loss_only - 0.001:
                self.best_val_loss_only = val_loss
                self.best_model_state_by_loss = {k: v.cpu().clone() for k, v in self.model.state_dict().items()}
                self.best_epoch_by_loss = epoch + 1
                self.best_f1_at_best_loss = val_f1
                self.best_threshold_by_loss = best_thresh
                torch.save(self.model.state_dict(), r'TrainedModel_0201_v3\best_model_by_val_loss.pth')
                is_best_by_loss = True


            is_best = False
            if val_f1 > self.best_val_f1 + 0.005:
                is_best = True
            elif val_f1 >= self.best_val_f1 - 0.005 and val_loss < self.best_val_loss - 0.001:

                is_best = True

            if is_best:
                self.best_val_loss = val_loss
                self.best_val_f1 = val_f1
                self.best_threshold = best_thresh
                self.best_model_state = {k: v.cpu().clone() for k, v in self.model.state_dict().items()}
                self.best_epoch = epoch + 1
                no_improve_count = 0
                torch.save(self.model.state_dict(), r'TrainedModel_0201_v3\best_model_by_F1.pth')
                best_marker = " ★ Best(F1)"
            else:
                no_improve_count += 1
                best_marker = ""


            if is_best_by_loss and not is_best:
                best_marker += " ◆ Best(Loss)"
            elif is_best_by_loss and is_best:
                best_marker = " ★◆ Best(F1&Loss)"


            if (epoch + 1) % 2 == 0 or epoch == 0 or is_best or is_best_by_loss:
                current_lr = self.optimizer.param_groups[0]['lr']
                print(f"Epoch {epoch + 1:3d}/{num_epochs} [lr={current_lr:.1e}]: "
                      f"Train Loss={train_loss:.4f}, Train Acc={train_acc:.4f} | "
                      f"Val Loss={val_loss:.4f}, Val Acc={val_acc:.4f}, "
                      f"P={val_precision:.4f}, R={val_recall:.4f}, F1={val_f1:.4f}, "
                      f"Thresh={best_thresh:.2f}{best_marker}")


            if no_improve_count >= early_stop_patience:
                print(f"\n早停触发！验证指标已经 {early_stop_patience} 轮未改善。")
                break


        if self.best_model_state is not None:
            self.model.load_state_dict(self.best_model_state)
            print(f"\n已恢复第 {self.best_epoch} 轮的最佳模型（基于F1）")
            print(f"  Val Loss={self.best_val_loss:.4f}, F1={self.best_val_f1:.4f}, "
                  f"Threshold={self.best_threshold:.2f}")


        if self.best_model_state_by_loss is not None:
            print(f"\n另有第 {self.best_epoch_by_loss} 轮的模型（基于Val Loss最优）已保存为 best_model_by_val_loss.pth")
            print(f"  Val Loss={self.best_val_loss_only:.4f}, F1={self.best_f1_at_best_loss:.4f}, "
                  f"Threshold={self.best_threshold_by_loss:.2f}")
            print(f"\n提示：如果验证集Loss在后期明显上升（过拟合），建议使用 best_model_by_val_loss.pth")

        self._print_final_results()

    def _print_class_distribution(self, train_loader, val_loader):

        train_labels = []
        for batch in train_loader:
            for labels in batch['labels_list']:
                train_labels.extend(labels)

        val_labels = []
        for batch in val_loader:
            for labels in batch['labels_list']:
                val_labels.extend(labels)

        train_pos = sum(train_labels)
        train_neg = len(train_labels) - train_pos
        val_pos = sum(val_labels)
        val_neg = len(val_labels) - val_pos

        print(f"\n类别分布:")
        print(f"  训练集: 负样本(0)={train_neg}, 正样本(1)={train_pos}, "
              f"正样本比例={train_pos / len(train_labels) * 100:.2f}%")
        print(f"  验证集: 负样本(0)={val_neg}, 正样本(1)={val_pos}, "
              f"正样本比例={val_pos / len(val_labels) * 100:.2f}%")
        print()

    def _print_final_results(self):

        print(f"\n{'=' * 60}")
        print(f"训练完成！")
        print(f"{'=' * 60}")
        print(f"最佳模型: 第 {self.best_epoch} 轮")
        print(f"  Val Loss: {self.best_val_loss:.4f}")
        print(f"  Val F1: {self.best_val_f1:.4f}")
        print(f"  最优阈值: {self.best_threshold:.2f}")
        if self.val_f1s:
            best_idx = self.best_epoch - 1
            print(f"  Val Accuracy: {self.val_accs[best_idx]:.4f}")
            print(f"  Val Precision: {self.val_precisions[best_idx]:.4f}")
            print(f"  Val Recall: {self.val_recalls[best_idx]:.4f}")


def first_runing_save_training_data(busline_distance_between_stops):

    print("=" * 60 + "\n第一次运行：生成训练数据\n" + "=" * 60)
    print("\n[1/4] 加载所有实验...")
    data_manager = MultiExperimentDataManager()
    data_manager.load_all_experiments_from_directory("../i_up_control_update")
    data_manager.save_to_disk('b_all_gurobi_exper_data_0201_v3.pkl')
    stats = data_manager.get_statistics()
    print(
        f"\n实验数量: {stats['num_experiments']}, 晚点场景数: {stats['total_delay_scenes']}, 总时刻数: {stats['total_time_moments']}")
    print("\n[2/4] 计算归一化参数...")
    processor = BusDelayDataProcessor()
    processor.compute_normalization_params(data_manager.experiments, busline_distance_between_stops)
    print("\n[3/4] 构建训练数据集...")
    dataset_builder = BusDelayDatasetBuilder(data_manager, processor)
    samples = dataset_builder.build_dataset(max_future_stops=10,
                                            busline_distance_between_stops=busline_distance_between_stops)
    if len(samples) == 0:
        print("\n⚠️  警告：没有生成任何训练样本！")
        return
    print("\n[4/4] 保存数据集...")
    dataset_builder.save_dataset('b_bus_delay_dataset_0201_v3.pkl')
    print("\n" + "=" * 60 + "\n✓ 训练数据生成完成！\n" + "=" * 60)


def run_training_pipeline(data_file='b_all_gurobi_exper_data.pkl', dataset_file='b_bus_delay_dataset.pkl',
                          num_epochs=100, learning_rate=0.001, batch_size=32, busline_distance_between_stops=None):

    print("=" * 60 + "\n公交晚点预测系统 - 训练流程（批处理模式）\n" + "=" * 60)
    if busline_distance_between_stops is None:
        busline_distance_between_stops = BUSLINE_DISTANCE_BETWEEN_STOPS
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"\n使用设备: {device}")
    print("\n[1/5] 加载实验数据...")
    data_manager = MultiExperimentDataManager()
    data_manager.load_from_disk(data_file)
    print(f"✓ 加载了 {len(data_manager.experiments)} 个实验")
    print("\n[2/5] 加载训练数据集...")
    processor = BusDelayDataProcessor()
    processor.compute_normalization_params(data_manager.experiments, busline_distance_between_stops)
    dataset_builder = BusDelayDatasetBuilder(data_manager, processor)
    dataset_builder.load_dataset(dataset_file)
    samples = dataset_builder.samples
    print(f"✓ 加载了 {len(samples)} 个训练样本")
    print("\n[3/5] 划分训练集和验证集...")
    experiment_ids = list(set(s['experiment_id'] for s in samples))
    np.random.seed(42)
    np.random.shuffle(experiment_ids)
    split_idx = int(0.8 * len(experiment_ids))
    train_exp_ids, val_exp_ids = set(experiment_ids[:split_idx]), set(experiment_ids[split_idx:])
    train_samples = [s for s in samples if s['experiment_id'] in train_exp_ids]
    val_samples = [s for s in samples if s['experiment_id'] in val_exp_ids]
    print(f"✓ 训练集: {len(train_samples)} 样本, 验证集: {len(val_samples)} 样本")
    print("\n[4/5] 创建数据加载器...")
    train_loader = create_dataloader(train_samples, batch_size=batch_size, shuffle=True)
    val_loader = create_dataloader(val_samples, batch_size=batch_size, shuffle=False)
    print(f"✓ 训练批次: {len(train_loader)}（batch_size={batch_size}）, 验证批次: {len(val_loader)}")
    print("\n[5/5] 初始化并训练模型...")
    model = BusDelayPredictionGNN(
        bus_feature_dim=3,
        stop_feature_dim=2,
        hidden_dim=64,
        num_gnn_layers=3,
        bus_bus_edge_dim=2,
        station_station_edge_dim=2,
        bus_station_edge_dim=3,
        dropout=0.15
    )
    print(f"✓ 模型参数量: {sum(p.numel() for p in model.parameters()):,}")

    train_labels = []
    for sample in train_samples:
        train_labels.extend(sample['labels'])
    num_neg = train_labels.count(0)
    num_pos = train_labels.count(1)

    class_weights = None
    if num_pos > 0 and num_pos / len(train_labels) < 0.3:


        raw_weight = num_neg / num_pos

        weight_for_pos = min(np.sqrt(raw_weight) * 1.2, 5.0)
        class_weights = [1.0, weight_for_pos]
        print(f"检测到类别不平衡 (正样本比例={num_pos / len(train_labels) * 100:.2f}%)")
        print(f"原始权重比: {raw_weight:.2f}, 调整后权重: {class_weights}")

    trainer = MultiExperimentTrainer(model, learning_rate=learning_rate, device=device, class_weights=class_weights)
    trainer.train(train_loader, val_loader, num_epochs=num_epochs)
    print("\n" + "=" * 60 + "\n训练完成！\n" + "=" * 60)
    return model, trainer


def test_single_sample(model, sample, device='cpu'):
    model.eval()
    model.to(device)
    with torch.no_grad():
        predictions, attn_weights = model.forward_single(sample['graph_data'], sample['target_bus_idx'],
                                                         torch.tensor(sample['future_stop_indices']))
        pred_labels = predictions.argmax(dim=1)
        print(
            f"\n测试样本:\n  实验ID: {sample['experiment_id']}\n  时刻: {sample['time_moment']}\n  公交车: {sample['bus_id']}\n\n预测结果:")
        for i, stop_id in enumerate(sample['future_stop_ids']):
            true_label, pred_label = sample['labels'][i], pred_labels[i].item()
            status = "✓" if true_label == pred_label else "✗"
            print(
                f"  {status} 站点 {stop_id}: 真实={'受影响' if true_label == 1 else '不受影响'}, 预测={'受影响' if pred_label == 1 else '不受影响'}")


def extract_bus_distances(xml_file_path):
    tree = ET.parse(xml_file_path)
    root = tree.getroot()
    busline_distance_between_stops = {}
    for line_obj in root.findall('.//line_obj'):
        line_id = line_obj.get('line_id_s')
        if not line_id:
            continue
        distance_elem = line_obj.find('distance_between_stop')
        if distance_elem is None:
            continue
        stop_distances = OrderedDict()
        for attr_name, attr_value in distance_elem.attrib.items():
            if attr_name.startswith('s_'):
                stop_name = attr_name[2:]
                try:
                    stop_distances[stop_name] = float(attr_value)
                except ValueError:
                    continue
        line_distances = {}
        stop_names = list(stop_distances.keys())
        for i in range(len(stop_names) - 1):
            current_stop, next_stop = stop_names[i], stop_names[i + 1]
            line_distances[(current_stop, next_stop)] = stop_distances[next_stop]
        if line_distances:
            busline_distance_between_stops[line_id] = line_distances
    return busline_distance_between_stops


def save_model_and_trainer(model, trainer, save_dir='./saved_models_V2'):


    save_dir = Path(save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)

    print(f"\n{'=' * 60}")
    print(f"正在保存模型到: {save_dir}")
    print(f"{'=' * 60}")


    model_path = save_dir / 'model_weights.pth'
    torch.save(model.state_dict(), model_path)
    print(f"✓ 模型权重已保存（F1最优，Epoch {trainer.best_epoch}）: {model_path}")


    full_model_path = save_dir / 'full_model.pth'
    torch.save(model, full_model_path)
    print(f"✓ 完整模型已保存: {full_model_path}")


    if trainer.best_model_state_by_loss is not None:
        model_by_loss_path = save_dir / 'model_weights_by_val_loss.pth'
        torch.save(trainer.best_model_state_by_loss, model_by_loss_path)
        print(f"✓ Val Loss最优模型已保存（Epoch {trainer.best_epoch_by_loss}）: {model_by_loss_path}")


    training_history = {
        'train_losses': trainer.train_losses,
        'val_losses': trainer.val_losses,
        'val_accs': trainer.val_accs,
        'val_f1s': trainer.val_f1s,
        'val_precisions': trainer.val_precisions,
        'val_recalls': trainer.val_recalls,

        'best_epoch_by_f1': trainer.best_epoch,
        'best_val_loss_by_f1': trainer.best_val_loss,
        'best_val_f1': trainer.best_val_f1,
        'best_threshold_by_f1': trainer.best_threshold,
        'best_epoch_by_loss': trainer.best_epoch_by_loss,
        'best_val_loss_only': trainer.best_val_loss_only,
        'best_f1_at_best_loss': trainer.best_f1_at_best_loss,
        'best_threshold_by_loss': trainer.best_threshold_by_loss,
    }
    history_path = save_dir / 'training_history.pth'
    torch.save(training_history, history_path)
    print(f"✓ 训练历史已保存: {history_path}")


    optimizer_path = save_dir / 'optimizer.pth'
    torch.save(trainer.optimizer.state_dict(), optimizer_path)
    print(f"✓ 优化器状态已保存: {optimizer_path}")


    hyperparams = {
        'bus_feature_dim': 3,
        'stop_feature_dim': 2,
        'hidden_dim': model.hidden_dim,
        'num_gnn_layers': model.num_gnn_layers,
    }
    hyperparams_path = save_dir / 'hyperparameters.pth'
    torch.save(hyperparams, hyperparams_path)
    print(f"✓ 超参数已保存: {hyperparams_path}")

    print(f"{'=' * 60}")
    print(f"✅ 所有文件保存完成！")
    print(f"{'=' * 60}")


    print(f"\n📊 模型对比:")
    print(f"  ┌─────────────────────────────────────────────────────────┐")
    print(f"  │ F1最优模型 (model_weights.pth)                         │")
    print(f"  │   Epoch: {trainer.best_epoch:3d}, Val Loss: {trainer.best_val_loss:.4f}, F1: {trainer.best_val_f1:.4f} │")
    print(f"  ├─────────────────────────────────────────────────────────┤")
    print(f"  │ Loss最优模型 (model_weights_by_val_loss.pth)           │")
    print(f"  │   Epoch: {trainer.best_epoch_by_loss:3d}, Val Loss: {trainer.best_val_loss_only:.4f}, F1: {trainer.best_f1_at_best_loss:.4f} │")
    print(f"  └─────────────────────────────────────────────────────────┘")
    print(f"\n💡 建议: 如果观察到后期Val Loss上升（过拟合），可以使用 model_weights_by_val_loss.pth\n")


def debug_dynamic_schedule(experiment_data, bus_id, stop_id):

    print(f"\n{'=' * 60}")
    print(f"调试动态计划时刻表")
    print(f"公交车: {bus_id}, 站点: {stop_id}")
    print(f"{'=' * 60}")

    schedule_manager = DynamicScheduleManager(
        experiment_data['original_bus_schedule'],
        experiment_data['all_time_bus_decision_timetable']
    )

    line_id = bus_id.rsplit('_', 1)[0]


    try:
        original_time = experiment_data['original_bus_schedule'][line_id][bus_id][stop_id]
        print(f"\n原始计划时间: 到达={original_time[0]}, 发车={original_time[1]}")
    except KeyError:
        print(f"\n原始计划时间: 未找到")


    print(f"\n各时刻的动态计划时间:")
    for time_value in schedule_manager.sorted_times:
        info = schedule_manager.get_schedule_info_for_debug(time_value, bus_id, stop_id)
        dynamic_time = info['dynamic_schedule']
        update_source = info['update_source_time']

        if dynamic_time:
            source_str = f" (来源: {update_source}时刻的决策)" if update_source else " (原始计划)"
            print(f"  时刻 {time_value}: 到达={dynamic_time[0]:.2f}, 发车={dynamic_time[1]:.2f}{source_str}")
        else:
            print(f"  时刻 {time_value}: 未找到")

    print(f"\n{'=' * 60}\n")


def verify_affected_only_update(experiment_data):

    print(f"\n{'=' * 60}")
    print(f"验证：动态计划时刻表只更新'受影响'的组合")
    print(f"{'=' * 60}")

    schedule_manager = DynamicScheduleManager(
        experiment_data['original_bus_schedule'],
        experiment_data['all_time_bus_decision_timetable']
    )

    original_schedule = experiment_data['original_bus_schedule']


    for time_value in schedule_manager.sorted_times:
        dynamic_schedule = schedule_manager.get_dynamic_schedule(time_value)


        updated_count = 0
        affected_count = 0


        past_times = [t for t in schedule_manager.sorted_times if t < time_value]
        all_affected = {}
        for pt in past_times:
            affected = schedule_manager.affected_decisions.get(pt, {})
            for bus_id, stops in affected.items():
                if bus_id not in all_affected:
                    all_affected[bus_id] = set()
                all_affected[bus_id].update(stops.keys())


        for line_id, buses in dynamic_schedule.items():
            for bus_id, stops in buses.items():
                for stop_id, times in stops.items():

                    try:
                        original_times = original_schedule[line_id][bus_id][stop_id]
                        if times[0] != original_times[0] or times[1] != original_times[1]:
                            updated_count += 1

                            if bus_id in all_affected and stop_id in all_affected[bus_id]:
                                affected_count += 1
                            else:
                                print(f"  ⚠️ 警告：时刻{time_value}，{bus_id}在{stop_id}被更新但不在'受影响'列表中")
                    except KeyError:
                        continue

        if updated_count > 0:
            print(f"  时刻 {time_value}: 更新数={updated_count}, 均来自'受影响'={affected_count == updated_count}")

    print(f"{'=' * 60}\n")


def compare_old_new_logic(experiment_data):

    print(f"\n{'=' * 60}")
    print(f"对比：修改前（全部更新） vs 修改后（只更新受影响）")
    print(f"{'=' * 60}")


    new_manager = DynamicScheduleManager(
        experiment_data['original_bus_schedule'],
        experiment_data['all_time_bus_decision_timetable']
    )

    decision_timetable = experiment_data['all_time_bus_decision_timetable']
    original_schedule = experiment_data['original_bus_schedule']

    for time_value in new_manager.sorted_times[1:3]:
        print(f"\n时刻 {time_value}:")


        past_times = [t for t in new_manager.sorted_times if t < time_value]
        old_update_count = 0
        for pt in past_times:
            decisions = new_manager._get_decision_at_time(pt)
            if decisions:
                for bus_id, stops in decisions.items():
                    old_update_count += len(stops)


        new_update_count = 0
        for pt in past_times:
            affected = new_manager.affected_decisions.get(pt, {})
            for bus_id, stops in affected.items():
                new_update_count += len(stops)

        print(f"  旧逻辑更新数（全部决策）: {old_update_count}")
        print(f"  新逻辑更新数（只受影响）: {new_update_count}")
        if old_update_count > 0:
            print(f"  减少比例: {(old_update_count - new_update_count) / old_update_count * 100:.1f}%")

    print(f"\n{'=' * 60}\n")


def demonstrate_dynamic_update_example(experiment_data, max_buses=2, max_stops_per_bus=3):

    print(f"\n{'=' * 80}")
    print(f"{'动态计划时刻表更新过程详细示例':^70}")
    print(f"{'=' * 80}")


    schedule_manager = DynamicScheduleManager(
        experiment_data['original_bus_schedule'],
        experiment_data['all_time_bus_decision_timetable']
    )

    original_schedule = experiment_data['original_bus_schedule']
    decision_timetable = experiment_data['all_time_bus_decision_timetable']


    print(f"\n【基本参数】")
    print(f"  偏差阈值 THRESHOLD_CHANGE = {DynamicScheduleManager.THRESHOLD_CHANGE} 秒")
    print(f"  总决策时刻数: {len(schedule_manager.sorted_times)}")
    print(f"  决策时刻序列: {schedule_manager.sorted_times[:10]}{'...' if len(schedule_manager.sorted_times) > 10 else ''}")


    print(f"\n{'─' * 80}")
    print(f"【寻找有'受影响'决策的公交车-站点组合】")
    print(f"{'─' * 80}")


    affected_examples = []

    for time_value, affected in schedule_manager.affected_decisions.items():
        for bus_id, stops in affected.items():
            for stop_id in stops.keys():

                existing = [e for e in affected_examples if e[0] == bus_id and e[1] == stop_id]
                if not existing:
                    affected_examples.append((bus_id, stop_id, time_value))

    if not affected_examples:
        print("  未找到任何'受影响'的决策，无法展示示例。")
        return


    selected_examples = affected_examples[:max_buses * max_stops_per_bus]
    print(f"  找到 {len(affected_examples)} 个受影响的公交车-站点组合")
    print(f"  选择展示 {len(selected_examples)} 个示例")


    for example_idx, (bus_id, stop_id, first_affected_time) in enumerate(selected_examples[:max_buses]):
        line_id = bus_id.rsplit('_', 1)[0]

        print(f"\n{'=' * 80}")
        print(f"【示例 {example_idx + 1}】公交车: {bus_id}, 站点: {stop_id}")
        print(f"{'=' * 80}")


        try:
            original_times = original_schedule[line_id][bus_id][stop_id]
            print(f"\n[原始计划时刻表]")
            print(f"  到达时间: {original_times[0]:.2f} 秒")
            print(f"  发车时间: {original_times[1]:.2f} 秒")
        except KeyError:
            print(f"\n[原始计划时刻表] 未找到")
            continue


        print(f"\n[逐时刻更新过程]")
        print(f"  {'─' * 76}")
        print(f"  {'时刻':<10} │ {'决策到达':<12} │ {'动态计划':<12} │ {'偏差':<10} │ {'是否受影响':<10} │ {'更新后计划':<12}")
        print(f"  {'─' * 76}")

        cumulative_dynamic_arrival = original_times[0]

        for time_idx, time_value in enumerate(schedule_manager.sorted_times):

            decisions = schedule_manager._get_decision_at_time(time_value)


            dynamic_schedule_before = schedule_manager.get_dynamic_schedule(time_value)
            try:
                dynamic_arrival_before = dynamic_schedule_before[line_id][bus_id][stop_id][0]
            except KeyError:
                dynamic_arrival_before = original_times[0]


            if decisions and bus_id in decisions and stop_id in decisions[bus_id]:
                decision_arrival = decisions[bus_id][stop_id][0]
                deviation = abs(decision_arrival - dynamic_arrival_before)
                is_affected = deviation > DynamicScheduleManager.THRESHOLD_CHANGE


                if time_idx + 1 < len(schedule_manager.sorted_times):
                    next_time = schedule_manager.sorted_times[time_idx + 1]
                    dynamic_schedule_after = schedule_manager.get_dynamic_schedule(next_time)
                    try:
                        dynamic_arrival_after = dynamic_schedule_after[line_id][bus_id][stop_id][0]
                    except KeyError:
                        dynamic_arrival_after = dynamic_arrival_before
                else:
                    dynamic_arrival_after = decision_arrival if is_affected else dynamic_arrival_before


                affected_marker = "✓ 是" if is_affected else "✗ 否"
                deviation_str = f"{deviation:.1f}秒"


                if is_affected:
                    print(
                        f"  {time_value:<10.1f} │ {decision_arrival:<12.2f} │ {dynamic_arrival_before:<12.2f} │ {deviation_str:<10} │ {affected_marker:<10} │ {dynamic_arrival_after:<12.2f} ★")
                else:
                    print(
                        f"  {time_value:<10.1f} │ {decision_arrival:<12.2f} │ {dynamic_arrival_before:<12.2f} │ {deviation_str:<10} │ {affected_marker:<10} │ {dynamic_arrival_after:<12.2f}")
            else:

                print(
                    f"  {time_value:<10.1f} │ {'--':<12} │ {dynamic_arrival_before:<12.2f} │ {'--':<10} │ {'无决策':<10} │ {dynamic_arrival_before:<12.2f}")

        print(f"  {'─' * 76}")


        print(f"\n[更新总结]")


        affected_times = []
        for t, affected in schedule_manager.affected_decisions.items():
            if bus_id in affected and stop_id in affected[bus_id]:
                affected_times.append(t)

        print(f"  该公交车-站点被判定为'受影响'的时刻: {affected_times}")
        print(f"  共被更新 {len(affected_times)} 次")


        final_time = schedule_manager.sorted_times[-1]
        final_schedule = schedule_manager.get_dynamic_schedule(final_time)
        try:
            final_arrival = final_schedule[line_id][bus_id][stop_id][0]
            total_change = final_arrival - original_times[0]
            print(f"  原始计划到达: {original_times[0]:.2f} → 最终动态计划: {final_arrival:.2f}")
            print(f"  总变化量: {total_change:+.2f} 秒")
        except KeyError:
            pass


    print(f"\n{'=' * 80}")
    print(f"【对比：全量更新 vs 受影响更新】")
    print(f"{'=' * 80}")


    total_decisions = 0
    total_affected = 0

    for time_value in schedule_manager.sorted_times:
        decisions = schedule_manager._get_decision_at_time(time_value)
        affected = schedule_manager.affected_decisions.get(time_value, {})

        if decisions:
            for bus_id, stops in decisions.items():
                total_decisions += len(stops)

        for bus_id, stops in affected.items():
            total_affected += len(stops)

    print(f"\n  全部决策数量（旧逻辑会全部更新）: {total_decisions}")
    print(f"  受影响决策数量（新逻辑只更新这些）: {total_affected}")
    print(
        f"  减少的更新量: {total_decisions - total_affected} ({(total_decisions - total_affected) / total_decisions * 100:.1f}%)")

    print(f"\n{'=' * 80}")
    print(f"{'示例展示完毕':^70}")
    print(f"{'=' * 80}\n")


if __name__ == "__main__":
    line_xml_path = "../i_up_control_update/initialize_obj/save_obj_line.add.xml"
    BUSLINE_DISTANCE_BETWEEN_STOPS = extract_bus_distances(line_xml_path)





    model, trainer = run_training_pipeline(data_file='b_all_gurobi_exper_data_0201_v3.pkl',
                                           dataset_file='b_bus_delay_dataset_0201_v3.pkl',
                                           num_epochs=100, learning_rate=0.001, batch_size=32,
                                           busline_distance_between_stops=BUSLINE_DISTANCE_BETWEEN_STOPS)
    debug = 0

    save_model_and_trainer(model, trainer, save_dir='TrainedModel_0201_v3')
    DEBUG = 0
