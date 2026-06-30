#!/usr/bin/env python3

import json
from pathlib import Path
from geopy.distance import distance
from robot_core.Tools import clamp_int


class Mission:
    def __init__(self, raw_json: str):
        
        self.raw_json = raw_json
        self.cloud_id = 0
        self.name = ""

        self.home_lat = 0.0
        self.home_lon = 0.0
        self.paths = []
        self.areas = []
        self.routes = []

        self.waypoints = []
        self.total_distance = 0.0
        self.completion_rate = 0
        self.index = 0

        self.read_mission_json(self.mission_json)

    def next_wp(self):
        response = self.waypoints[self.index]
        self.index += 1
        return response
    

    def read_mission_json(self, mission_json) -> None:
        mission = json.loads(mission_json)

        self.cloud_id = int(mission["ID"])
        self.name = str(mission["isim"])
        self.home_lat = mission["home"][0]
        self.home_lon = mission["home"][1]
        self.paths = list(mission["yollar"])
        self.areas = list(mission["araziler"])
        self.routes = list(mission["rotalar"])

        self.fill_waypoints()
        self.total_distance = self.calculate_distance(self.waypoints)
        self.completion_rate = self.calculate_completion_rate(self.waypoints, self.index)

    
    def fill_waypoints(self) -> None:
        self.waypoints.append([self.home_lat, self.home_lon, False])

        for route in self.routes:
            etiket = route["etiket"]
            overlay_type = route["overlayType"]
            reverse = route["reverse"]

            if overlay_type == "polyline":
                temp = next((u for u in self.paths if u["etiket"] == etiket), None)
                coords = temp["konumlar"] if temp else None
                if coords:
                    if reverse:
                        coords = coords[::-1]
                    formatted_coords = [[lat, lon, False] for lat, lon in coords]
                    self.waypoints.extend(formatted_coords)
            elif overlay_type == "polygon":
                temp = next((u for u in self.areas if u["etiket"] == etiket), None)
                coords = temp["generatedRoute"] if temp else None
                if coords:
                    formatted_coords = [[lat, lon, True] for lat, lon in coords]
                    self.waypoints.extend(formatted_coords)

        self.waypoints.append([self.home_lat, self.home_lon, False])

    def calculate_completion_rate(self, all_waypoints: list, index: int) -> int:
        total_distance = self.calculate_distance(all_waypoints)
        if total_distance <= 0.0:
            return 0

        clamped_index = clamp_int(index, 0, len(all_waypoints))
        current_distance = self.calculate_distance(all_waypoints[:clamped_index])
        completion_rate = int((current_distance / total_distance) * 100)

        return clamp_int(completion_rate, 0, 100)
        


    def calculate_distance(self, waypoints) -> float:
        total_distance = 0.0

        for previous_waypoint, next_waypoint in zip(waypoints, waypoints[1:]):
            previous_position = previous_waypoint[:2]
            next_position = next_waypoint[:2]
            total_distance += distance(previous_position, next_position).meters

        return total_distance


if __name__ == "__main__":
    mission_path = Path(__file__).with_name("utaem_mission.json")
    mission_text = mission_path.read_text(encoding="utf-8")

    mission = Mission(mission_text)
    mission.index = 10
    mission.completion_rate = mission.calculate_completion_rate(mission.waypoints, mission.index)

    print(mission.completion_rate)
