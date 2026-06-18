#!/usr/bin/env python3

import json
from pathlib import Path

from geopy.distance import distance


class Mission:
    def __init__(self):
        self.filename = Path(__file__).with_name("utaem_mission.json")
        self.cloud_id = 0
        self.name = ""

        self.home_lat = 0.0
        self.home_lon = 0.0
        self.paths = []
        self.areas = []
        self.routes = []

        self.waypoints = []

        with open(self.filename, "r", encoding="utf-8") as file:
            self.read_mission_json(file.read())

    def read_mission_json(self, mission_json: str) -> None:
        mission = json.loads(mission_json)

        self.cloud_id = int(mission["ID"])
        self.name = str(mission["isim"])
        self.paths = list(mission["yollar"])
        self.areas = list(mission["araziler"])
        self.routes = list(mission["rotalar"])
        self.home_lat = mission["home"][0]
        self.home_lon = mission["home"][1]

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

    def get_total_distance(self) -> float:
        total_distance = 0.0

        for previous_waypoint, next_waypoint in zip(self.waypoints, self.waypoints[1:]):
            previous_position = previous_waypoint[:2]
            next_position = next_waypoint[:2]
            total_distance += distance(previous_position, next_position).meters

        return total_distance


if __name__ == "__main__":
    mission = Mission()
    print(len(mission.waypoints))
    print(mission.get_total_distance())
