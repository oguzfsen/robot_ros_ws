#!/usr/bin/env python3

import json

class Mission():
    def __init__(self):

        self.filename = "utaem_mission.json"
        self.cloud_id = 0
        self.name = ""

        self.home_lat = 0.0
        self.home_lon = 0.0
        self.paths = []

        self.waypoints = []

        with open(self.filename, "r", encoding="utf-8") as file:
            fc = file.read()
            self.read_mission_json(fc)
        

    def read_mission_json(self, mission_json: str) -> None:
        mission = json.loads(mission_json)

        self.cloud_id = int(mission["ID"])
        self.name = str(mission["isim"])
        self.paths = list(mission["yollar"])
        self.areas = list(mission["araziler"])
        self.routes = list(mission["rotalar"])
        self.home_lat = mission["home"][0]
        self.home_lon = mission["home"][1]

        self.waypoints.extend([self.home_lat, self.home_lon, False])

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

        self.waypoints.extend([self.home_lat, self.home_lon, False])

        

a = Mission()
print(len(a.waypoints))








