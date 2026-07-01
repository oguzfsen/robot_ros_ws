#!/usr/bin/env python3

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional, Sequence, Union

from geopy.distance import distance

from robot_core.Tools import clamp_int


WaypointLike = Union["Waypoint", Sequence[Union[float, bool]]]


@dataclass(frozen=True)
class Waypoint:
    latitude: float
    longitude: float
    is_area_scan: bool = False
    source_label: str = ""
    overlay_type: str = ""

    @property
    def position(self) -> tuple[float, float]:
        return (self.latitude, self.longitude)

    def as_list(self) -> list[Union[float, bool]]:
        return [self.latitude, self.longitude, self.is_area_scan]

    def as_dict(self) -> dict[str, Any]:
        return {
            "latitude": self.latitude,
            "longitude": self.longitude,
            "is_area_scan": self.is_area_scan,
            "source_label": self.source_label,
            "overlay_type": self.overlay_type,
        }


class Mission:
    """
    Mission JSON verisini otonom surus icin sirali waypoint listesine cevirir.

    Public arayuz:
    - from_file(...)
    - waypoint_count
    - is_completed
    - target_wp
    - next_waypoint()
    - next_wp()
    - has_next_waypoint()
    - get_current_target()
    - get_previous_waypoint()
    - set_index(...)
    - mark_completed()
    - distance_to_target(...)
    - is_target_reached(...)
    - advance_if_target_reached(...)
    - remaining_distance(...)
    - calculate_completion_rate(...)
    - calculate_distance(...)
    - serialize_waypoints()
    - to_waypoint_lists()
    - to_waypoint_dicts()
    - to_storage_dict(...)
    - summary()

    Internal arayuz:
    - Alt cizgi ile baslayan tum fonksiyonlar (`_parse_home`, `_refresh_progress_cache` gibi)
      sinif ici yardimci fonksiyonlardir. Disaridan cagrilmamalari gerekir.
    """

    def __init__(
        self,
        raw_json: str,
        target_sufficient_distance: float = 1.0,
        last_waypoint: int = 0,
    ):
        if target_sufficient_distance <= 0.0:
            raise ValueError("target_sufficient_distance must be greater than zero")

        self.raw_json = raw_json
        self.target_sufficient_distance = float(target_sufficient_distance)

        self.cloud_id = 0
        self.name = ""

        self.home_lat = 0.0
        self.home_lon = 0.0
        self.paths: list[dict[str, Any]] = []
        self.areas: list[dict[str, Any]] = []
        self.routes: list[dict[str, Any]] = []

        self.waypoints: list[Waypoint] = []
        self.remaining_wps: list[Waypoint] = []
        self.total_distance = 0.0
        self.completion_rate = 0
        self.index = 0
        self.route_warnings: list[str] = []

        self.read_mission_json(self.raw_json)
        self.set_index(last_waypoint)

    @classmethod
    def from_file(
        cls,
        file_path: Union[str, Path],
        target_sufficient_distance: float = 1.0,
        last_waypoint: int = 0,
    ) -> "Mission":
        mission_path = Path(file_path)
        mission_text = mission_path.read_text(encoding="utf-8")
        return cls(
            mission_text,
            target_sufficient_distance=target_sufficient_distance,
            last_waypoint=last_waypoint,
        )

    @property
    def waypoint_count(self) -> int:
        return len(self.waypoints)

    @property
    def is_completed(self) -> bool:
        return self.index >= self.waypoint_count

    @property
    def target_wp(self) -> Optional[Waypoint]:
        """core_node icinde `mission.target_wp` seklinde erisim icin kisa yol."""
        return self.get_current_target()

    def read_mission_json(self, mission_json: str) -> None:
        mission = json.loads(mission_json)

        self.cloud_id = int(mission.get("ID", 0))
        self.name = str(mission.get("isim", ""))
        self.home_lat, self.home_lon = self._parse_home(mission.get("home", [0.0, 0.0]))
        self.paths = list(mission.get("yollar", []))
        self.areas = list(mission.get("araziler", []))
        self.routes = list(mission.get("rotalar", []))

        self.fill_waypoints()
        self.total_distance = self.calculate_distance(self.waypoints)

    def fill_waypoints(self) -> None:
        """Mission JSON icindeki rota sirasini tek bir waypoint listesine cevirir."""
        self.route_warnings.clear()

        built_waypoints = [
            Waypoint(
                latitude=self.home_lat,
                longitude=self.home_lon,
                is_area_scan=False,
                source_label="HOME_START",
                overlay_type="home",
            )
        ]

        for route in self.routes:
            label = str(route.get("etiket", ""))
            overlay_type = str(route.get("overlayType", ""))
            reverse = bool(route.get("reverse", False))

            coordinates = self._resolve_route_coordinates(label, overlay_type)
            if reverse:
                coordinates.reverse()

            is_area_scan = overlay_type == "polygon"
            built_waypoints.extend(
                Waypoint(
                    latitude=float(latitude),
                    longitude=float(longitude),
                    is_area_scan=is_area_scan,
                    source_label=label,
                    overlay_type=overlay_type,
                )
                for latitude, longitude in coordinates
            )

        built_waypoints.append(
            Waypoint(
                latitude=self.home_lat,
                longitude=self.home_lon,
                is_area_scan=False,
                source_label="HOME_FINISH",
                overlay_type="home",
            )
        )

        self.waypoints = built_waypoints
        self._refresh_progress_cache()

    def next_waypoint(self) -> Optional[Waypoint]:
        if not self.has_next_waypoint():
            return None

        waypoint = self.waypoints[self.index]
        self.set_index(self.index + 1)
        return waypoint

    def next_wp(self) -> list[Union[float, bool]]:
        waypoint = self.next_waypoint()
        if waypoint is None:
            raise IndexError("mission has no remaining waypoint")
        return waypoint.as_list()

    def has_next_waypoint(self) -> bool:
        return self.index < self.waypoint_count

    def get_current_target(self) -> Optional[Waypoint]:
        if not self.has_next_waypoint():
            return None
        return self.waypoints[self.index]

    def get_previous_waypoint(self) -> Optional[Waypoint]:
        if self.index <= 0 or not self.waypoints:
            return None
        previous_index = clamp_int(self.index - 1, 0, self.waypoint_count - 1)
        return self.waypoints[previous_index]

    def set_index(self, index: int) -> None:
        self.index = clamp_int(int(index), 0, self.waypoint_count)
        self._refresh_progress_cache()

    def mark_completed(self) -> None:
        self.set_index(self.waypoint_count)

    def distance_to_target(
        self,
        current_lat: float,
        current_lon: float,
        target_index: Optional[int] = None,
    ) -> Optional[float]:
        target = self._resolve_target_waypoint(target_index)
        if target is None:
            return 0.0 if self.is_completed else None

        return distance((current_lat, current_lon), target.position).meters

    def is_target_reached(
        self,
        current_lat: float,
        current_lon: float,
        target_index: Optional[int] = None,
        sufficient_distance: Optional[float] = None,
    ) -> bool:
        """Robotun aktif hedefe yeterince yaklasip yaklasmadigini kontrol eder."""
        target_distance = self.distance_to_target(
            current_lat=current_lat,
            current_lon=current_lon,
            target_index=target_index,
        )
        if target_distance is None:
            return False

        threshold = (
            self.target_sufficient_distance
            if sufficient_distance is None
            else float(sufficient_distance)
        )
        return target_distance <= threshold

    def advance_if_target_reached(
        self,
        current_lat: float,
        current_lon: float,
        sufficient_distance: Optional[float] = None,
    ) -> bool:
        """Hedefe ulasildiysa mission indeksini bir sonraki waypoint'e tasir."""
        if not self.is_target_reached(
            current_lat=current_lat,
            current_lon=current_lon,
            sufficient_distance=sufficient_distance,
        ):
            return False

        if self.has_next_waypoint():
            self.set_index(self.index + 1)
        return True

    def remaining_distance(
        self,
        current_lat: Optional[float] = None,
        current_lon: Optional[float] = None,
    ) -> float:
        if self.is_completed:
            return 0.0

        remaining_waypoints = self.waypoints[self.index:]
        if not remaining_waypoints:
            return 0.0

        remaining_distance = self.calculate_distance(remaining_waypoints)
        if current_lat is None or current_lon is None:
            return remaining_distance

        return distance(
            (current_lat, current_lon),
            remaining_waypoints[0].position,
        ).meters + self.calculate_distance(remaining_waypoints)

    def calculate_completion_rate(
        self,
        all_waypoints: Optional[Sequence[WaypointLike]] = None,
        index: Optional[int] = None,
    ) -> int:
        if all_waypoints is None:
            all_waypoints = self.waypoints

        if index is None:
            index = self.index

        if len(all_waypoints) == 0:
            return 0

        clamped_index = clamp_int(index, 0, len(all_waypoints))
        total_distance = self.calculate_distance(all_waypoints)

        if clamped_index >= len(all_waypoints):
            return 100

        if total_distance <= 0.0:
            return 0

        travelled_distance = self.calculate_distance(all_waypoints[:clamped_index])
        completion_rate = int((travelled_distance / total_distance) * 100)
        return clamp_int(completion_rate, 0, 100)

    def calculate_distance(
        self,
        waypoints: Optional[Sequence[WaypointLike]] = None,
    ) -> float:
        if waypoints is None:
            waypoints = self.waypoints

        total_distance = 0.0
        for previous_waypoint, next_waypoint in zip(waypoints, waypoints[1:]):
            previous_position = self._waypoint_position(previous_waypoint)
            next_position = self._waypoint_position(next_waypoint)
            total_distance += distance(previous_position, next_position).meters

        return total_distance

    def serialize_waypoints(self) -> str:
        return json.dumps(self.to_waypoint_lists(), ensure_ascii=False)

    def to_waypoint_lists(self) -> list[list[Union[float, bool]]]:
        return [waypoint.as_list() for waypoint in self.waypoints]

    def to_waypoint_dicts(self) -> list[dict[str, Any]]:
        return [waypoint.as_dict() for waypoint in self.waypoints]

    def to_storage_dict(
        self,
        receive_date: str,
        last_waypoint: Optional[int] = None,
        is_completed: Optional[bool] = None,
    ) -> dict[str, Any]:
        """Mission'i sade database/service kaydina donusturur."""
        mission_completed = self.is_completed
        if is_completed is not None:
            mission_completed = mission_completed or bool(is_completed)
        persisted_last_waypoint = self.index if last_waypoint is None else int(last_waypoint)
        if mission_completed:
            persisted_last_waypoint = self.waypoint_count

        return {
            "mission_id": self.cloud_id,
            "receive_date": receive_date,
            "raw_json": self.raw_json,
            "last_waypoint": max(0, persisted_last_waypoint),
        }

    def summary(self) -> dict[str, Any]:
        current_target = self.get_current_target()
        return {
            "mission_id": self.cloud_id,
            "name": self.name,
            "waypoint_count": self.waypoint_count,
            "current_index": self.index,
            "completion_rate": self.completion_rate,
            "total_distance": self.total_distance,
            "target_sufficient_distance": self.target_sufficient_distance,
            "is_completed": self.is_completed,
            "current_target": None if current_target is None else current_target.as_dict(),
            "route_warnings": list(self.route_warnings),
        }

    def _parse_home(self, home_value: Sequence[float]) -> tuple[float, float]:
        if len(home_value) < 2:
            raise ValueError("mission home field must contain latitude and longitude")

        return (float(home_value[0]), float(home_value[1]))

    def _resolve_route_coordinates(
        self,
        label: str,
        overlay_type: str,
    ) -> list[list[float]]:
        if overlay_type == "polyline":
            route = next((item for item in self.paths if item.get("etiket") == label), None)
            if route is None:
                self.route_warnings.append(f"Polyline route not found: {label}")
                return []
            return list(route.get("konumlar", []))

        if overlay_type == "polygon":
            route = next((item for item in self.areas if item.get("etiket") == label), None)
            if route is None:
                self.route_warnings.append(f"Polygon route not found: {label}")
                return []
            return list(route.get("generatedRoute", []))

        self.route_warnings.append(f"Unsupported overlay type: {overlay_type} ({label})")
        return []

    def _resolve_target_waypoint(self, target_index: Optional[int]) -> Optional[Waypoint]:
        if not self.waypoints:
            return None

        resolved_index = self.index if target_index is None else int(target_index)
        if resolved_index < 0 or resolved_index >= self.waypoint_count:
            return None

        return self.waypoints[resolved_index]

    def _refresh_progress_cache(self) -> None:
        self.remaining_wps = self.waypoints[self.index:]
        self.completion_rate = self.calculate_completion_rate(self.waypoints, self.index)

    def _waypoint_position(self, waypoint: WaypointLike) -> tuple[float, float]:
        if isinstance(waypoint, Waypoint):
            return waypoint.position

        if len(waypoint) < 2:
            raise ValueError("waypoint must contain at least latitude and longitude")

        return (float(waypoint[0]), float(waypoint[1]))


if __name__ == "__main__":
    print("=== Mission Showcase ===")
    print(
        "Bu bolum bir mini kullanim kilavuzudur. "
        "Mission sinifini core_node ve database_node icinde nasil kullanacagini gosterir."
    )

    mission_path = Path(__file__).with_name("utaem_mission.json")
    mission = Mission.from_file(mission_path)

    print("\n[1] Mission olusturma")
    print("mission = Mission.from_file('utaem_mission.json')")
    print(json.dumps(mission.summary(), ensure_ascii=False, indent=2))
    print(f"Toplam mesafe: {mission.total_distance:.2f} m")

    print("\n[2] Public alanlar ve property'ler")
    print(f"mission.cloud_id = {mission.cloud_id}")
    print(f"mission.name = {mission.name}")
    print(f"mission.waypoint_count = {mission.waypoint_count}")
    print(f"mission.index = {mission.index}")
    print(f"mission.is_completed = {mission.is_completed}")
    print(f"mission.target_sufficient_distance = {mission.target_sufficient_distance}")

    current_target = mission.target_wp
    if current_target is not None:
        print(f"mission.target_wp = {current_target.as_dict()}")

    print("\n[3] core_node benzeri kullanim")
    print(
        "Asagidaki akis tipik olarak otonom suruste her kontrol dongusunda kullanilabilir:"
    )
    print(
        """
raw_json = mission_path.read_text(encoding="utf-8")
mission = Mission(raw_json, target_sufficient_distance=1.0)

if mission.target_wp is not None:
    target_lat = mission.target_wp.latitude
    target_lon = mission.target_wp.longitude

robot_lat = mission.home_lat
robot_lon = mission.home_lon

reached = mission.is_target_reached(robot_lat, robot_lon)
print(reached)

if mission.advance_if_target_reached(robot_lat, robot_lon):
    print("Sonraki waypoint'e gecildi")
        """.strip()
    )

    print("\n[4] Adim adim waypoint ilerletme")
    step_mission = Mission.from_file(mission_path)
    print(f"Baslangic hedefi: {step_mission.target_wp.as_list()}")
    consumed_wp = step_mission.next_waypoint()
    print(f"next_waypoint() -> {consumed_wp.as_dict()}")
    print(f"Yeni index: {step_mission.index}")
    print(f"Yeni hedef: {step_mission.target_wp.as_dict()}")

    print("\n[5] Konuma gore hedef kontrolu")
    near_target_mission = Mission.from_file(mission_path)
    first_target = near_target_mission.target_wp
    if first_target is not None:
        distance_m = near_target_mission.distance_to_target(
            first_target.latitude,
            first_target.longitude,
        )
        print(f"Hedefe mesafe: {distance_m:.3f} m")
        print(
            "is_target_reached(...) = "
            f"{near_target_mission.is_target_reached(first_target.latitude, first_target.longitude)}"
        )
        near_target_mission.advance_if_target_reached(
            first_target.latitude,
            first_target.longitude,
        )
        print(f"Ilerleme sonrasi index: {near_target_mission.index}")

    print("\n[6] database_node benzeri kullanim")
    print("Mission nesnesi veritabanina yazilacak sade kayda tek satirda donusturulebilir:")
    storage_record = mission.to_storage_dict(receive_date="2026-07-01T00:00:00Z")
    print(json.dumps(storage_record, ensure_ascii=False, indent=2))
    print(
        "database_node icinde mission_set callback'i tam olarak bu mantigi kullanir: "
        "raw_json -> Mission(...) -> to_storage_dict(...)"
    )

    print("\n[7] Public ve internal notu")
    print(
        "Disaridan kullanman gereken fonksiyonlar alt cizgisiz olanlardir. "
        "Alt cizgi ile baslayanlar (_resolve_target_waypoint gibi) sinif ici yardimci fonksiyonlardir."
    )

    print("\n[8] Onerilen kullanim ozeti")
    print(
        """
core_node:
- Mission(raw_json) ile mission olustur.
- Aktif hedefi mission.target_wp ile al.
- Robot konumu geldikce mission.is_target_reached(...) ile kontrol et.
- Hedefe varinca mission.advance_if_target_reached(...) cagir.
- UI veya status icin mission.completion_rate, mission.remaining_distance() ve mission.is_completed kullan.

database_node:
- Yeni mission geldiginde Mission(raw_json) olustur.
- mission.to_storage_dict(...) ile DB kaydina cevir.
- MissionGet cevabinda mission_id, receive_date, raw_json ve last_waypoint alanlarini geri don.
- Waypoint gecildikce mission_progress_set servisi ile last_waypoint guncellenir.
        """.strip()
    )
