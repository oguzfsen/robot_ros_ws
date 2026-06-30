#!/usr/bin/env python3

import json
import os
import sqlite3
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from Tools import _get_param

from robot_interfaces.srv import (
    EncoderZeroGet,
    EncoderZeroSet,
    MissionGet,
    MissionSet,
)

class DatabaseNode(Node):
    def __init__(self):
        super().__init__("database_node", automatically_declare_parameters_from_overrides=True)

        self.db_path = self.resolve_db_path(_get_param(self, "db_path", "~/.ros/robot_brain.db"))
        tables_json = _get_param(self, "tables_json", None)
        self.tables = json.loads(tables_json)

        self.service_encoder_zero_get = self.create_service(
            EncoderZeroGet,
            "/database/encoder_zero_get",
            self.encoder_zero_get_callback,
        )

        self.service_encoder_zero_set = self.create_service(
            EncoderZeroSet,
            "/database/encoder_zero_set",
            self.encoder_zero_set_callback,
        )

        self.mission_get = self.create_service(
            MissionGet,
            "/database/mission_get",
            self.mission_get_callback,
        )

        self.mission_set = self.create_service(
            MissionSet,
            "/database/mission_set",
            self.mission_set_callback,
        )

        self.ensure_database()

        self.get_logger().info("database_node started")


    def encoder_zero_get_callback(self, request, response):
        del request

        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()

            cursor.execute(
                "SELECT motor1, motor2, motor3, motor4 FROM encoder_zero_raw WHERE id = 1"
            )
            row = cursor.fetchone()
            conn.close()

            if row is None:
                response.success = False
                response.message = "encoder_zero_raw row not found"
                response.zero_raw = []
                return response

            response.success = True
            response.message = "encoder zero raw values read"
            response.zero_raw = [int(row[0]), int(row[1]), int(row[2]), int(row[3])]
            return response

        except Exception as exc:
            response.success = False
            response.message = f"encoder zero get failed: {exc}"
            response.zero_raw = []
            return response

    def encoder_zero_set_callback(self, request, response):
        try:
            motor_id = int(request.motor_id)
            raw_value = int(request.raw_value)

            if motor_id < 1 or motor_id > 4:
                response.success = False
                response.message = "motor_id must be between 1 and 4"
                return response

            if raw_value < 0 or raw_value > 4095:
                response.success = False
                response.message = "raw_value must be between 0 and 4095"
                return response

            column_name = f"motor{motor_id}"

            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()

            cursor.execute(
                f"UPDATE encoder_zero_raw SET {column_name} = ? WHERE id = 1",
                (raw_value,),
            )

            if cursor.rowcount == 0:
                cursor.execute(
                    "INSERT INTO encoder_zero_raw (id, motor1, motor2, motor3, motor4) VALUES (1, 0, 0, 0, 0)"
                )
                cursor.execute(
                    f"UPDATE encoder_zero_raw SET {column_name} = ? WHERE id = 1",
                    (raw_value,),
                )

            conn.commit()
            conn.close()

            response.success = True
            response.message = f"motor{motor_id} zero raw set to {raw_value}"
            return response

        except Exception as exc:
            response.success = False
            response.message = f"encoder zero set failed: {exc}"
            return response
        

    def mission_get_callback(self, request, response):
        ...

    def mission_set_callback(self, request, response):
        ...


#region General Functions

    def resolve_db_path(self, db_path: str) -> str:
        """Expand the configured database path and create its parent directory."""
        resolved_path = os.path.abspath(os.path.expandvars(os.path.expanduser(db_path)))
        parent_dir = os.path.dirname(resolved_path)

        if parent_dir:
            os.makedirs(parent_dir, exist_ok=True)

        return resolved_path
    
    @staticmethod
    def _utc_now_iso() -> str:
        return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace(
            "+00:00",
            "Z",
        )

    def ensure_database(self):
        db_exists = os.path.exists(self.db_path)

        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()

        for table_name, table_def in self.tables.items():
            expected_columns = table_def["columns"]

            if not self.table_exists(cursor, table_name):
                self.create_table(cursor, table_name, expected_columns)
                self.insert_default_row_if_needed(cursor, table_name, expected_columns)
                self.get_logger().info(f"Created table: {table_name}")
                continue

            if not self.table_schema_matches(cursor, table_name, expected_columns):
                self.get_logger().warn(f"Schema mismatch detected: {table_name}")
                cursor.execute(f"DROP TABLE IF EXISTS {table_name}")
                self.create_table(cursor, table_name, expected_columns)
                self.insert_default_row_if_needed(cursor, table_name, expected_columns)
                self.get_logger().info(f"Recreated table: {table_name}")

        conn.commit()
        conn.close()

        if db_exists:
            self.get_logger().info(f"Database checked: {self.db_path}")
        else:
            self.get_logger().info(f"Database created: {self.db_path}")

    def table_exists(self, cursor, table_name: str) -> bool:
        cursor.execute(
            "SELECT name FROM sqlite_master WHERE type='table' AND name=?",
            (table_name,),
        )
        return cursor.fetchone() is not None

    def create_table(self, cursor, table_name: str, columns: list):
        column_defs = []

        for col in columns:
            name = col["name"]
            col_type = col["type"]
            column_defs.append(f"{name} {col_type}")

        sql = f"CREATE TABLE {table_name} ({', '.join(column_defs)})"
        cursor.execute(sql)

    def table_schema_matches(self, cursor, table_name: str, expected_columns: list) -> bool:
        """
        Check whether the existing table has the expected column names and base types.

        This function intentionally ignores PRIMARY KEY formatting differences,
        because SQLite stores primary key information separately.
        """
        cursor.execute(f"PRAGMA table_info({table_name})")
        existing_info = cursor.fetchall()

        existing_columns = []

        for column in existing_info:
            name = column[1]
            column_type = str(column[2]).upper().strip()
            pk = int(column[5])

            existing_columns.append({
                "name": name,
                "type": column_type,
                "pk": pk,
            })

        expected_normalized = []

        for column in expected_columns:
            name = column["name"]
            column_type = str(column["type"]).upper().strip()

            is_primary_key = "PRIMARY KEY" in column_type
            base_type = column_type.replace("PRIMARY KEY", "").strip()

            expected_normalized.append({
                "name": name,
                "type": base_type,
                "pk": 1 if is_primary_key else 0,
            })

        if len(existing_columns) != len(expected_normalized):
            return False

        for existing, expected in zip(existing_columns, expected_normalized):
            if existing["name"] != expected["name"]:
                return False

            if existing["type"] != expected["type"]:
                return False

            if expected["pk"] == 1 and existing["pk"] != 1:
                return False

        return True

    def insert_default_row_if_needed(self, cursor, table_name: str, columns: list):
        has_defaults = any("default" in col for col in columns)

        if not has_defaults:
            return

        names = []
        values = []

        for col in columns:
            if "default" in col:
                names.append(col["name"])
                values.append(col["default"])

        placeholders = ", ".join(["?"] * len(values))
        sql = f"INSERT INTO {table_name} ({', '.join(names)}) VALUES ({placeholders})"
        cursor.execute(sql, values)
#endregion



def main(args=None):
    rclpy.init(args=args)

    node = DatabaseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
