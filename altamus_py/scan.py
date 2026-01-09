from enum import Flag
import simplejson
import struct
import math
import sys
from pathlib import Path
import argparse
from dataclasses import dataclass
import altamus_py.mavlink as mavlink
import numpy as np
from pypcd4.pypcd4 import PointCloud, Encoding


class PointFlags(Flag):
    HEALTHY = 1
    NO_RESPONSE = 2
    NO_RETURN = 4
    TOO_FAR = 8
    TOO_CLOSE = 16
    OUTLIER = 32


@dataclass
class Preamble:
    version: int | None = None
    preamble_stop: int | None = None
    notes_start: int | None = None
    notes_stop: int | None = None
    header_start: int | None = None
    header_stop: int | None = None
    points_start: int | None = None
    points_stop: int | None = None

    @classmethod
    def parse_from_bytes(cls, data: bytes):
        cursor = 0

        def read_and_increment():
            nonlocal cursor
            res = int.from_bytes(data[cursor:cursor + 4], byteorder='little')
            cursor += 4
            return res

        preamble = Preamble()
        preamble.version = read_and_increment()
        preamble.preamble_stop = read_and_increment()
        preamble.notes_start = read_and_increment()
        preamble.notes_stop = read_and_increment()
        preamble.header_start = read_and_increment()
        preamble.header_stop = read_and_increment()
        preamble.points_start = read_and_increment()
        preamble.points_stop = read_and_increment()
        return preamble


@dataclass
class Header:
    identifier: mavlink.MAVLink_identifier_message | None = None
    scan_settings: mavlink.MAVLink_scan_settings_message | None = None
    scan_transform: mavlink.MAVLink_scan_transform_message | None = None
    lidar_settings: mavlink.MAVLink_lidar_settings_message | None = None
    pitch_motor_settings: mavlink.MAVLink_motor_settings_message | None = None
    yaw_motor_settings: mavlink.MAVLink_motor_settings_message | None = None
    scan_result: mavlink.MAVLink_scan_result_info_message | None = None
    orientation: mavlink.MAVLink_orientation_message | None = None
    average_power: mavlink.MAVLink_power_information_message | None = None
    minimum_power: mavlink.MAVLink_power_information_message | None = None
    maximum_power: mavlink.MAVLink_power_information_message | None = None

    @classmethod
    def parse_from_bytes(cls, data: bytes, dialect_module):
        header = Header()
        parser = dialect_module.MAVLink(", 1, 1")
        parsed_mavlink = parser.parse_buffer(data)
        if parsed_mavlink is None:
            print("No mavlink parsed, returning empty header")
            return header

        # Loop through the mavlink messages and extract to a "nice" json structure
        for chapter in parsed_mavlink:
            if isinstance(chapter, dialect_module.MAVLink_identifier_message):
                header.identifier = chapter
            elif isinstance(chapter, dialect_module.MAVLink_scan_settings_message):
                header.scan_settings = chapter
            elif isinstance(chapter, dialect_module.MAVLink_scan_transform_message):
                header.scan_transform = chapter
            elif isinstance(chapter, dialect_module.MAVLink_lidar_settings_message):
                header.lidar_settings = chapter
            elif isinstance(chapter, dialect_module.MAVLink_motor_settings_message):
                if (chapter.motor == dialect_module.EOS_COMPONENT_PITCH_MOTOR):
                    header.pitch_motor_settings = chapter
                if (chapter.motor == dialect_module.EOS_COMPONENT_YAW_MOTOR):
                    header.yaw_motor_settings = chapter
            elif isinstance(chapter, dialect_module.MAVLink_scan_result_info_message):
                header.scan_result = chapter
            elif isinstance(chapter, dialect_module.MAVLink_orientation_message):
                header.orientation = chapter
            elif isinstance(chapter, dialect_module.MAVLink_power_information_message):
                if chapter.type == dialect_module.POWER_INFORMATION_TYPE_AVERAGE:
                    header.average_power = chapter
                elif chapter.type == dialect_module.POWER_INFORMATION_TYPE_MINIMUM:
                    header.minimum_power = chapter
                elif chapter.type == dialect_module.POWER_INFORMATION_TYPE_MAXIMUM:
                    header.maximum_power = chapter
        return header

    def to_json(self) -> str:
        return "TBI"

    def to_dict_annotated(self) -> dict:
        j = {}
        for field in self.__dataclass_fields__:
            value = getattr(self, field)
            if value is None:
                continue
            chapter = value.to_dict()
            key_name = chapter['mavpackettype']
            # there are multiple MOTOR_SETTINGS messages, override the dict key here based on the 'motor' field
            if key_name == 'MOTOR_SETTINGS':
                if chapter['motor'] == mavlink.EOS_COMPONENT_PITCH_MOTOR:
                    key_name = "EOS_COMPONENT_PITCH_MOTOR"
                elif chapter['motor'] == mavlink.EOS_COMPONENT_YAW_MOTOR:
                    key_name = "EOS_COMPONENT_YAW_MOTOR"
            # there are also multiple POWER_INFORMATION
            if key_name == 'POWER_INFORMATION':
                if chapter['type'] == mavlink.POWER_INFORMATION_TYPE_AVERAGE:
                    key_name = "POWER_INFORMATION_TYPE_AVERAGE"
                elif chapter['type'] == mavlink.POWER_INFORMATION_TYPE_MINIMUM:
                    key_name = "POWER_INFORMATION_TYPE_MINIMUM"
                elif chapter['type'] == mavlink.POWER_INFORMATION_TYPE_MAXIMUM:
                    key_name = "POWER_INFORMATION_TYPE_MAXIMUM"
            j[key_name] = chapter

        return j


@dataclass
class RawPolarPoint:
    distance_cm: int
    pitch: int
    yaw: int
    return_strength: int

    def to_list(self):
        return [self.distance_cm, self.pitch, self.yaw, self.return_strength]

    def to_cartesian(self, roll_offset_deg: float, pitch_offset_deg: float, yaw_scale: float, pitch_scale: float):
        flags = PointFlags.HEALTHY
        if self.distance_cm == 65535:
            # -2, no reply at all from lidar
            flags = PointFlags.NO_RESPONSE
        elif self.distance_cm == 0:
            # no reading from lidar, pointted at sky
            flags = PointFlags.NO_RETURN
        elif self.distance_cm < 100:
            # Hard coded minimum distance below which we shouldn't show
            flags = PointFlags.TOO_CLOSE
        elif self.distance_cm > 15000:
            # Hard coded maximum distance, 150 meters. Above this, dont show.
            flags = PointFlags.TOO_FAR

        if PointFlags.HEALTHY in flags:
            radius_meters = self.distance_cm / 100.0
        else:
            # Doing this will project all the error points onto a 50 cm sphere about the origin, allowing the ability to see where in the orbit points are failing
            radius_meters = 0.5

        pitch_radians = self.pitch / 10000
        yaw_radians = self.yaw / 10000
        roll_offset_radians = math.radians(roll_offset_deg)
        pitch_offset_radians = math.radians(pitch_offset_deg)

        pitch_adjusted_radians = pitch_radians + pitch_offset_radians

        x = math.cos(yaw_radians) * math.sin(pitch_adjusted_radians) * math.cos(
            roll_offset_radians) + math.sin(yaw_radians) * math.sin(roll_offset_radians)
        y = math.sin(yaw_radians) * math.sin(pitch_adjusted_radians) * math.cos(
            roll_offset_radians) - math.cos(yaw_radians) * math.sin(roll_offset_radians)
        z = math.cos(pitch_adjusted_radians) * math.cos(roll_offset_radians)

        x = x * radius_meters
        y = y * radius_meters
        z = z * radius_meters

        return CartesianPoint(x, y, z, self.return_strength, flags)


@dataclass
class CartesianPoint:
    x: float
    y: float
    z: float
    strength: float
    point_flags: PointFlags

class EOSV2Scan:
    preamble: Preamble
    header: Header
    notes = ""
    polar_points: list[RawPolarPoint] = []
    cartesian_points: list[CartesianPoint] = []
    pcd = None

    @property
    def invalid_points_count(self) -> int:
        res = 0
        for point in self.cartesian_points:
            if PointFlags.NO_RETURN in point.point_flags:
                res += 1
        return res
    
    @property
    def error_points_count(self) -> int:
        res = 0
        for point in self.cartesian_points:
            if PointFlags.NO_RESPONSE in point.point_flags:
                res += 1
        return res

    @property
    def too_close_points_count(self) -> int:
        res = 0
        for point in self.cartesian_points:
            if PointFlags.TOO_CLOSE in point.point_flags:
                res += 1
        return res

    @property
    def points_count(self) -> int:
        return len(self.cartesian_points)

    def parse_polar_points_from_bytes(self, data: bytes):
        self.polar_points.clear()

        # parse raw bytes into polar points
        for i in range(int(len(data) / 8)):
            point_bytes = data[(8*i):(8*i) + 8]
            dist, pitch_raw, yaw_raw, strength = struct.unpack_from(
                "<HHHH", point_bytes)
            pitch = pitch_raw
            yaw = yaw_raw
            polar_point = RawPolarPoint(dist, pitch, yaw, strength)
            self.polar_points.append(polar_point)

    def create_cartesian_points_from_polar(self):
        self.cartesian_points.clear()
        roll_offet = 0
        pitch_offset = 0
        pitch_scale = 1.0
        yaw_scale = 1.0

        if self.header is not None:
            if self.header.scan_transform is not None:
                transform = self.header.scan_transform
                roll_offet = transform.roll_offset
                pitch_offset = transform.pitch_offset
                pitch_scale = transform.pitch_scale
                yaw_scale = transform.yaw_scale

        for polar_point in self.polar_points:
            self.cartesian_points.append(polar_point.to_cartesian(
                roll_offset_deg=roll_offet, pitch_offset_deg=pitch_offset, yaw_scale=yaw_scale, pitch_scale=pitch_scale))


    # Parse polar points into cartesian points and save to a pcd

    def make_pcd(self, filename: str, with_polar: bool = True, with_invalid_points=True):
        # All versions have xyzi fields
        fields = ("x", "y", "z", "intensity")
        types = (np.float32, np.float32, np.float32, np.int16)
        arr = []

        # Add polar values if requested
        if with_polar:
            fields = fields + ("pitch", "yaw", "distance",)
            types = types + (np.uint16, np.uint16, np.uint16,)

        # Add flags if requested
        if with_invalid_points:
            fields = fields + ("flags",)
            types = types + (np.uint8,)

        for index, c_point in enumerate(self.cartesian_points):
            point = None
            point = [c_point.x, c_point.y, c_point.z,
                     c_point.strength]  # default values
            if PointFlags.HEALTHY not in c_point.point_flags and not with_invalid_points:
                # don't add the point if it's not healthy and we've been asked to ignore it.
                continue

            # Add polar values if requested
            if with_polar:
                p_point = self.polar_points[index]
                point.extend([p_point.pitch, p_point.yaw, p_point.distance_cm])

            # Add flags if requested
            if with_invalid_points:
                point.extend([c_point.point_flags.value])

            if point is not None:
                arr.append(point)

        np_arr = np.array(arr)
        pcd = PointCloud.from_points(np_arr, fields, types)
        pcd.save(filename, encoding=Encoding.BINARY_COMPRESSED)

    @classmethod
    def from_binfile(cls, path_to_file):
        with open(path_to_file, "rb") as file:
            content = file.read()
        return EOSV2Scan._parse_binfile_data(content)

    @classmethod
    def from_stdin(cls, data):
        return EOSV2Scan._parse_binfile_data(data)

    @classmethod
    def _parse_binfile_data(cls, data):
        scan = EOSV2Scan()

        # Parse preamble from binfile
        preamble = Preamble.parse_from_bytes(data)
        scan.preamble = preamble

        # If preamble has header start/stop, parse the header
        if (preamble.header_start is not None and preamble.header_stop is not None):
            scan.header = Header.parse_from_bytes(
                data[preamble.header_start:preamble.header_stop + 1], mavlink)

        # If preamble has notes start/stop, parse them
        if (preamble.notes_start is not None and preamble.notes_stop is not None):
            notes = data[preamble.notes_start: preamble.notes_stop + 1]
            scan.notes = notes.decode().rstrip('\x00')

        # if preamble has points start, parse from there to the end of the file
        if (preamble.points_start is not None):
            scan.parse_polar_points_from_bytes(data[preamble.points_start:])
            scan.create_cartesian_points_from_polar()

        return scan

    def to_json(self) -> str:
        d = self.header.to_dict_annotated()
        polar_points_list = []
        for point in self.polar_points:
            polar_points_list.append(point.to_list())
        d["NOTES"] = self.notes
        d["POINTS"] = polar_points_list
        j = simplejson.dumps(d, ignore_nan=True)
        return j
